import pdb
import time

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix, quaternion_slerp
from gps.utility.general_utils import get_ee_points
from gps.algorithm.policy.lin_gauss_init import init_pd_ref

from gps.agent.ros.cad.agent_cad import AgentCAD
from gps.agent.ros.cad.util import *


class AgentCADExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
        self.attempts = 70 # Make 70 plans 
        pdb.set_trace()     # for optional setup, not debugging

    def setup(self):
        self.configure_scene()
        self.grasp_prep()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        print 'Resetting piece'
        self.reset_piece()

        print 'Adding objects to planning scene'
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.5,0.,0.42], size=[0.7,1.5,0.03], type='box')

        for name in ('held_piece', 'fixed_piece'):
            pose = self.get_pose(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.05,0.05,0.0254),
                    filename=self._hyperparams['cad_path'])

    def reset_piece(self):
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def grasp_prep(self):
        self.use_controller('MoveIt')
        self.ungrip(None)

        target_position = listify(self.get_pose('held_piece').position)
        target_position[0] -= 0.3
        target_position[2] += 0.01
        target_pose = [0,0,0]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

        self.ungrip(15)

        target_position[0] += 0.1
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

    def grasp(self):
        self.grip(None)
        time.sleep(5)
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])

    # Exceptions don't matter because it's fine if some plans fail
    def plan(self, use_plan=None): # Lmao a little override
	try:
		thePlan = super(AgentCADExperiment, self).plan(use_plan)
		return thePlan
	except:
		return None # Just return if it fails

    # Override so we make multiple plans and then choose one with best
    def compute_reference_trajectory(self, condition, policy):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
	best_ref_ee = None # HAHAHAHAHA to store the best one
	best_ref_ja = None # HAHAHAHAHA to store the best one
	best_dist = float("inf") # Infinity itself

        while True:
	    for attempt in range(self.attempts): # Make this many attempts
		    plan = self.plan_end_effector(target['position'], target['orientation'])
		    if plan is None: # Leave this iteration lmao
			continue

		    cur_dist = self.get_dist(plan) # Get the current distance

		    # If it beats it we need to use it! Woah!
                    if cur_dist < best_dist:
		        plan_joints = [np.array(point.positions) for point in plan.joint_trajectory.points]
		        ref_ja = interpolate(plan_joints, self.T_interpolation)
		        ref_ja.extend([ref_ja[-1]] * (self.T - self.T_interpolation))
		        ref_poses = self.forward_kinematics(ref_ja, 'torso_lift_link')
		        ref_ee = []
		        ee_offsets = self._hyperparams['end_effector_points']
		        for pose in ref_poses:
				position, orientation = np.array([pose[:3]]), pose[3:]
				rotation_mat = quaternion_matrix(orientation)[:3,:3]
				points = np.ndarray.flatten(get_ee_points(ee_offsets, position, rotation_mat).T)
				ref_ee.append(points)
                    	best_dist = cur_dist
                        best_ref_ee, best_ref_ja = ref_ee, ref_ja

	    # Plot the very best plan that we found!
            plot_trajectories([best_ref_ee])

            if not self.require_approval or yesno('Does this trajectory look ok?'):
		print("this is the distance of the best one: ")
                print(best_dist)
                break

        self.trajectories[condition] = ref_ee
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja, ref_ee))

