import pdb
import get_plan 
import time

import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix, quaternion_slerp
from gps.utility.general_utils import get_ee_points
from gps.algorithm.policy.lin_gauss_init import init_pd_ref

from gps.agent.ros.cad.agent_cad import AgentCAD
from gps.agent.ros.cad.util import *


class JPieceExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
        self.trajs = [] # Get a list of trajectories and stuff
        self.dists = [] # Get a list of the distances in the trajs
        self.attempts = 50
        self.use_saved_traj = False # If we already have one saved (pickled DisplayTrajectory)
        self.pickled_traj_filename = '' # Where you saved the DisplayTrajectory
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
        self.add_object('table', position=[0.88,0.,0.42], size=[0.7,1.5,0.03], type='box')
       #self.add_object('help_box', position=[0.5,0.,0.42], size=[0.285,0.2384,0.07], type='box')
	the_size = {'j_piece': (0.00015, 0.00015, 0.00015), 'j_box': (0.0002, 0.0002, 0.0002)}

        for name in ['j_piece', 'j_box']:
            pose = self.get_pose(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=the_size[name],
                    filename=self._hyperparams[name])

    def reset_piece(self):
        quat = Quaternion(*quaternion_from_euler(1.571577, 0.011550, -1.524775))
        pose = Pose(Point(0.85, 0.703, 0.193), quat)
        self.set_pose('j_piece', pose)

    def grasp_prep(self):
        self.use_controller('MoveIt')
        self.ungrip(None)

        target_position = listify(self.get_pose('j_piece').position)
	print("This is the target position")
	print(target_position)
	target_position[0] = 0.59
        target_position[1] =  0.2
	target_position[2] =  0.54
        target_pose = [0,0,0]
        self.ungrip(15)
        time.sleep(2)
	init_plan = get_plan.get_start_plan()
        #init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

        #self.ungrip(15)
	#target_position[2] -= 0.05
        #init_plan = self.plan_end_effector(target_position, target_pose)
        #self.group.execute(init_plan)

    def grasp(self):
        self.grip(None)
        time.sleep(5)
        self.attach('j_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
                
    # Can call this in debug/setup mode
    # Makes a motion plan then executes it if possible
    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
	target_position = [pos_x, pos_y, pos_z]
	target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

    # Overrides the reset method because we want to move the box, then reset
    # then move box back so the PR2's hand doesn't get stuck in the box
    def reset(self, condition):
    	#self.move_box_away() # Move the box away
    	super(JPieceExperiment, self).reset(condition) # Call the super method
	#self.move_box_back() # Move the box back into position lmao
	
    # Moves the box in the scene away from the PR2
    def move_box_away(self):
        quat = Quaternion(*quaternion_from_euler(0.012339, -0.007963, 1.60533))
        pose = Pose(Point(1, -0.178, 0.525), quat)
        self.set_pose('j_box', pose)
    
    # Moves the box in the scene back to the original experiment position
    def move_box_back(self):
        quat = Quaternion(*quaternion_from_euler(0.012339, -0.007963, 1.60533))
        pose = Pose(Point(0.81, -0.178, 0.525), quat)
        self.set_pose('j_box', pose)

    # Also override LMAOOOOOOO
    def plan(self, use_plan=None):
        # If already given a plan, just return it
        if use_plan is not None:
                return use_plan
	used_time = 0 # We'll give them a total of 60 seconds
	for time in self.planning_schedule:
	    print 'Planning with {} seconds'.format(time)
	    used_time += time # Increment the time used
	    self.group.set_planning_time(time)
	    plan = self.group.plan()
	    if len(plan.joint_trajectory.points) > 0:
		print 'Success!'
	        self.save_traj(plan) # Save the trajectory
	        dist = self.get_dist(plan) # Get the distance of the plan
		self.dists.append(dist) # Add it to the list
		print('The distance was ' + str(dist) + '\n')
		return plan
	    else:
		print 'Failed.'.format(time)
	if self.indefatigable:
	    print 'Failed to find a valid plan under the given schedule, but trying again'
	    time = 1
	    while used_time <= 80:
		print 'Planning with {} seconds'.format(time)
		self.group.set_planning_time(time)
		plan = self.group.plan()
		if len(plan.joint_trajectory.points) > 0:
		    print 'Success!'
		    self.save_traj(plan) # Save the trajectory
		    dist = self.get_dist(plan) # Get the distance of the plan
		    self.dists.append(dist) # Add it to the list
		    print('The distance was ' + str(dist) + '\n')
		    return plan
		else:
		    print 'Failed.'.format(time)
		used_time += time
		time *= 2
	else:
		return None
	return None

    # Override so we make multiple plans and then choose one with best
    def compute_reference_trajectory(self, condition, policy):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        best_ref_ee = None # HAHAHAHAHA to store the best one
        best_ref_ja = None # HAHAHAHAHA to store the best one
        best_dist = float("inf") # Infinity itself
        best_plan = None 

	plans_made = 0 # Count how many plans were actually created

        while True:

            for attempt in range(self.attempts): # Make this many attempts
                    plan = self.plan_end_effector(target['position'], target['orientation'])
                    if plan is None: # Leave this iteration lmao
                        continue
                    plans_made += 1 # Increment plans made
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
                        best_plan = plan
                        self.best_saved_traj = self.saved_traj # Save the trajectory and stuff


            # Plot the very best plan that we found!
            plot_trajectories([best_ref_ee])

            if not self.require_approval or yesno('Does this trajectory look ok?'):
                print("Of all the " + str(plans_made) + " plans made, ")
                print("this is the distance of the best one: " + str(best_dist))
                break

        if self.use_saved_traj: # If we already have one
            with open(self.pickled_traj_filename, 'r') as f: # Read from the pickled place
                self.best_saved_traj = pickle.load(f) # Load that pickled DisplayTrajectory!

        self.publishDisplayTrajectory(self.best_saved_traj) # Publish it so it is the last

        self.trajectories[condition] = best_ref_ee
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], best_ref_ja, best_ref_ee))

        with open('pickled_robot_traj', 'w') as f:
            pickle.dump(self.best_saved_traj, f)