import pdb
import time
import get_plan
import pickle

import numpy as np
import moveit_msgs.msg
from moveit_msgs.msg import DisplayTrajectory

import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix, quaternion_slerp
from gps.utility.general_utils import get_ee_points
from gps.algorithm.policy.lin_gauss_init import init_pd_ref

from gps.agent.ros.cad.agent_cad import AgentCAD
from gps.agent.ros.cad.util import *

from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker, MarkerArray
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_mechanism_msgs.srv import SwitchController
from tf.transformations import quaternion_matrix, quaternion_slerp
import tf

from gps.agent.ros.agent_ros import AgentROS
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.agent.ros.ros_utils import ServiceEmulator, msg_to_sample, \
        policy_to_msg
from gps.algorithm.policy.lin_gauss_init import init_pd_ref
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM, JOINT_ANGLES, \
        JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, \
        ACTION, TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest
from gps.utility.general_utils import get_ee_points

from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS, REF_OFFSETS, REF_TRAJ

class AgentCADExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
<<<<<<< HEAD
        self.attempts = 20 # Make 20 plans 
        self.use_saved_traj = None # Any saved trajectory??/
        self.ar = {'held_piece': 0, 'fixed_piece': 1} # Number of AR tag they have on them

        # Create the functions with the proper offsets and whatever
        self.ar_functions[self.ar['held_piece']] = self.create_AR_function( \
            self.ar['held_piece'], 0.003, -0.02, -0.0265, 0, 0, -1.57)
            #self.ar['held_piece'], 0.035, 0, -0.0465, 0, 0, -1.57)
        self.ar_functions[self.ar['fixed_piece']] = self.create_AR_function( \
            self.ar['fixed_piece'], 0, -0.025, -0.0325, 0, 0, 0)

=======
        self.attempts = 70 # Make 70 plans 
>>>>>>> c245b97f3cb77668c74789344f57be5d7274da8a
        pdb.set_trace()     # for optional setup, not debugging

    def setup(self):
        self.configure_scene()
        self.grasp_prep()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        #print 'Resetting piece'
        #self.reset_piece()
        table_pose = self.get_AR_pose(2) # Get the AR position of the table
        if not table_pose: # If there isn't an AR marker
            z = 0.72 # Ehh just some random height
        else:
            z = table_pose.position.z # Otherwise get the z coordinate
        print 'Adding objects to planning scene'
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.8,0.,z], size=[0.7,1.5,0.03], type='box')

        for name in ('held_piece', 'fixed_piece'):
            #pose = self.get_pose(name)
            # Get the position of the objects using their AR tags
            pose, euler = self.pose_from_AR(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.045,0.045,0.02286),
                    filename=self._hyperparams['cad_path'])
        self.change_goal() # Change the goal depending on where the goal piece is lmao

    def reset_piece(self):
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def grasp_prep(self):
        self.use_controller('MoveIt')
        self.ungrip(None)

        #target_position = listify(self.get_pose('held_piece').position)
        # Get the position of the held piece using their AR tags
        pose, euler = self.pose_from_AR('held_piece')
        target_position = listify(pose.position) # Get the position as list
        target_position[0] -= 0.23
        target_position[2] += 0
        target_pose = [0,0,0]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

        self.ungrip(15)

    def grasp(self):
        self.grip(None)
        time.sleep(5)
<<<<<<< HEAD
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    # Calculate the goal position depending on where the fixed piece is
    def change_goal(self):
        # Remove the thing from where it was in the rviz scene
        self.scene.remove_world_object('fixed_piece')

        # Get the pose twice just in case or whatever
        pose, euler = self.pose_from_AR('fixed_piece')
        pose, euler = self.pose_from_AR('fixed_piece')
        posi = pose.position # Get the position from the pose

        # Add the fixed piece back to the rviz scene
        self.add_object('fixed_piece', position=listify(pose.position),
            orientation=listify(pose.orientation),
            size=(0.045,0.045,0.02286),
            filename=self._hyperparams['cad_path'])
=======
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
>>>>>>> c245b97f3cb77668c74789344f57be5d7274da8a

        # Calculate where the hand should be compared to the block
        new_pos = [posi.x, posi.y + 0.28018, posi.z + 0.055]

        # For as many conditions there are 
        for i in range(self._hyperparams['conditions']):
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))

    # Override so we make multiple plans and then choose one with best
    def compute_reference_trajectory(self, condition, policy):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        best_ref_ee = None # HAHAHAHAHA to store the best one
        best_ref_ja = None # HAHAHAHAHA to store the best one
        best_dist = float("inf") # Infinity itself

        while True:
            for attempt in range(self.attempts): # Let's do asked attempts on plan
                if self.use_saved_traj: # If there is a plan to read
                    best_plan = self.load_plan(self.plan_filename)
                    best_dist = self.get_dist(best_plan) # Best plan!
                    break # We can leave this place
                else: # Otherwise just create a motion plan
                    plan = self.plan_end_effector(target['position'], target['orientation'])
 

                if plan is not None:
                    cur_dist = self.get_dist(plan) # Get the distance of cur plan
                    #self.dists.append(cur_dist)
                    # If it beats it we need to use it! Woah!
                    if cur_dist < best_dist:
                        best_dist = cur_dist # This is the current best distance
                        best_plan = plan # This is the best plan
                        # Save the display trajectory and stuff
                        self.best_saved_traj[condition] = self.saved_traj 

            self.edit_plan_if_necessary(best_plan) # Edit the plan if we have a diff end
            # Only continue on with these calculations if necessary
            plan_joints = [np.array(point.positions) for point in best_plan.joint_trajectory.points]
            best_ref_ja = interpolate(plan_joints, self.T_interpolation)
            best_ref_ja.extend([best_ref_ja[-1]] * (self.T - self.T_interpolation))
            ref_poses = self.forward_kinematics(best_ref_ja, 'torso_lift_link')
            best_ref_ee = []
            ee_offsets = self._hyperparams['end_effector_points']

            for pose in ref_poses:
                position, orientation = np.array([pose[:3]]), pose[3:]
                rotation_mat = quaternion_matrix(orientation)[:3,:3]
                points = np.ndarray.flatten(get_ee_points(ee_offsets, position, rotation_mat).T)
                best_ref_ee.append(points)     

            # Publish it so it is the last thing you see before question
            self.publishDisplayTrajectory(self.best_saved_traj[condition]) 

            if not self.require_approval or yesno('Does this trajectory look ok?'):
                print("this is the distance of the best one: " + str(best_dist))
                break 

        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], best_ref_ja, best_ref_ee))

        with open('pickled_robot_traj_cond' + str(condition), 'w') as f:
            pickle.dump(self.best_saved_traj[condition], f)

        ref_offsets = np.array(best_ref_ee) - best_ref_ee[-1]

        traj_info = {
            'ja': np.array(best_ref_ja),
            'ee': np.array(best_ref_ee),
            'offsets': ref_offsets,
            'flattened': ref_offsets.flatten()
        }
        self.trajectories[condition] = traj_info


    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
        self.use_controller('MoveIt')
        target_position = [pos_x, pos_y, pos_z]
        target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

<<<<<<< HEAD
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
                dist = self.get_dist(plan) # Get the distance of the plan
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
                    dist = self.get_dist(plan) # Get the distance of the plan
                    print('The distance was ' + str(dist) + '\n')
                    return plan
                else:
                    print 'Failed.'.format(time)
                used_time += time
                time *= 2
        else:
                return None
        return None
=======
>>>>>>> c245b97f3cb77668c74789344f57be5d7274da8a
