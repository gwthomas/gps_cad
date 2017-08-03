import pdb
import time
import get_plan
import pickle
import copy

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

class RealGearExperiment(AgentCAD):
	# Initialize the AR function stuff and all that stuff
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
        self.attempts = 20 # Make 20 plans 
        self.use_saved_traj = None # Any saved trajectory??
        # Number of AR tag they have on them
        self.ar = {'shaft2': 6, 'base_plate': 4, 'compound_gear': 8} 

        # Create the functions with the proper offsets and whatever
        self.ar_functions[self.ar['shaft2']] = self.create_AR_function( \
        	self.ar['shaft2'], -0.02, 0, 0.015, -3.14, 0, 3.14)

        self.ar_functions[self.ar['base_plate']] = self.create_AR_function( \
            #self.ar['base_plate'], 0.02, -0.005, 0, -1.45, 0, 0.785) #0.785
            self.ar['base_plate'], 0.0126, 0, -0.01, 0, 0, -0.785) # z 0.0155

        self.ar_functions[self.ar['compound_gear']] = self.create_AR_function( \
            self.ar['compound_gear'], -0.055, 0, -0.0281, 0, 0, 0) #0.03

        pdb.set_trace() # Need to stop and setup

    # Time to set up the scene for the shaft2 experiment lmao
    def setup_shaft2(self):
        self.configure_scene_base()
        self.configure_scene_shaft2()
        self.grasp_prep_shaft2()

    # Time to set up the scene for the compound gear experiment lmao
    def setup_compound_gear(self):
        self.configure_scene_base()
        self.configure_scene_compound_gear()
        self.grasp_prep_compound_gear()

    def configure_scene_base(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        table_pose = self.get_AR_pose(7) # Get the AR position of the table
        if not table_pose: # If there isn't an AR marker
            z = 0.72 # Ehh just some random height
        else:
            z = table_pose.position.z # Otherwise get the z coordinate
        print 'Adding objects to planning scene'
        self.add_object('table', position=[0.8,0.,z], size=[0.7,1.5,0.03], type='box')

        for name in ['base_plate']:
            # Get the position of the objects using their AR tags
            pose, euler = self.pose_from_AR(name)
            self.add_object(name, position=listify(pose.position), 
            	orientation=[0, 0, -0.36, 1], size=(0.025, 0.025, 0.025), 
            	filename=self._hyperparams[name])
    
    # Configure the rviz scene so motion planning and stuff can work well
    def configure_scene_shaft2(self):
        # Get the position of the shaft using their AR tags
        pose, euler = self.pose_from_AR('shaft2')
        self.add_object('shaft2', position=listify(pose.position),
                orientation=[1, -0.027, 0, 0],
                size=(0.025, 0.025, 0.025),
                filename=self._hyperparams['shaft2'])

    # Configure the rviz scene so motion planning and stuff can work well
    def configure_scene_compound_gear(self):
        # Get the position of the shaft using their AR tags
        pose, euler = self.pose_from_AR('compound_gear')
        self.add_object('compound_gear', position=listify(pose.position),
                orientation=[0, 0, 0, 1],
                size=(0.025, 0.025, 0.025),
                filename=self._hyperparams['compound_gear'])

    # Move the hand to a proper position to grasp the shaft2
    def grasp_prep_compound_gear(self):
        self.use_controller('MoveIt')
        self.ungrip(15)
        self.ungrip(15) # UGHHHH DO IT AGAIN ITS SO DUMB
        time.sleep(3)
        pose, euler = self.pose_from_AR('compound_gear')
        pos = pose.position # Get the position from the pose
        # 0.238
        self.move_to(pos.x - 0.24, pos.y, pos.z + 0.008, 1.57, 0, 0)
        time.sleep(2) # Wait for a little bit

    # Move the hand to a proper position to grasp the shaft2
    def grasp_prep_shaft2(self):
        self.use_controller('MoveIt')
        self.ungrip(15)
        self.ungrip(15) # UGHHHH DO IT AGAIN ITS SO DUMB
        time.sleep(3)
        pose, euler = self.pose_from_AR('shaft2')
        pos = pose.position # Get the position from the pose
        self.move_to(pos.x - 0.205, pos.y, pos.z - 0.02, 0, 0, 0)
        time.sleep(2) # Wait for a little bit

    # Grasp whatever object you wanna grasp or something
    def grasp(self, object):
        self.grip(None)
        time.sleep(3)
        self.attach(object, touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    # Just a function to use MoveIt controllers to move to a certin position
    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
        self.use_controller('MoveIt')
        target_position = [pos_x, pos_y, pos_z]
        target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)

    # Change the goal point depending on the location of the base plate lmao
    def change_goal(self):
        # Reset the object in the rviz scene
        pose, euler = self.reset_object_AR('base_plate', (0.025, 0.025, 0.025))
        posi = pose.position # Get the position from the pose

        # Calculate where the hand should be compared to the base plate
        new_pos = [posi.x - 0.122374, posi.y - 0.076841, posi.z + 0.088421]

        # For as many conditions there are 
        for i in range(self._hyperparams['conditions']):
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))

    # Overrides the reset method to do nothing because we want to do learning
    def reset(self, condition):
    	self.use_controller('GPS') # Make sure you are using GPS controller tho
        pass

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

    # Override so we make multiple plans and then choose one with best
    def compute_reference_trajectory(self, condition, policy):
    	#super(RealGearExperiment, self).reset(condition) # Call the super method
        #self.reset(condition)
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
            self.reset_plans[condition] = copy.deepcopy(best_plan) # Amazing really
            self.reverse_plan(best_plan) # We have to reverse the plan in this case

            # Only continue on with these calculations if necessary
            best_ref_ee, best_ref_ja = self.calc_ee_and_ja(best_plan)

            if self.best_saved_traj[condition] is not None:
            	# Publish it so it is the last thing you see before question
            	self.publishDisplayTrajectory(self.best_saved_traj[condition]) 

            if not self.require_approval or yesno('Does this trajectory look ok?'):
                print("this is the distance of the best one: " + str(best_dist))
                break 

        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], best_ref_ja, best_ref_ee))

        with open('pickled_robot_traj_cond' + str(condition), 'w') as f:
            pickle.dump(self.best_saved_traj[condition], f)

        # Calculate the trajectory information using this
        traj_info = self.calc_trajectory_info(best_ref_ee, best_ref_ja)
        pdb.set_trace() # Ehh just stop here just in case real quick
        self.trajectories[condition] = traj_info
        self.reset(condition)

    # Hmmm some resetting things??? 
    '''
    def reset(self, condition):
        self.use_controller('GPS')
        condition_data = self._hyperparams['reset_conditions'][condition]
        
        if self.do_reset: # Only reset if we're supposed to reset lmao
            try:
            	try:
            		# Get the joint angle value that is like 5 cm above the current pose?
            		goal_ja = self.get_ja_above(self.group.get_current_pose(), 0.05)
            		self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
            			goal_ja)
            	except Exception as e:
            		print(e)
                self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
                    condition_data[TRIAL_ARM]['data'])
            except Exception as e:
                print(e)
                print 'Trial arm reset timed out'
        else:
            self.do_reset = True # Set it to true now
        
        #self.group.execute(self.reset_plans[condition])
        try:
            self.reset_arm(AUXILIARY_ARM, condition_data[AUXILIARY_ARM]['mode'],
                       condition_data[AUXILIARY_ARM]['data'])
        except Exception as e:
            print(e)
            print 'Auxiliary arm reset timed out'
            '''