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

class AgentCADExample(AgentCAD):
	# Initialize the AR function stuff and all that stuff
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
        # Number of AR tag they have on them
        self.ar = {'shaft2': 1, 'base_plate': 2} 

        # Create the functions with the proper offsets and whatever
        self.ar_functions[self.ar['shaft2']] = self.create_AR_function( \
        	self.ar['shaft2'], -0.027, -0.01, -0.0067, -3.14, 0, 3.14)

        self.ar_functions[self.ar['base_plate']] = self.create_AR_function( \
            self.ar['base_plate'], -0.005, -0.025, 0.05, 0.0067, 0, 0) 

        self.plans_made = False # Plans made or not
        self.stored_poses = {} # Stored poses for the objects in the scene
        self.change_conds_to_goals() # Because we want to reverse plan
        #self.wipe_plans() # Get rid of all the plans because we need to replan
        pdb.set_trace() # Need to stop and setup

    # Call the super reset
    def super_reset(self, condition):
    	super(AgentCADExample, self).reset(condition)

    # Time to set up the scene for the shaft2 experiment lmao
    def setup_shaft2(self):
    	self.get_all_poses() # Store the poses lol
        self.configure_scene_base()
        self.configure_scene_shaft2()
        self.grasp_prep_shaft2()

    def configure_scene_base(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        table_pose = self.get_AR_pose(3) # Get the AR position of the table
        if not table_pose: # If there isn't an AR marker
            z = 0.5 #Just some random height
        else:
            z = table_pose.position.z # Otherwise get the z coordinate
        print 'Adding objects to planning scene'
        self.add_object('table', position=[0.8,0.,z - 0.03], size=[0.7,1.5,0.03], type='box')

        for name in ['base_plate']:
            # Get the position of the objects using their AR tags
            pose = self.stored_poses[name]
            self.add_object(name, position=listify(pose.position), 
            	orientation=[0, 0, -0.3825, 0.9239557], size=(0.025, 0.025, 0.025), 
            	filename=self._hyperparams[name])

    def get_all_poses(self):
		for obj in self.ar: # For all the objects that we are keeping track of
			pose = self.get_pose(obj) # Get the pose from the thing
			self.stored_poses[obj] = pose # Store it in the dictionary
    
    # Configure the rviz scene so motion planning and stuff can work well
    def configure_scene_shaft2(self):
        # Get the position of the shaft using their AR tags
        pose = self.stored_poses['base_plate']
        newPose = copy.deepcopy(pose) # Make a deep copy of this
        # Make some modifications 
        newPose.position.x += 0.08020783
        newPose.position.y -= 0.08020783
        newPose.position.z += 0.1042719

        self.stored_poses['shaft2'] = newPose # Store this pose in relation
        #newPose = self.stored_poses['shaft2']
        self.add_object('shaft2', position=listify(newPose.position),
                orientation=[1, -0.027, 0, 0],
                #size=(0.025, 0.025, 0.025),
                size=(0.023, 0.023, 0.0245),
                filename=self._hyperparams['shaft2'])

    # Move the hand to a proper position to grasp the shaft2
    def grasp_prep_shaft2(self):
        self.use_controller('MoveIt')
        self.ungrip(15)
        self.ungrip(15) # Do it again just to make sure
        time.sleep(3)
        pose = self.stored_poses['shaft2']
        pos = pose.position # Get the position from the pose
        self.move_to(pos.x, pos.y, pos.z + 0.183, 0, 1.57, 0)
        time.sleep(2) # Wait for a little bit

    # Grasp whatever object you wanna grasp or something
    def grasp(self, object):
        self.grip(None)
        time.sleep(3)
        self.attach(object, touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    # Just for attaching so we can create a motion plan
    def attach_mesh(self, object):
        self.attach(object, touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    # Just a function to use MoveIt controllers to move to a certain position
    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
        self.use_controller('MoveIt')
        target_position = [pos_x, pos_y, pos_z]
        target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose, 1)
        self.group.execute(init_plan)

    # Change the goal point depending on the location of the base plate 
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
        if self.reset_time or self.plans_made is False:
            pass
        else:
            self.super_reset(condition)
            #time.sleep(2.0)

    # Override of this because we want to compute all the reference trajectories now
    # Otherwise there would be a lot of manual resetting 
    def determine_reference_trajectory(self, condition):
        for i in self.actual_conditions: # Let's do this for all the conditions now
            # This is due to the reversing we have to do
            self.determine_reference_trajectory_2(i)
            self.set_current_as_goal(i)
        self.plans_made = True # Set this to true


    def determine_reference_trajectory_2(self, condition):
        plan = self.get_existing_plan(condition)
        if plan is None:
            print 'No valid plan found for condition {}. Computing a fresh one'.format(condition)
            plan = self.reverse_plan(self.compute_plan(condition))
            self.offset_whole_plan(plan)
            # The reset plan is the reverse of the normal plan
            self.reset_plans[condition] = self.reverse_plan(plan) # Amazing really
            info = self.condition_info[condition]
            info.plan = plan
            info.save() 
        self.offset_whole_plan(plan)            
        info = self.condition_info[condition]
        info.plan = plan
        info.save() 
        self.reset_plans[condition] = self.reverse_plan(plan) # Amazing really
        self.trajectories[condition] = self.compute_reference_trajectory(plan)
        pdb.set_trace()
        # Copy these real quick 
        self.full_ref_ee[condition] = np.copy(self.trajectories[condition]['ee'])
        self.full_ref_ja[condition] = np.copy(self.trajectories[condition]['ja_pos'])
        self.full_ref_vel[condition] = np.copy(self.trajectories[condition]['ja_vel'])

