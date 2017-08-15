import copy
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

class RealAgentCADExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True, trace=True):
        self.fixed_pose = Pose(Point(0.5, -0.1628, 0.5), Quaternion(0.5, -0.5, -0.5, 0.5))
        #with open('/home/gwthomas/.gazebo/models/piece/model-static.sdf', 'r') as f:
        #    self.piece_xml = f.read()
        self.diffPos, self.diffOri = None, None # Just set these

        AgentCAD.__init__(self, hyperparams, init_node)
        self.ar = {'held_piece': 6, 'fixed_piece': 5} # Number of AR tag they have on them

        # Create the functions with the proper offsets and whatever
        self.ar_functions[self.ar['held_piece']] = self.create_AR_function( \
            self.ar['held_piece'], 0.003, -0.02, -0.0265, 0, 0, -1.57)
            #self.ar['held_piece'], 0.035, 0, -0.0465, 0, 0, -1.57)
        self.ar_functions[self.ar['fixed_piece']] = self.create_AR_function( \
            self.ar['fixed_piece'], -0.015, -0.025, -0.0325 - 0.015, 1.57, 0, 0)

        if trace:
            pdb.set_trace()     # for optional setup, not debugging

    def all_resets(self, repetitions=1):
        conditions = self._hyperparams['conditions']
        for _ in range(repetitions):
            for i in range(conditions):
                self.reset(i)

    def setup(self):
        self.configure_scene()
        self.grasp_prep()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        #print 'Resetting piece'
        #self.reset_piece()
        table_pose = self.get_AR_pose(7) # Get the AR position of the table
        if not table_pose: # If there isn't an AR marker
            z = 0.72 # Ehh just some random height
        else:
            z = table_pose.position.z # Otherwise get the z coordinate

        #self.reset_held_piece()

        print 'Adding objects to planning scene'
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.8,0.,z], size=[0.7,1.5,0.03], type='box')

        for name in ('fixed_piece', 'held_piece'):
            #pose = self.get_pose(name)
            # Get the position of the objects using their AR tags
            pose, euler = self.pose_from_AR(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.045,0.045,0.02286),
                    filename=self._hyperparams['cad_path'])
        self.change_goal() # Change the goal depending on where the goal piece is lmao

    def reset_held_piece(self):
        print 'Resetting held piece'
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def reset(self, condition):
        #try:
        #    self.delete_model('fixed_piece')
        #except:
        #    pass
        AgentCAD.reset(self, condition)
        #self.spawn_model('fixed_piece', self.piece_xml, self.fxied_pose)

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
        init_plan = self.plan_end_effector(target_position, target_pose, 1)
        self.group.execute(init_plan)

        self.ungrip(15)

    def grasp(self):
        self.grip(None)
        time.sleep(5)
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

        # Calculate where the hand should be compared to the block
        new_pos = [posi.x, posi.y + 0.28018, posi.z + 0.055]

        # For as many conditions there are
        for i in range(self._hyperparams['conditions']):
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))

    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
        self.use_controller('MoveIt')
        target_position = [pos_x, pos_y, pos_z]
        target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose, 1)
        self.group.execute(init_plan)

    # Override of this because we want to use a different plan [offsetted] (???)
    def determine_reference_trajectory(self, condition):
        plan = self.get_existing_plan(condition)
        if plan is None:
            print 'No valid plan found for condition {}. Computing a fresh one'.format(condition)
            plan = self.compute_plan(condition)
            self.offset_whole_plan(plan) # LMAO offset the plan as desired
            filename = self._plan_file(condition)
            with open(filename, 'wb') as f:
                pickle.dump(plan, f)
        self.trajectories[condition] = self.compute_reference_trajectory(plan)
