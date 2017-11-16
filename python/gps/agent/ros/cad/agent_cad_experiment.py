import copy
import os.path as osp
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
    def __init__(self, hyperparams, init_node=True, trace=True):
        self.fixed_pose = Pose(Point(0.5, -0.1628, 0.5), Quaternion(0.5, -0.5, -0.5, 0.5))

        with open('/home/gwthomas/.gazebo/models/fixed_piece/model-static.sdf', 'r') as f:
           self.fixed_piece_xml = f.read()

        AgentCAD.__init__(self, hyperparams, init_node)

        if self._hyperparams['use_AR_markers']:
            self.ar = {'held_piece': 0, 'fixed_piece': 1} # Number of AR tag they have on them

            # Create the functions with the proper offsets and whatever
            self.ar_functions[self.ar['held_piece']] = self.create_AR_function( \
                self.ar['held_piece'], 0.003, -0.02, -0.0265, 0, 0, -1.57)
                #self.ar['held_piece'], 0.035, 0, -0.0465, 0, 0, -1.57)
            self.ar_functions[self.ar['fixed_piece']] = self.create_AR_function( \
                self.ar['fixed_piece'], 0, -0.025, -0.0325, 0, 0, 0)

        if trace:
            pdb.set_trace()     # for optional setup, not debugging

    def all_resets(self, repetitions=1):
        for _ in range(repetitions):
            for condition in range(self._hyperparams['conditions']):
                self.reset(condition)

    def setup(self):
        self.configure_scene()
        self.grasp_prep()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        self.reset_held_piece()

        print 'Adding objects to planning scene'
        self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        # self.add_object('table', position=[0.8,0.,z], size=[0.7,1.5,0.03], type='box')

        exp_dir = self._hyperparams['exp_dir']
        for name in ('held_piece', 'fixed_piece'):
            pose = self.get_pose(name)

            # Get the position of the objects using their AR tags
            #pose, euler = self.pose_from_AR(name)
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.001, 0.001, 0.001),
                    filename=osp.join(exp_dir, '%s.stl' % name))

    def reset_held_piece(self):
        print 'Resetting held piece'
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def reset(self, condition):
        try:
           self.delete_model('fixed_piece')
        except:
           pass
        AgentCAD.reset(self, condition)
        self.spawn_model('fixed_piece', self.fixed_piece_xml, self.fixed_pose)

    def compute_plan(self, condition, dx=0.0, dy=0.0, dz=0.0):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        position = list(target['position'])
        position[0] += dx
        position[1] += dy
        position[2] += dz
        return self.plan_end_effector(position, target['orientation'], attempts=self.planning_attempts)

    def grasp_prep(self, dx=-0.2, dy=0.023, dz=0.00375):
        self.use_controller('MoveIt')
        self.ungrip(15)

        target_position = listify(self.get_pose('held_piece').position)
        target_position[0] += dx - 0.03
        target_position[1] += dy
        target_position[2] += dz
        self.move_to(target_position, [0,0,0])

        target_position[0] += 0.03
        self.move_to(target_position, [0,0,0])

    def grasp(self):
        # self.grip(None)
        # time.sleep(5)
        self.grip(3)
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    # Calculate the goal position depending on where the fixed piece is
    def change_goal(self):
        # Remove the thing from where it was in the rviz scene
        self.scene.remove_world_object('fixed_piece')

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

    def move_to(self, position, orientation):
        self.use_controller('MoveIt')
        init_plan = self.plan_end_effector(position, orientation, attempts=1)
        self.group.execute(init_plan)

    # Override so we make multiple plans and then choose one with best
    # def determine_reference_trajectory(self, condition):
    #     plan = self.get_existing_plan(condition)
    #     if plan is None:
    #         print 'No valid plan found for condition {}. Computing a fresh one'.format(condition)
    #         plan = self.reverse_plan(self.compute_plan(condition))
    #         self.edit_plan_if_necessary(plan) # Edit the plan if we have a diff end
    #
    #         filename = self._plan_file(condition)
    #         with open(filename, 'wb') as f: # Put the plan into a file
    #             pickle.dump(plan, f)
    #      # The reset plan is the reverse of the normal plan
    #     plan = self.reverse_plan(plan) # Amazing really
    #     self.trajectories[condition] = self.compute_reference_trajectory(plan)
    #     self.publishDisplayTrajectory(plan) # Publish the plan to the motion planner
    #     self.use_controller('GPS')
    #     # Calculate the trajectory information using this
    #     pdb.set_trace() # Ehh just stop here just in case real quick\
