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

from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.agent.ros.ros_utils import ServiceEmulator, TimeoutException, msg_to_sample, policy_to_msg

from gps.algorithm.policy.lin_gauss_init import init_pd_ref
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM, JOINT_ANGLES, \
        JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, \
        ACTION, TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest
from gps.utility.general_utils import get_ee_points

from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS, REF_OFFSETS, REF_TRAJ


class AgentCADEx(AgentCAD):
    def __init__(self, hyperparams, init_node=True, trace=True):
        self.fixed_pose = Pose(Point(0.5, -0.1628, 0.5), Quaternion(0.5, -0.5, -0.5, 0.5))

        with open(osp.join(hyperparams['exp_dir'], 'model-static.sdf'), 'r') as f:
           self.fixed_piece_xml = f.read()

        AgentCAD.__init__(self, hyperparams, init_node)

        self.hack = -1.0
        self.try_mp = False
        pdb.set_trace()
        self.wipe_plans() # Make sure the condition info does not get saved
        self.setup()
        self.grasp()

    def all_resets(self, repetitions=1):
        for _ in range(repetitions):
            for condition in range(self._hyperparams['conditions']):
                self.reset(condition)

    def setup(self):
        self.configure_scene()
        self.grasp_prep()
        self.configure_scene()

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        self.reset_held_piece()

        print 'Adding objects to planning scene'
        self.add_object('table', position=[0.6,0.,0.42], size=[0.9,1.5,0.03], type='box')

        exp_dir = self._hyperparams['exp_dir']
        for name in ('held_piece', 'fixed_piece'):
            pose = self.get_pose(name)

            # Get the position of the objects
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=(0.05, 0.05, 0.0254),
                    filename=osp.join(exp_dir, 'piece.stl'))

    def reset_held_piece(self):
        print 'Resetting held piece'
        quat = Quaternion(*quaternion_from_euler(1.57971, 0.002477, 3.11933))
        pose = Pose(Point(0.841529, 0.209424, 0.501394), quat)
        self.set_pose('held_piece', pose)

    def reset_arm(self, ja, arm=TRIAL_ARM):
        self.use_controller('GPS')
        if self.close_to_ja(ja):
            print 'Ignoring reset (within tolerance)'
            return

        timeout = self._hyperparams['reset_timeout']
        if arm == TRIAL_ARM:
            if self.try_mp:
                try:
                    plan = self.plan_joints(ja)
                    if plan:
                        self.execute(plan)
                    else:
                        print 'Failed to generate reset plan'
                except Exception as e:
                    print 'Exception thrown:', e
                    pass # just let GPS reset try
                print 'Post-MoveIt', self.get_joint_angles()

            if self.hack is not None:
                hack_idx = 1   # yikes
                ja_hack = np.array(ja)
                ja_hack[hack_idx] = self.hack
                try:
                    self.issue_position_command(ja_hack, timeout, arm=arm)
                except TimeoutException:
                    pass

        self.issue_position_command(ja, timeout, arm=arm)

    def reset(self, condition):
        try:
           self.delete_model('fixed_piece')
        except: pass
        AgentCAD.reset(self, condition)
        self.spawn_model('fixed_piece', self.fixed_piece_xml, self.fixed_pose)

    def plan_for_condition(self, condition, dx=0.0, dy=0.0, dz=0.0):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        position = list(target['position'])
        position[0] += dx
        position[1] += dy
        position[2] += dz
        return self.plan_end_effector(position, target['orientation'], attempts=self.planning_attempts)

    def grasp_prep(self, dx=-0.2, dy=0.0, dz=0.0075):
        self.use_controller('MoveIt')
        self.ungrip(15)

        target_position = listify(self.get_pose('held_piece').position)
        target_position[0] += dx - 0.05
        target_position[1] += dy
        target_position[2] += dz
        self.move_to(target_position, [0,0,0])

        target_position[0] += 0.05
        self.move_to(target_position, [0,0,0])

    def grasp(self):
        self.grip(None)
        time.sleep(3)
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', \
            'l_gripper_r_finger_link', 'l_gripper_l_finger_link'])

    def random_initial(self):
        x, y = np.random.uniform(low=[0.2, -0.3], high=[0.6, 0.7])
        z = 0.525

        print x, y

        rot_x = np.random.choice([0.0, np.pi])
        rot_z = np.random.choice([0.0, np.pi/2, 3*np.pi/2])
        orientation = [rot_x, 0.0, rot_z]

        if self.move_to([x,y,z], orientation):
            time.sleep(2)
            ja = self.get_joint_angles()
            return list(ja)
        else:
            # try again 
            return self.random_initial()

    def random_initials(self, n):
        return [self.random_initial() for _ in range(n)]



