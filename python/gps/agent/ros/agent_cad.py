import numpy as np
import pdb
import time

import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker, MarkerArray
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_mechanism_msgs.srv import SwitchController
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_slerp
import tf

from gps.agent.ros.agent_ros import AgentROS
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.agent.ros.ros_utils import ServiceEmulator, msg_to_sample, \
        policy_to_msg
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM, JOINT_ANGLES, \
        JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, \
        ACTION, TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest
from gps.utility.general_utils import get_ee_points

try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None

JOINT_NAMES = [
        'l_shoulder_pan_joint',
        'l_shoulder_lift_joint',
        'l_upper_arm_roll_joint',
        'l_elbow_flex_joint',
        'l_forearm_roll_joint',
        'l_wrist_flex_joint',
        'l_wrist_roll_joint'
]


def listify(o):
    if isinstance(o, Point):
        return [o.x, o.y, o.z]
    if isinstance(o, Quaternion):
        return [o.x, o.y, o.z, o.w]
    else:
        return list(o)


def interpolate(points, T):
    n = len(points)
    if n > T:
        raise RuntimeError('Cannot interpolate {} points over {} timesteps'.format(n,T))

    interpolated_points = []
    segment_lengths = []
    num_segments = n - 1
    base_pps = T // num_segments    # points per segment
    leftover = T - base_pps * num_segments
    for segment_idx in range(num_segments):
        prev, next = points[segment_idx], points[segment_idx+1]
        segment = []
        if segment_idx < num_segments - 1:
            if leftover:
                extra = np.random.binomial(leftover, 1.0/num_segments)
                leftover -= extra
            else:
                extra = 0
            points_in_this_segment = base_pps + extra
            close = False
        else:
            points_in_this_segment = T - sum(segment_lengths)   # los demas
            close = True

        segment_lengths.append(points_in_this_segment)

        for i in range(points_in_this_segment):
            if close:
                progress = float(i) / (points_in_this_segment - 1)
            else:
                progress = float(i) / points_in_this_segment
            interpolated_point = prev + progress * (next - prev)
            interpolated_points.append(interpolated_point)
            segment.append(interpolated_point)

    print 'Interpolation segment lengths', segment_lengths
    return interpolated_points


class AgentCAD(AgentROS):
    def __init__(self, hyperparams, init_node=True):
        AgentROS.__init__(self, hyperparams, init_node)

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')

        if 'planner' in hyperparams:
            print 'Using planner', hyperparams['planner']
            self.group.set_planner_id(hyperparams['planner'])

        if 'planning_schedule' in hyperparams:
            self.planning_schedule = hyperparams['planning_schedule']
        else:
            self.planning_schedule = [3, 10]
        print 'Planning schedule:', self.planning_schedule

        self.T_interpolation = hyperparams['T_interpolation']

        self.fk = rospy.ServiceProxy('pr2_left_arm_kinematics/get_fk', GetPositionFK)
        self.get_model_state_srv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.set_model_state_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.use_controller_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        self.gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        print 'Waiting for gripper server to start'; self.gripper_client.wait_for_server()
        self.visual_pub = rospy.Publisher('move_group/ompl_planner_data_marker_array', MarkerArray)

        if 'ee_link' in hyperparams:
            ee_link = hyperparams['ee_link']
            assert ee_link in self.robot.get_link_names()
            print 'Using link {} as end effector'.format(ee_link)
            self.group.set_end_effector_link(ee_link)
        self.ee_link = self.group.get_end_effector_link()

        self.initial_joint_positions = [4.371272987757635e-05, 0.0038376809966607084, -0.001063654465488284, -0.3349022940558859, -0.0014703147353571921, -0.7034821724667868, 0.00012755038417200382]
        self.trajectories = {}
        self.current_controller = None

    def use_controller(self, target):
        assert target in ('GPS', 'MoveIt')
        switch = False
        if target == 'GPS' and self.current_controller != 'GPS':
            start = ['GPSPR2Plugin']
            stop = ['l_arm_controller', 'r_arm_controller']
            self.current_controller = 'GPS'
            switch = True
        elif target == 'MoveIt' and self.current_controller != 'MoveIt':
            start = ['l_arm_controller', 'r_arm_controller']
            stop = ['GPSPR2Plugin']
            self.current_controller = 'MoveIt'
            switch = True

        if switch:
            print 'Switching to {} controllers'.format(target)
            self.use_controller_srv(start, stop, 2) # 2 means STRICT
            time.sleep(1)

    def clear_visuals(self):
        header = Header(0, rospy.Time(), self.group.get_planning_frame())
        markers = []
        for id in range(10):
            delmarker = Marker()
            delmarker.header = header
            delmarker.action = Marker.DELETE
            delmarker.ns = 'points'
            delmarker.id = id
            markers.append(delmarker)
        marker_array = MarkerArray(markers=markers)
        self.visual_pub.publish(marker_array)

    def visualize_points(self, points, id, size=0.01, color=(1.,1.,1.,1.), frame=None):
        frame = self.group.get_planning_frame() if frame is None else frame
        header = Header(0, rospy.Time(), frame)
        marker = Marker()
        marker.header = header
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.ns = 'points'
        marker.id = id
        marker.points = [Point(*point) for point in points]
        marker.colors = [ColorRGBA(*color)] * len(points)
        marker.scale.x = size
        marker.scale.y = size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker_array = MarkerArray(markers=[marker])
        self.visual_pub.publish(marker_array)

    # Query Gazebo for the pose of a named object
    def get_pose(self, name, relative_entity_name=''):
        response = self.get_model_state_srv(name, relative_entity_name)
        if not response.success:
            raise RuntimeError('Failed to get pose for object {}'.format(name))
        return response.pose

    def set_pose(self, name, pose, relative_entity_name=''):
        model_state = ModelState(name, pose, Twist(), relative_entity_name)
        response = self.set_model_state_srv(model_state)
        if not response.success:
            raise RuntimeError('Failed to set pose for object {}'.format(name))

    def add_object(self, name, position, orientation=(0,0,0,0), size=(1,1,1), type=None, filename=None):
        assert type or filename
        pose = Pose(Point(*position), Quaternion(*orientation))
        header = Header(0, rospy.Time.now(), self.group.get_planning_frame())
        pose_stamped = PoseStamped(header, pose)
        if filename:
            self.scene.add_mesh(name, pose_stamped, filename, size)
        elif type == 'box':
            self.scene.add_box(name, pose_stamped, size)
        elif type == 'sphere':
            radius = max(size)
            self.scene.add_sphere(name, pose_stamped, radius)

    def attach(self, name, touch_links=[]):
        self.scene.attach_mesh(self.ee_link, name, touch_links=touch_links)
        pdb.set_trace()
        self.scene.remove_world_object(name)
        pdb.set_trace()


    def reset_arm(self, arm, mode, data):
        """
        Issues a position command to an arm.
        Args:
            arm: Either TRIAL_ARM or AUXILIARY_ARM.
            mode: An integer code (defined in gps_pb2).
            data: An array of floats.
        """
        reset_command = PositionCommand()
        reset_command.mode = mode
        reset_command.data = data
        reset_command.pd_gains = self._hyperparams['pid_params']
        reset_command.arm = arm
        timeout = self._hyperparams['reset_timeout']
        reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(reset_command, timeout=timeout)
        #TODO: Maybe verify that you reset to the correct position.

    def reset(self, condition):
        self.use_controller('GPS')
        condition_data = self._hyperparams['reset_conditions'][condition]
        try:
            self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
                    condition_data[TRIAL_ARM]['data'])
        except:
            print 'Failed to reset trial arm'
        try:
            self.reset_arm(AUXILIARY_ARM, condition_data[AUXILIARY_ARM]['mode'],
                       condition_data[AUXILIARY_ARM]['data'])
        except:
            print 'Failed to reset auxiliary arm'

    def set_gripper(self, position, max_effort, wait):
        self.gripper_client.cancel_all_goals()

        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        if wait is None:
            self.gripper_client.send_goal(goal)
        else:
            duration = rospy.Duration(wait)
            self.gripper_client.send_goal_and_wait(goal)

    def grip(self, wait):
        self.set_gripper(0.0, 50.0, wait)

    def ungrip(self, wait):
        self.set_gripper(0.08, 50.0, wait)

    def plan(self):
        for time in self.planning_schedule:
            print 'Planning with {} seconds'.format(time)
            self.group.set_planning_time(time)
            plan = self.group.plan()
            if len(plan.joint_trajectory.points) > 0:
                print 'Success!'
                return plan
            else:
                print 'Failed.'.format(time)
        raise RuntimeError('Unable to find a valid plan. Consider modifying the schedule')

    def plan_end_effector(self, target_position, target_orientation=None):
        current_position = listify(self.group.get_current_pose().pose.position)
        target_position = listify(target_position)
        if target_orientation is None:
            print 'Planning path from {} to {}'.format(current_position, target_position)
            self.group.set_position_target(target_position)
        else:
            assert len(target_orientation) in (3,4)
            target_orientation = listify(target_orientation)
            print 'Planning path from {} to {} with orientation {}'.format(
                    current_position, target_position, target_orientation)
            self.group.set_pose_target(target_position + target_orientation)
        return self.plan()

    def plan_joints(self, target_joint_positions):
        self.group.set_joint_value_target(target_joint_positions)
        return self.plan()

    def smart_reset(self, condition):
        self.group.execute(self.plan_joints(self.initial_joint_positions))

    def forward_kinematics1(self, joint_positions, frame):
        header = Header(0, rospy.Time.now(), frame)
        rs = moveit_msgs.msg.RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = joint_positions
        response = self.fk(header, [self.ee_link], rs)
        pose = response.pose_stamped[0].pose
        return np.array(listify(pose.position) + listify(pose.orientation))

    def forward_kinematics(self, plan, frame):
        return [self.forward_kinematics1(point.positions, frame) for point in plan.joint_trajectory.points]

    def get_end_effector_pose(self):
        state = self.robot.get_current_state().joint_state
        joints = []
        for joint in JOINT_NAMES:
            index = state.name.index(joint)
            joints.append(state.position[index])
        return self.forward_kinematics1(joints)

    def get_reference_trajectory(self, condition):
        if condition not in self.trajectories:
            target = self._hyperparams['targets'][condition]
            plan = self.plan_end_effector(target['position'], target['orientation'])

            fk_poses = self.forward_kinematics(plan, 'torso_lift_link')
            ref_poses = interpolate(fk_poses, self.T_interpolation)
            ref_poses.extend([fk_poses[-1]] * (self.T - self.T_interpolation))
            ref_traj = []
            ee_offsets = self._hyperparams['end_effector_points']
            for pose in ref_poses:
                position, orientation = np.array([pose[:3]]), pose[3:]
                rotation_mat = quaternion_matrix(orientation)[:3,:3]
                points = np.ndarray.flatten(get_ee_points(ee_offsets, position, rotation_mat).T)
                ref_traj.append(points)
            self.trajectories[condition] = (plan, ref_traj)
        else:
            plan, ref_traj = self.trajectories[condition]

        # all_points = []
        # for points in ref_traj:
        #     all_points.extend([points[0:3], points[3:6], points[6:9]])
        plot_poses = self.forward_kinematics(plan, 'world')
        plot_positions = [points[:3] for points in plot_poses]
        self.visualize_points(plot_positions, 0, size=0.01, color=(0.,1.,1.,1.), frame='odom_combined')

        return ref_traj

    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
        """
        Reset and execute a policy and collect a sample.
        Args:
            policy: A Policy object.
            condition: Which condition setup to run.
            verbose: Unused for this agent.
            save: Whether or not to store the trial into the samples.
        Returns:
            sample: A Sample object.
        """
        print 'Sampling, condition', condition

        #added from agent_ros.py of public gps codebase
        if TfPolicy is not None:  # user has tf installed.
            if isinstance(policy, TfPolicy):
                self._init_tf(policy.dU)

        self.reset(condition)

        ref_traj = self.get_reference_trajectory(condition)

        # Generate noise.
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))

        # Execute trial.
        trial_command = TrialCommand()
        trial_command.id = self._get_next_seq_id()
        trial_command.controller = policy_to_msg(policy, noise)
        trial_command.T = self.T
        trial_command.id = self._get_next_seq_id()
        trial_command.frequency = self._hyperparams['frequency']
        ee_points = self._hyperparams['end_effector_points']
        trial_command.ee_points = ee_points.reshape(ee_points.size).tolist()
        trial_command.ee_points_tgt = ref_traj[-1]
        trial_command.state_datatypes = self._hyperparams['state_include']
        trial_command.obs_datatypes = self._hyperparams['state_include']


        if self.use_tf is False:
            sample_msg = self._trial_service.publish_and_wait(
                trial_command, timeout=self._hyperparams['trial_timeout']
            )
        else:
            self._trial_service.publish(trial_command)
            sample_msg = self.run_trial_tf(policy, time_to_run=self._hyperparams['trial_timeout'])

        sample = msg_to_sample(sample_msg, self)
        sample.set('target_traj_ee_points', [points - ref_traj[-1] for points in ref_traj])

        if save:
            self._samples[condition].append(sample)
        return sample


class AgentCADExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)

        self.use_controller('GPS')
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
        self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')

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
        pdb.set_trace()

        self.ungrip(15)

        target_position[0] += 0.1
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)
        self.use_controller('MoveIt')

    def grasp(self):
        self.grip(None)
        time.sleep(7)
        self.attach('held_piece', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])
