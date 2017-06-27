import numpy as np
import pdb
import time
import pickle
import os.path as osp
import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist
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
from gps.agent.ros.cad.util import *


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

class AgentCAD(AgentROS):
    def __init__(self, hyperparams, init_node=True):
        AgentROS.__init__(self, hyperparams, init_node)

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')

        self.group.set_planner_id(hyperparams['planner'])
        self.planning_schedule = hyperparams['planning_schedule']
        self.indefatigable = hyperparams['indefatigable']
        self.require_approval = hyperparams['require_approval']
        self.T_interpolation = hyperparams['T_interpolation']

        self.fk = rospy.ServiceProxy('pr2_left_arm_kinematics/get_fk', GetPositionFK)
        self.get_model_state_srv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.set_model_state_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.use_controller_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        self.gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        print 'Waiting for gripper server to start'; self.gripper_client.wait_for_server()
        self.visual_pub = rospy.Publisher('move_group/ompl_planner_data_marker_array', MarkerArray)

        self.traj_read = rospy.Subscriber('move_group/display_planned_path', DisplayTrajectory, self.getTrajectory)
        self.traj_display = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory)
        self.best_saved_traj = None # Set this to nothing for now
        self.saved_traj = None # Saved each time a thing is published


        if 'ee_link' in hyperparams:
            ee_link = hyperparams['ee_link']
            assert ee_link in self.robot.get_link_names()
            print 'Using link {} as end effector'.format(ee_link)
            self.group.set_end_effector_link(ee_link)
        self.ee_link = self.group.get_end_effector_link()

        self.trajectories = {}
        self.current_controller = None

    def getTrajectory(self, msg):
        print("Received a trajectory, yayy!!!!\n")
        self.saved_traj = msg # Stores the trajectory that was received

    def publishDisplayTrajectory(self, traj):
        self.traj_display.publish(traj) # Publish the DisplayTrajectory

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
        marker = Marker()
        marker.header.frame_id = frame
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
        self.scene.remove_world_object(name)

    # def reset_arm(self, arm, mode, data):
    #     """
    #     Issues a position command to an arm.
    #     Args:
    #         arm: Either TRIAL_ARM or AUXILIARY_ARM.
    #         mode: An integer code (defined in gps_pb2).
    #         data: An array of floats.
    #     """
    #     reset_command = PositionCommand()
    #     reset_command.mode = mode
    #     reset_command.data = data
    #     reset_command.pd_gains = self._hyperparams['pid_params']
    #     reset_command.arm = arm
    #     timeout = self._hyperparams['reset_timeout']
    #     reset_command.id = self._get_next_seq_id()
    #     self._reset_service.publish_and_wait(reset_command, timeout=timeout)
    #     #TODO: Maybe verify that you reset to the correct position.

    def reset(self, condition):
        self.use_controller('GPS')
        condition_data = self._hyperparams['reset_conditions'][condition]
        try:
            self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
                    condition_data[TRIAL_ARM]['data'])
        except Exception as e:
	    print(e)
            print 'Trial arm reset timed out'
        try:
            self.reset_arm(AUXILIARY_ARM, condition_data[AUXILIARY_ARM]['mode'],
                       condition_data[AUXILIARY_ARM]['data'])
        except TimeoutException:
            print 'Auxiliary arm reset timed out'

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

    # This calculates the distance of the joints in a trajectory
    def get_dist(self, plan):
	prevPoint = None # Start off with nothing
	dist = 0 # Start off the distance as 0
	for thePoint in plan.joint_trajectory.points:
		position = np.array(thePoint.positions)
		if prevPoint is not None: # If there was a previous pt
			dist += np.sqrt(np.sum(np.square(position - prevPoint)))
		prevPoint = position # Aww man almost forgot this
	return dist

    # This will help save a plan and what not
    def save_plan(self, plan, toSave=None):
        if toSave is None:
            toSave = 'savedTraj.txt'
        with open(toSave, 'w') as f:
            pickle.dump(plan, f)

    # Load a plan from a file lmaoooo
    def load_plan(self, filename):
        with open(filename, 'r') as f:
            return pickle.load(f)

    def plan(self):
        for time in self.planning_schedule:
            print 'Planning with {} seconds'.format(time)
            self.group.set_planning_time(time)
            plan = self.group.plan()
            if len(plan.joint_trajectory.points) > 0:
                print 'Success!'
                self.save_traj(plan)
                return plan
            else:
                print 'Failed.'.format(time)
        if self.indefatigable:
            print 'Failed to find a valid plan under the given schedule, but trying again'
            time = 1
            while True:
                print 'Planning with {} seconds'.format(time)
                self.group.set_planning_time(time)
                plan = self.group.plan()
                if len(plan.joint_trajectory.points) > 0:
                    print 'Success!'
                    self.save_traj(plan)
                    return plan
                else:
                    print 'Failed.'.format(time)
                time *= 2
        else:
            raise RuntimeError('Unable to find a valid plan. Consider modifying the schedule or using the indefatigable option')

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

    def forward_kinematics1(self, joint_angles, frame):
        header = Header(0, rospy.Time.now(), frame)
        rs = moveit_msgs.msg.RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = joint_angles
        response = self.fk(header, [self.ee_link], rs)
        pose = response.pose_stamped[0].pose
        return np.array(listify(pose.position) + listify(pose.orientation))

    def forward_kinematics(self, joint_angles, frame):
        return [self.forward_kinematics1(angles, frame) for angles in joint_angles]

    def get_end_effector_pose(self):
        state = self.robot.get_current_state().joint_state
        joints = []
        for joint in JOINT_NAMES:
            index = state.name.index(joint)
            joints.append(state.position[index])
        return self.forward_kinematics1(joints)

    def compute_reference_trajectory(self, condition, policy):
    #def compute_reference_trajectory(self, condition):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]

        while True:
            plan = self.plan_end_effector(target['position'], target['orientation'])
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

            plot_trajectories([ref_ee])

            if not self.require_approval or yesno('Does this trajectory look ok?'):
                break

        # trajectories = []
        # for i in range(3):
        #     plan = self.plan_end_effector(target['position'], target['orientation'])
        #     fk_poses = self.forward_kinematics(plan, 'torso_lift_link')
        #     ref_poses = interpolate(fk_poses, self.T_interpolation)
        #     ref_poses.extend([fk_poses[-1]] * (self.T - self.T_interpolation))
        #     trajectories.append(ref_poses)
        # plot_trajectories(trajectories)
        # pdb.set_trace()

        self.trajectories[condition] = ref_ee
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja, ref_ee))

        ref_offsets = np.array([points - ref_ee[-1] for points in ref_ee])
        ref_ja = np.array(ref_ja)
        ref_ee = np.array(ref_ee)
        ref_offsets = ref_ee - ref_ee[-1]
        return {
            'ja': ref_ja,
            'ee': ref_ee,
            'offsets': ref_offsets,
            'flattened': ref_offsets.flatten()
        }

    def determine_reference_trajectory(self, condition, policy):
        filename = 'ref_traj_{}.npz'.format(condition)
        if osp.exists(filename):
            print 'Using existing reference trajectory for condition', condition
            ref_traj_info = np.load(filename)
        else:
            print 'No reference trajectory found for condition {}. Computing a fresh one'.format(condition)
            ref_traj_info = self.compute_reference_trajectory(condition)
            np.savez(filename, **ref_traj_info)
        ref_ja, ref_ee = ref_traj_info['ja'], ref_traj_info['ee']
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja, ref_ee))
        self.trajectories[condition] = ref_traj_info

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
        if condition not in self.trajectories:
            self.compute_reference_trajectory(condition, policy)

        print 'Sampling, condition', condition
        self.reset(condition)

        #added from agent_ros.py of public gps codebase
        self.determine_reference_trajectory(condition, policy)

        self.reset(condition)
        if TfPolicy is not None:  # user has tf installed.
            if isinstance(policy, TfPolicy):
                self._init_tf(policy.dU)

        ref_traj_info = self.trajectories[condition]
        ref_ee = ref_traj_info['ee']

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

        trial_command.frequency = self._hyperparams['frequency']
        ee_points = self._hyperparams['end_effector_points']
        trial_command.ee_points = ee_points.reshape(ee_points.size).tolist()
        trial_command.ee_points_tgt = ref_ee[-1]
        trial_command.state_datatypes = self._hyperparams['state_include']
        trial_command.obs_datatypes = self._hyperparams['state_include'] # changing this to 'obs_include' resulted in weird Gazebo memory corruption

        if self.use_tf is False or not isinstance(policy, TfPolicy):
            sample_msg = self._trial_service.publish_and_wait(
                trial_command, timeout=self._hyperparams['trial_timeout']
            )
        else:
            self._trial_service.publish(trial_command)
            sample_msg = self.run_trial_tf(policy, condition, time_to_run=self._hyperparams['trial_timeout'])

        sample = msg_to_sample(sample_msg, self)
        sample.set(REF_OFFSETS, ref_traj_info['offsets'])
        sample.set(REF_TRAJ, np.array([ref_traj_info['flattened']]*self.T))

        if save:
            self._samples[condition].append(sample)
        return sample

    def _get_new_action(self, policy, obs):
        # extra = ['ref_traj', 'distances', 'weights', 'attended', 'ee_pos']
        extra = []
        action, debug = policy.act(None, obs, None, None, extra=extra)
        return action

    def _get_obs(self, msg, condition):
        array = AgentROS._get_obs(self, msg, condition)
        if self._hyperparams['attention']:
            ref_flattened = self.trajectories[condition]['flattened']
            obs = np.concatenate([array, ref_flattened])
        else:
            obs = array
        return obs
