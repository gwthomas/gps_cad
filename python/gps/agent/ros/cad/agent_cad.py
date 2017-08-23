import cPickle as pickle
import numpy as np
import pdb
import time
import pickle
import os.path as osp
import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import DisplayTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion, Twist
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from visualization_msgs.msg import Marker, MarkerArray
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from pr2_mechanism_msgs.srv import SwitchController
from tf.transformations import quaternion_matrix, quaternion_slerp
from genpy import Duration, Time
import tf
import copy # So we can copy things
from tf import TransformListener # So we can listen for transforms and that sort of thing

# For the messages from the AR sensing thing
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection

from gps.agent.ros.agent_ros import AgentROS
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd, init_pd_ref
from gps.algorithm.policy.lin_gauss_policy import LinearGaussianPolicy
from gps.agent.ros.ros_utils import ServiceEmulator, TimeoutException, msg_to_sample, policy_to_msg
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM, JOINT_ANGLES, \
        JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, \
        ACTION, TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, \
        REF_TRAJ, REF_OFFSETS, PROXY_CONTROLLER, NOISE
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest
from gps.utility.general_utils import get_ee_points
from gps.agent.ros.cad.util import *

from gps.proto.gps_pb2 import JOINT_ANGLES, END_EFFECTOR_POINTS, \
        END_EFFECTOR_POINT_JACOBIANS, REF_OFFSETS, REF_TRAJ

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

    _unpickleables = AgentROS._unpickleables + [
            'robot',
            'scene',
            'group',
            'fk',
            'ik',
            'get_model_state_srv',
            'set_model_state_srv',
            'spawn_model_srv',
            'delete_model_srv',
            'use_controller_srv',
            'gripper_client',
            'visual_pub',
            'trial_manager',
            'trajectories',
            'ar_marker_sub',
            'head_pub',
            'ik',
            'traj_read',
            'traj_display',
            'ar_functions',
            'tf'
    ]

    def __init__(self, hyperparams, init_node=True):
        AgentROS.__init__(self, hyperparams, init_node)
        self.actual_conditions = hyperparams['actual_conditions']
        self.condition_info = hyperparams['condition_info']

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')
        # This is for controlling the head or something like that
        self.head_pub = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)

        self.group.set_planner_id(hyperparams['planner'])
        self.planning_time = hyperparams['planning_time']
        self.planning_attempts = hyperparams['plan_attempts']
        self.T_interpolation = hyperparams['T_interpolation']

        self.tf = TransformListener() # This is to listen for transforms and that sort of thing
        self.fk = rospy.ServiceProxy('pr2_left_arm_kinematics/get_fk', GetPositionFK)
        self.ik = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik', GetPositionIK)
        self.get_model_state_srv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.set_model_state_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.spawn_model_srv = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        self.use_controller_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        self.gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        print 'Waiting for gripper server to start'; self.gripper_client.wait_for_server()
        self.visual_pub = rospy.Publisher('move_group/ompl_planner_data_marker_array', MarkerArray)
        # Gonna subscribe to the thing publishing locations for all the AR tags
        #self.ar_marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.getAR)
        self.ar_marker_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.getAR)

        self.use_AR_markers = hyperparams['use_AR_markers']
        self.cur_ar_markers = None # To store the AR information

        self.traj_read = rospy.Subscriber('move_group/display_planned_path', DisplayTrajectory, self.getTrajectory)
        self.traj_display = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory)
        self.saved_traj = None # Saved each time a thing is published

        self.ar = {} # Dictionary of AR objects
        self.ar_functions = {} # A dictionary of AR functions
        self.ee_goal = None # Set the EE goal (if this has been set or not)
        self.ja_goal = None # Set the joint angle goal (if this has been set or not)\
        self.reset_plans = [0] * 5 # Empty array for the reset plans

        self.dumb_ilqr = False # If we are just going to use the end positions for the cost fns

        self.trial_manager = ProxyTrialManager(self)

        if 'ee_link' in hyperparams:
            ee_link = hyperparams['ee_link']
            assert ee_link in self.robot.get_link_names()
            print 'Using link {} as end effector'.format(ee_link)
            self.group.set_end_effector_link(ee_link)
        self.ee_link = self.group.get_end_effector_link()

        self.trajectories = {}
        self.reset_trajectories = {} # For dat fancy reset or something
        self.current_controller = None
        self.initialized = set()

    # Move the head to look at the designated point (so camera point in right dir)
    def move_head(self, x, y, z):
        goal = PointHeadGoal() # Make a new goal
        point = PointStamped() # make the goal point
        point.header.frame_id = 'base_link'
        point.point.x = x
        point.point.y = y
        point.point.z = z
        goal.target = point
        # Just set all of these up hahahaha
        goal.pointing_frame = "high_def_frame"
        goal.pointing_axis.x = 1
        goal.pointing_axis.y = 0
        goal.pointing_axis.z = 0
        goal.min_duration = Duration(0.5)
        goal.max_velocity = 1
        self.head_pub.send_goal(goal) # Publish the head goal

    # Stores the gotten AR tag information in an instance variable
    def getAR(self, msg):
        self.cur_ar_markers = msg # Store what we have gotten lmao

    def get_AR_pose(self, number):
        # Get the markers from the detections array
        markers = self.get_AR_markers()
        for tag in markers: # Go through all of them and look at them or something
            if tag.id == number: # Check if it's the right thing
                gottenPose = self.tf.transformPose('base_footprint', tag.pose)
                return gottenPose.pose # Return the pose of the thing
        return None # Otherwise, return none

    # Stores the gotten visual trajectory information in an instance variable
    def getTrajectory(self, msg):
        self.saved_traj = msg # Stores the trajectory that was received

    # Just a nice function to get the current AR markers
    def get_AR_markers(self):
        # Returns the array of AR markers
        #return self.cur_ar_markers.markers
        return self.cur_ar_markers.detections

    # Get the chosen tag of the AR markers
    # If the AR tag doesn't exist in the array, return None
    #def get_AR_pose(self, number):
    #    the_markers = self.get_AR_markers()
    #    for tag in the_markers: # Look through all the tags in the array
    #        if tag.id == number: # If we found the ID we are looking for
    #            return tag.pose.pose # Return the pose of the tag
    #    return None # If we couldn't find anything, return None

    # This will create an AR function that will return the pose of the
    # actual item depending on where the AR tag is placed (that's why offsets)
    def create_AR_function(self, id_number, x_offset, y_offset, z_offset, \
        euler_offset_0, euler_offset_1, euler_offset_2):
        def get_item_pose():
            ar_pose = copy.deepcopy(self.get_AR_pose(id_number)) # Get the pose of AR marker
            if ar_pose is None: # If you can't find the AR marker
                print("Item " + str(id_number) + " AR tag not found!")
                return None, None
            # Consider the offsets
            ar_pose.position.x += x_offset
            ar_pose.position.y += y_offset
            ar_pose.position.z += z_offset
            # Ehhh get the transformation or something
            euler = list(tf.transformations.euler_from_quaternion(listify( \
                ar_pose.orientation)))

            #print("Position: " + str(ar_pose.position)) # This is kind of for debugging
            #print("Euler: " + str(euler))
            # No idea why we have to do this but sometimes the orientation tracking is wonky
            #if euler[0] < 0:
            #    euler[0] += 1.57
            #if euler[2] > 0:
            #    euler[2] -= 1.57
            # Do the euler offsets as well
            euler[0] += euler_offset_0
            euler[1] += euler_offset_1
            euler[2] += euler_offset_2
            ar_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*euler))
            return ar_pose, euler # Return both the ar_pose and the euler angle
        return get_item_pose # Return the function

    # Just a nicer function to get the pose of the thing
    def pose_from_AR(self, obj_name):
        # If this is not an object in the object dictionary
        if obj_name not in self.ar:
            print("No such object: " + str(obj_name))
            return None, None
        # Just run the ar_functions thing
        return self.ar_functions[self.ar[obj_name]]()

    # Just calculates the distance between the tags and whatever
    def dist_AR_tags(self, tag1ID, tag2ID):
        tag1Pose = self.get_AR_pose(tag1ID) # Get the poses of the tags
        tag2Pose = self.get_AR_pose(tag2ID) # Get the pose of the other tag
        tag1pos, tag1ori = tag1Pose.position, tag1Pose.orientation
        tag2pos, tag2ori = tag2Pose.position, tag2Pose.orientation
        diff_pos = np.array(listify(tag1pos)) - np.array(listify(tag2pos))
        diff_ori = np.array(listify(tag1ori)) - np.array(listify(tag2ori))
        diff_euler = tf.transformations.euler_from_quaternion(diff_ori)
        # Calculate how they are different and all that
        return diff_pos, np.array(diff_euler)

    # Gets the average distance because fluctuations and all that
    def avg_dist_AR_tags(self, tag1ID, tag2ID, attempts=10):
        diffs_posi = np.zeros((attempts, 3))
        diffs_euler = np.zeros((attempts, 3))
        for i in range(attempts): # For as many attempts or whatever
            time.sleep(0.5) # Sleep for 0.5 seconds and stuff
            dif_pos, dif_eul = self.dist_AR_tags(tag1ID, tag2ID)
            diffs_posi[i, :] = np.copy(dif_pos) # Put it in the slot
            diffs_euler[i, :] = np.copy(dif_eul)
        print(diffs_posi)
        print(diffs_euler)
        print("The average distance was: ")
        print(str(np.mean(diffs_posi, axis=0)))
        print("The average euler difference was: ")
        print(str(np.mean(diffs_euler, axis=0)))
        print("Range of values for distance difference: ")
        print(str(np.ptp(diffs_posi, axis=0)))
        print("Range of values for euler difference: ")
        print(str(np.ptp(diffs_euler, axis=0)))

    # Resets the object depending on the AR tag and stuff
    def reset_object_AR(self, name, theSize):
        # Remove the object from rviz first
        self.scene.remove_world_object(name)

        # Get the pose twice just in case or whatever
        pose, euler = self.pose_from_AR(name)
        pose, euler = self.pose_from_AR(name)
        posi = pose.position # Get the position from the pose

        # Add the fixed piece back to the rviz scene
        self.add_object(name, position=listify(pose.position),
            orientation=listify(pose.orientation),
            size=theSize,
            filename=self._hyperparams[name])

        return pose, euler # Return the found pose and euler for convenience

    # Publishes the given visual plan to rViz
    def publishDisplayTrajectory(self, plan):
        traj = DisplayTrajectory()
        traj.model_id = 'pr2' # It's the pr2!
        traj.trajectory = [plan] # Put the plan in the array maybe?
        # Get the current state of the robot or something
        traj.trajectory_start = self.robot.get_current_state()
        self.traj_display.publish(traj) # Publish the DisplayTrajectory

    def use_controller(self, target):
        assert target in ('GPS', 'MoveIt')
        switch = True
        if target == 'GPS' and self.current_controller != 'GPS':
            start = ['GPSPR2Plugin']
            stop = ['l_arm_controller', 'r_arm_controller']
            self.current_controller = 'GPS'
        elif target == 'MoveIt' and self.current_controller != 'MoveIt':
            start = ['l_arm_controller', 'r_arm_controller']
            stop = ['GPSPR2Plugin']
            self.current_controller = 'MoveIt'
        else:
            switch = False

        if switch:
            print 'Switching to {} controllers'.format(target)
            self.use_controller_srv(start, stop, 2) # 2 means STRICT
            time.sleep(1)

    # Query Gazebo for the pose of a named object
    def get_gazebo_pose(self, name, relative_entity_name=''):
        response = self.get_model_state_srv(name, relative_entity_name)
        if not response.success:
            raise RuntimeError('Failed to get pose for model {}'.format(name))
        return response.pose

    def get_pose(self, id):
        if self.use_AR_markers:
            return self.get_AR_pose(id)
        else:
            return self.get_gazebo_pose(id)

    def set_pose(self, name, pose, relative_entity_name=''):
        model_state = ModelState(name, pose, Twist(), relative_entity_name)
        response = self.set_model_state_srv(model_state)
        if not response.success:
            raise RuntimeError('Failed to set pose for model {}'.format(name))

    def spawn_model(self, name, xml, pose, namespace='', reference_frame=''):
        request = SpawnModelRequest(name, xml, namespace, pose, reference_frame)
        response = self.spawn_model_srv(request)
        if not response.success:
            raise RuntimeError('Failed to spawn model {}'.format(name))

    def delete_model(self, name):
        request = DeleteModelRequest(name)
        response = self.delete_model_srv(request)
        if not response.success:
            raise RuntimeError('Failed to delete model {}'.format(name))

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
        time.sleep(2) # Just rest for a little bit before removing
        self.scene.remove_world_object(name)

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
        except Exception as e:
            print(e)
            print 'Auxiliary arm reset timed out'

    # Reset the arm and wait the alloted time or something
    def reset_arm_and_wait(self, arm, mode, data, timeout):
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
        reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(reset_command, timeout=timeout)

    def set_gripper(self, position, max_effort, wait):
        self.gripper_client.cancel_all_goals()
        rospy.sleep(1)

        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        if wait is None:
            self.gripper_client.send_goal(goal)
        else:
            duration = rospy.Duration(wait)
            result = self.gripper_client.send_goal_and_wait(goal, execute_timeout=duration)
            import pdb; pdb.set_trace()

    def grip(self, wait):
        self.set_gripper(0.0, 50.0, wait)

    def ungrip(self, wait):
        self.set_gripper(0.08, 50.0, wait)

    def compute_plan_cost(self, plan):
    	# prevPoint = None # Start off with nothing
    	# dist = 0 # Start off the distance as 0
    	# for thePoint in plan.joint_trajectory.points:
    	# 	position = np.array(thePoint.positions)
    	# 	if prevPoint is not None: # If there was a previous pt
    	# 		dist += np.sqrt(np.sum(np.square(position - prevPoint)))
    	# 	prevPoint = position # Aww man almost forgot this
    	# return dist
        ref_traj = self.compute_reference_trajectory(plan)
        ee_pos = ref_traj['ee']
        return sum([np.sum((ee_pos[i] - ee_pos[i-1])**2) for i in range(1, len(ee_pos))])

    def plan(self, attempts=1):
        self.group.set_planning_time(self.planning_time)
        best_plan, best_cost = None, float('inf')  # Infinity itself

        # For as many attempts as allowed
        for attempt in range(attempts):
            plan = self.group.plan()
            if plan is not None and len(plan.joint_trajectory.points) > 0:
                cur_cost = self.compute_plan_cost(plan)
                print cur_cost
                if cur_cost < best_cost:
                    best_cost = cur_cost    # update current best distance
                    best_plan = plan        # update current best plan
                    print 'New best cost:', best_cost
        return best_plan

    def plan_end_effector(self, target_position, target_orientation=None, attempts=None):
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
        return self.plan(attempts) # Send in the attempts lmao


    def plan_joints(self, target_joint_positions, attempts=1):
        self.group.set_joint_value_target(target_joint_positions)
        return self.plan(attempts)

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

    # Get the inverse kinematics for this
    def inverse_kinematics1(self, pose_stamped):
        request = PositionIKRequest() # Create a new request
        request.ik_link_name = self.ee_link # We want the end effector
        request.pose_stamped = pose_stamped # Desired pose and what not
        request.timeout = Duration(5.0) # Here have another duration or something
        # This is to seed the IK service or something like that???
        rs = moveit_msgs.msg.RobotState() # Getting the robot state
        rs.joint_state.name = JOINT_NAMES # This is the current joint value thing
        # I dunno get the current joint values
        rs.joint_state.position = self.group.get_current_joint_values()
        # Not really sure so am just gonna put this down lmao
        #request.ik_seed_state = rs
        request.robot_state = rs

        # let's call it now and get a response
        response = self.ik(request)
        return response.solution # Return the solution found (hopefully something)

    # Get the joint angles of a pose that is some m[meters]_above the current pose (of the end
    # effector) that we have lmao
    def get_ja_above(self, cur_pose, m_above):
        cur_pose.pose.position.z += m_above # Increase the z value by some amount
        solution = self.inverse_kinematics1(cur_pose)
        # Return the array of values lmao
        return solution.joint_state.position

    def get_end_effector_pose(self):
        state = self.robot.get_current_state().joint_state
        joints = []
        for joint in JOINT_NAMES:
            index = state.name.index(joint)
            joints.append(state.position[index])
        return self.forward_kinematics1(joints)

    def get_initial(self, condition, arm=TRIAL_ARM):
        condition_data = self._hyperparams['reset_conditions'][condition]
        return condition_data[arm]['data']

    def reset_arm(self, arm, mode, data):
        reset_command = PositionCommand()
        reset_command.mode = mode
        reset_command.data = data
        reset_command.pd_gains = self._hyperparams['pid_params']
        reset_command.arm = arm
        timeout = self._hyperparams['reset_timeout']
        reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(reset_command, timeout=timeout)

    def reset(self, condition):
        self.use_controller('GPS')
        print 'Resetting to condition {} (actually {})'.format(condition, self.actual_conditions[condition])

        try:
            self.reset_arm(TRIAL_ARM, JOINT_SPACE, self.get_initial(condition, arm=TRIAL_ARM))
        except TimeoutException:
            print 'Trial arm reset timed out'

        try:
            self.reset_arm(AUXILIARY_ARM, JOINT_SPACE, self.get_initial(condition, arm=AUXILIARY_ARM))
        except TimeoutException:
            print 'Auxiliary arm reset timed out'

    def compute_plan(self, condition):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        # while True:
        #     plan = self.plan_end_effector(target['position'], target['orientation'])
        #     self.edit_plan_if_necessary(plan) # Just edit it if you need to change the goal lmao
        #
        #     if not self.require_approval or yesno('Does this trajectory look ok?'):
        #         return plan
        return self.plan_end_effector(target['position'], target['orientation'], attempts=self.planning_attempts)

    def _plan_file(self, condition):
        return osp.join(self._hyperparams['exp_dir'], 'plans', 'cond{}.pkl'.format(condition))

    # Do the initialization of the reset trajectory and policy and stuff??
    def init_reset_traj(self, condition, policy):
        plan = self.reset_plans[condition] # Get the plan for the reset trajectory
        # Compute the reference trajectory given the plan
        self.reset_trajectories[condition] = self.compute_reference_trajectory(plan)
        # Now it's time to initialize this policy or something
        ref_traj_info = self.reset_trajectories[condition]
        ref_ja_pos = ref_traj_info['ja_pos']
        ref_ja_vel = ref_traj_info['ja_vel']
        ref_ee = ref_traj_info['ee']
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja_pos, ref_ja_vel, ref_ee))

    # This is if you run it in the real world! Set the current position as the goal!
    def set_current_as_goal(self):
        pose = self.get_ee_pose() # Get the current pose of the group
        pose = self.get_ee_pose() # Get the current pose of the group

        # For as many conditions there are
        for i in range(self._hyperparams['conditions']):
            # Set the goal to the current position
            self._hyperparams['targets'][i]['position'] = listify(pose.position)
            self._hyperparams['targets'][i]['orientaton'] = list(tf.transformations.euler_from_quaternion( \
                listify(pose.orientation)))

    # Lmao for when using the real robot and you have set the pose
    def get_ee_pose(self):
        return self.group.get_current_pose().pose

    # This is if you want to set the ultimate destination from the
    def set_real_goal(self, condition):
        self.ee_goal = self.get_ee_pose() # Use what is happening in real world (??)
        self.ja_goal = self.group.get_current_joint_values() # Get the current joint values\
        # Get the difference in position from real to the specified position

    # Offset the plan depending on the differences
    def offset_whole_plan(self, thePlan):
        if self.ee_goal is None:
            return # If there is no special end goal just end it lol
        goalJoint = np.array(self.ja_goal)
        endGoal = thePlan.joint_trajectory.points[-1].positions # This is the current goal
        diff = np.array(goalJoint) - np.array(endGoal) # This is the difference
        print("This is the difference: " + str(diff)) # Print this
        #diffPos = np.array(listify(self.ee_goal.position)) - \
        #    np.array(self._hyperparams['targets'][condition]['position'])
        # Get the difference in orientation from real to specified position
        #diffOri = np.array(listify(self.ee_goal.orientation)) - \
        #    np.array(self._hyperparams['targets'][condition]['orientation'])
        # For all the points in there
        for i in range(len(thePlan.joint_trajectory.points)):
            # Add the difference to the plan and hope for the best or something
            thePlan.joint_trajectory.points[i].positions = \
            np.array(thePlan.joint_trajectory.points[i].positions) + diff

    # Edit the plan according to the different joint angle plan lmao
    def edit_plan_if_necessary(self, thePlan):
        if self.ja_goal is None: # If there is no change to the end goal
            return thePlan # Just return the normal plan
        # Otherwise, we must change the very last point to be the new goal or something?
        thePlan.joint_trajectory.points[-1].positions = self.ja_goal

    # Shift the plan depending on the changes to the end

    # This is if you start at the ending location and then want to move away
    def reverse_plan(self, thePlan):
        newPlan = copy.deepcopy(thePlan)
        newPlan.joint_trajectory.points.reverse() # Reverse the points
        # Lmao just use the list reversing thingq
        numPoints = len(thePlan.joint_trajectory.points) # How many points
        for i in range(numPoints): # Just flip everything around or something
            newPlan.joint_trajectory.points[i].time_from_start = \
                thePlan.joint_trajectory.points[i].time_from_start
        return newPlan

    def get_existing_plan(self, condition):
        return self.condition_info[condition].plan

    def compute_reference_trajectory(self, plan):
        plan_ja_pos = [np.array(point.positions) for point in plan.joint_trajectory.points]
        plan_ja_vel = [np.array(point.velocities) for point in plan.joint_trajectory.points]
        ref_ja_pos = interpolate(plan_ja_pos, self.T_interpolation)
        ref_ja_vel = interpolate(plan_ja_vel, self.T_interpolation)
        ref_ja_pos.extend([ref_ja_pos[-1]] * (self.T - self.T_interpolation))
        ref_ja_vel.extend([ref_ja_vel[-1]] * (self.T - self.T_interpolation))
        ref_poses = self.forward_kinematics(ref_ja_pos, 'torso_lift_link')
        ref_ee = []
        ee_offsets = self._hyperparams['end_effector_points']
        for pose in ref_poses:
            position, orientation = np.array([pose[:3]]), pose[3:]
            rotation_mat = quaternion_matrix(orientation)[:3,:3]
            points = np.ndarray.flatten(get_ee_points(ee_offsets, position, rotation_mat).T)
            ref_ee.append(points)

        ref_ja_pos = np.array(ref_ja_pos)
        ref_ja_vel = np.array(ref_ja_vel)
        ref_ee = np.array(ref_ee)
        ref_offsets = ref_ee - ref_ee[-1]
        return {
            'ja_pos': ref_ja_pos,
            'ja_vel': ref_ja_vel,
            'ee': ref_ee,
            'offsets': ref_offsets,
            'flattened': ref_offsets.flatten()
        }

    def determine_reference_trajectory(self, condition):
        plan = self.get_existing_plan(condition)
        if plan is None:
            print 'No valid plan found for condition {}. Computing a fresh one'.format(condition)
            plan = self.compute_plan(condition)
            info = self.condition_info[condition]
            info.plan = plan
            info.save()
        self.trajectories[condition] = self.compute_reference_trajectory(plan)

    def initialize_controller(self, condition, policy):
        if condition in self.initialized or not isinstance(policy, LinearGaussianPolicy):
            return

        ref_traj_info = self.trajectories[condition]
        ref_ja_pos = ref_traj_info['ja_pos']
        ref_ja_vel = ref_traj_info['ja_vel']
        ref_ee = ref_traj_info['ee']
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja_pos, ref_ja_vel, ref_ee))
        self.initialized.add(condition)

    # Does something different if reset is false
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
        # There are different trajectories based on if reset or not
        if self.reset_time:
            trajectories = self.reset_trajectories
        else:
            trajectories = self.trajectories

        if condition not in trajectories: # If this hasn't been initialized yet
            if self.reset_time:
                self.init_reset_traj(condition, policy)
            else:
                self.determine_reference_trajectory(condition)

        if self.reset_time:
            print('Reset sampling, condition ' + str(condition))
        else:
            print 'Sampling, condition', condition

        self.initialize_controller(condition, policy)
        self.reset(condition)

        # Get the information from the trajectory information
        ref_traj_info = trajectories[condition]
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

        if not isinstance(policy, TfPolicy):
            sample_msg = self._trial_service.publish_and_wait(
                trial_command, timeout=self._hyperparams['trial_timeout']
            )
        else:
            self.trial_manager.prep(policy, condition)
            self._trial_service.publish(trial_command)
            self.trial_manager.run(self._hyperparams['trial_timeout'])
            while self._trial_service._waiting:
                print 'Waiting for sample to come in'
                rospy.sleep(1.0)
            sample_msg = self._trial_service._subscriber_msg

        sample = msg_to_sample(sample_msg, self)
        sample.set(NOISE, noise)

        if self.dumb_ilqr: # If we are going to use the dumb ilqr cost fns
            print("Doing the dumb ilqr thing")
            sample.set(REF_OFFSETS, np.zeros((self.T, 9)))
            sample.set(REF_TRAJ, np.zeros(self.T * 9))
        else:
            sample.set(REF_OFFSETS, ref_traj_info['offsets'])
            sample.set(REF_TRAJ, np.array([ref_traj_info['flattened']]*self.T))

        if save and not self.reset_time: # If we are not gonna save this sample as reset
            self._samples[condition].append(sample)
        if save and self.reset_time: # If we are going to save the reset sample
            self._reset_samples[condition].append(sample)
        self.reset_time = False # It's not reset time after this lmaoo???
        return sample

    def get_action(self, policy, obs):
        # extra = ['distances', 'coeffs', 'ee_pos', 'attended', 'direction']
        extra = ['centered_traj', 'coeffs', 'ee_pos', 'attended']
        # extra = []
        action, debug = policy.act(None, obs, None, None, extra=extra)
        return action

    def get_obs(self, request, condition):
        array = np.array(request.obs)
        # print array[14:23]
        ref_flattened = self.trajectories[condition]['flattened']
        obs = np.concatenate([array, ref_flattened, [self.trial_manager.t]])
        return obs

    # Gets the difference between the goal and the estimated goal
    def diff_real_estimate(self):
        if self.ee_goal is None: # If you haven't set up the real thing yet
            print("The real goal hasn't been set yet!")
            return
        est_pos = self._hyperparams['targets'][0]['position']
        est_ori = self._hyperparams['targets'][0]['orientation']
        real_pos = listify(self.ee_goal.position)
        real_ori = tf.transformations.euler_from_quaternion(listify(self.ee_goal.orientation))
        print("The position diff (real - est): " + str(np.arrray(real_pos) - np.array(est_pos)))
        print("The euler diff (real - est): " + str(np.arrray(real_ori) - np.array(est_ori)))
