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
        REF_TRAJ, REF_OFFSETS, PROXY_CONTROLLER, NOISE, TIMESTEP
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest
from gps.utility.general_utils import get_ee_points
from gps.agent.ros.cad.util import *


try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None

f = open('actions.txt', 'w') # Get the file

JOINT_NAMES = [
        'l_shoulder_pan_joint',
        'l_shoulder_lift_joint',
        'l_upper_arm_roll_joint',
        'l_elbow_flex_joint',
        'l_forearm_roll_joint',
        'l_wrist_flex_joint',
        'l_wrist_roll_joint'
]

def normalize_ja(ja, limits):
    ja = np.array(ja)
    for idx, vmin, vmax in limits:
        while ja[idx] < vmin:
            ja[idx] += 2*np.pi
        while ja[idx] > vmax:
            ja[idx] -= 2*np.pi
    return ja

def normalize_ja_moveit(ja):
    return normalize_ja(ja, [
        (4, -np.pi, np.pi),
        (6, -np.pi, np.pi)
    ])

def normalize_ja_gps(ja):
    return normalize_ja(ja, [
        (4, 0, 2*np.pi),
        (6, -2*np.pi, 0)
    ])

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
        self.conditions = hyperparams['conditions']
        self.actual_conditions = hyperparams['actual_conditions']
        self.condition_info = hyperparams['condition_info']

        #for info in self.condition_info:
        #    info.plan = None    # disregard saved plans (for debugging)
        #    pass

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')
        # This is an ActionClient controlling the head of robot
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
        self.ar_marker_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.getAR)

        self.use_AR_markers = hyperparams['use_AR_markers']
        self.cur_ar_markers = None # To store the AR information

        self.traj_read = rospy.Subscriber('move_group/display_planned_path', DisplayTrajectory, self.getTrajectory)
        self.traj_display = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory)
        self.saved_traj = None # Saved each time a thing is published

        self.ar = {} # Dictionary of AR objects
        self.ar_functions = {} # A dictionary of AR functions
        self.ee_goal = None # Set the EE goal (if this has been set or not
        self.ja_goal = None # Set the joint angle goal (if this has been set or not)\
        self.reset_plans = [0] * self.conditions # Empty array for the reset plans
        self.num_samples = 5

        self.dumb_ilqr = False # If we are just going to use the end positions for the cost fns

        self.saved_samples = [[] * self.conditions] # Just for storing this real quick

        self.cur_T = [self.T] * self.conditions # For storing the T of each of the conditions!
        self.final_T = self.T # This is the original T 

        # Disregard these following variables - work in progress
        self.samples_taken = [0] * self.conditions
        self.full_ref_ee = [0] * self.conditions # Lol for the reference ee
        self.full_ref_ja = [0] * self.conditions # For the full reference ja
        self.full_ref_vel = [0] * self.conditions # For the full reference vel
        self.varying_T = False # If you want T to vary depending on a whole bunch of stuff
        self.the_tolerance = 0.015 # If the difference is that large it's concerning
        self.padding = 10 # How many timesteps to put as padding for each of the segments
        self.iter_per_seg = 1 # Let's train this many iterations per segment
        self.iter_count = 0 # Count iterations

        self.otherPol = None
        self.chosen_parts = None # Can use particular timesteps (use array)
        self.cur_part = -1 # Current index

        self.trial_manager = ProxyTrialManager(self)

        if 'ee_link' in hyperparams:
            ee_link = hyperparams['ee_link']
            assert ee_link in self.robot.get_link_names()
            print 'Using link {} as end effector'.format(ee_link)
            self.group.set_end_effector_link(ee_link)
        self.ee_link = self.group.get_end_effector_link()

        self.trajectories = {}
        self.reset_trajectories = {} # For the special reset
        self.current_controller = None
        self.initialized = set()

        self.plotter_xml = None
        self.plotted_names = []

        self.traj_plot_period = None

    # For the conditions specified (startInd - endInd, inclusive), use the reset
    def set_reset_pos(self, startInd, endInd):
        # For all the conditions in this nice range
        for i in range(startInd, endInd + 1):
            self._hyperparams['reset_conditions'][i] = \
                self.condition_info[i].initial

    # Wipe the plans of all the conditions 
    def wipe_plans(self):
        for i in range(self.conditions):
            self.condition_info[i].plan = None

    # Wipe the plan of the condition specified
    def wipe_plan(self, condition):
        self.condition_info[condition].plan = None

    # Calculate the T depending on how much of the trajectory we are using
    def change_T(self, condition):
        print("Change T is being called")
        # Length of the trajectory
        traj_length = len(self.trajectories[condition]['ee'])
        cur_T = self.cur_T[condition] # What we have right now!!

        # If we are using varying T, and it's time to check if we need to change
        if self.varying_T:
            the_cutoff = self.calculate_cutoff(condition) # Calculate the cutoff!
            if the_cutoff is None: # If we don't need a cutoff or something
                 # Just double the length of traj or something
                self.T = min((cur_T - self.padding) * 2 + self.padding, self.final_T)
            else: # Otherwise let's use the cutoff suggested
                self.T = the_cutoff
        elif self.chosen_parts is not None:
            self.cur_part += 1 # Increment the current part or something
            self.cur_part = min(self.cur_part, len(self.chosen_parts) - 1)
            self.T = self.chosen_parts[self.cur_part] # Change the current one
            self.T += self.padding # If not the full trajectory, add the padding
            self.T = min(self.T, self.final_T) # Choose the min of this

        print("cur_T: " + str(cur_T) + " and new self.T " + str(self.T))
        if cur_T != self.T: # If the current T is different from the new T
            return True # Things have changed
        else: # Otherwise, things haven't changed
            return False

    # This is for updating the T and then the policy after we've passed a certain number
    # of iterations!
    def update_T_then_policy(self, policy, condition):
        print(str(self.iter_per_seg * 5) + ' samples have passed lmao')
        changed = self.change_T(condition) # Call this now that everything has changed

        if not changed: # If T has not changed, just return
            return

        if self.T >= self.T_interpolation: # If we've reached T_interpolation already
            self.T = self.final_T # Just use the whole trajectory lmao
            ref_ja, ref_ee = self.full_ref_ja[condition], self.full_ref_ee[condition]
            ref_vel = self.full_ref_vel[condition]
        else:
            ref_ja = list(self.full_ref_ja[condition][:(self.T - self.padding)])
            ref_ee = list(self.full_ref_ee[condition][:(self.T - self.padding)])
            ref_vel = list(self.full_ref_vel[condition][:(self.T - self.padding)])
            # This is adding more timesteps to the last step in the segment
            ref_ja.extend([self.full_ref_ja[condition][self.T-1]] * self.padding)
            ref_ee.extend([self.full_ref_ee[condition][self.T-1]] * self.padding)
            ref_vel.extend([self.full_ref_vel[condition][self.T-1]] * self.padding)

        self.cur_T[condition] = self.T # Update!

        # We're gonna change policies - initialize with the old stuff learned tho
        self.change_policy(condition, policy, ref_ja, ref_vel, ref_ee)

    # This is initializing a new policy with more timesteps using the
    # information from the old policy! :D
    def change_policy(self, condition, policy, new_ref_ja, new_ref_vel, new_ref_ee):

        print("Changing the policy")
        # Save all of these because we're about to initialize with something newww
        old_K, old_k = copy.deepcopy(policy.K), copy.deepcopy(policy.k)
        # This was how many timesteps there were before, not including the padding!
        old_T = old_K.shape[0] - self.padding
        pdb.set_trace()
        with open('old_policy.txt', 'w') as f:
            noise = np.zeros((old_K.shape[0], self.dU))
            f.write(str(policy_to_msg(policy, noise)))

        old_pol_covar, old_chol_pol_covar = copy.deepcopy(policy.pol_covar), copy.deepcopy(policy.chol_pol_covar)
        old_inv_pol_covar = copy.deepcopy(policy.inv_pol_covar)
        old_T = min(old_T, self.T) # Make sure to choose the smaller one I think (???)
        # Now we change the initial values to match what we learned before
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], new_ref_ja, new_ref_vel, new_ref_ee))
        policy.K[:old_T, :, :], policy.k[:old_T, :] = old_K[:old_T, :, :], old_k[:old_T, :]
        policy.pol_covar[:old_T, :, :], policy.chol_pol_covar[:old_T, :, :] = old_pol_covar[:old_T, :, :], old_chol_pol_covar[:old_T, :, :]
        policy.inv_pol_covar[:old_T, :, :] = old_inv_pol_covar[:old_T, :, :]

        # Writing the new policy to a text file just so we can examine
        with open('new_policy.txt', 'w') as f:
            noise = np.zeros((self.T, self.dU))
            f.write(str(policy_to_msg(policy, noise)))
        pdb.set_trace()

    def calculate_cutoff(self, condition):
        use_prev = 5 # Randomly use the last 5 samples to figure this out
        check_every = 10 # Check every 10 samples for the proper cutoff
        # If there are no saved samples for this condition
        if len(self.saved_samples[condition]) == 0:
            return None # Cannot calculate the cutoff, return none

        T = self.saved_samples[condition][-1].T # Use the T of the samples
        chosen_samples = self.saved_samples[condition][-1 * use_prev:] # Get the last couple of samples
        avg = np.zeros(self.saved_samples[condition][-1].get(REF_OFFSETS).shape)[:T, :] # Get the shape

        for sample in chosen_samples: # For all the samples we are using
            tgt = sample.get(REF_OFFSETS)[:T, :]
            pt = sample.get(END_EFFECTOR_POINTS)[:T, :]
            dist = pt - tgt # Figure out the distance
            avg += dist # Add the found distance to the average

        avg /= float(use_prev) # Get the average and stuff
        new_thing = np.zeros((avg.shape[0], 3))
        # Gonna actually calculate the distances because these are three vectors
        new_thing[:, 0] = np.sqrt(np.sum(np.square(avg[:, :3]), axis=1))
        new_thing[:, 1] = np.sqrt(np.sum(np.square(avg[:, 3:6]), axis=1))
        new_thing[:, 2] = np.sqrt(np.sum(np.square(avg[:, 6:9]), axis=1))
        summed_dist = np.sum(new_thing, axis=1) # Sum the distances from end effector pts
        summed_dist /= 3 # This should be a T x 1 Thing

        best_cutoff, largest_diff = 0, 0 # Just initialize these
        the_ratio = 0 # The ratio
        # Start off with each little chunk
        for i in range(7 * check_every, T, check_every):
            #earlier_part = np.mean(avg[:i, :])
            #later_part = np.mean(avg[i:, :])
            earlier_part = np.mean(summed_dist[:i])
            later_part = np.mean(summed_dist[i:])
            the_diff = later_part - earlier_part
            if the_diff > largest_diff: # If the difference is larger
                best_cutoff, largest_diff = i, the_diff
                the_ratio = later_part / earlier_part

        print("The best cutoff found was at timestep: " + str(best_cutoff))
        print("Largest diff: " + str(largest_diff) + " Ratio: " + str(the_ratio))
        #pdb.set_trace()

        self.saved_samples[condition] = [] # Clear this as well

        if largest_diff >= self.the_tolerance:
            return best_cutoff
        else:
            return None

    def all_resets(self, repetitions=1):
        conditions = self._hyperparams['conditions']
        for _ in range(repetitions):
            for i in range(conditions):
                self.reset(i)

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
        self.cur_ar_markers = msg # Store what we have gotten 

    # Get the pose of the AR tag (it's the center) given the number
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

            # Do the euler offsets as well
            euler[0] += euler_offset_0
            euler[1] += euler_offset_1
            euler[2] += euler_offset_2
            ar_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*euler))
            return ar_pose, euler # Return both the ar_pose and the euler angle
        return get_item_pose # Return the function

    # Get the pose of an object (using the AR tag and AR tag dictionary)
    def pose_from_AR(self, obj_name):
        # If this is not an object in the object dictionary
        if obj_name not in self.ar:
            print("No such object: " + str(obj_name))
            return None, None
        # Just run the ar_functions thing
        return self.ar_functions[self.ar[obj_name]]()

    # THESE TWO FUNCTIONS WERE USED TO CALIBRATE/CHECK the tags
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

    # Resets the object in rviz depending on the AR tag 
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

    # Publich part of a trajectory
    def publish_part(self, plan, point):
        newPlan = copy.deepcopy(plan) # Make a copy of the plan
        # Take up to T or the points and stuff like that
        newPlan.joint_trajectory.points = plan.joint_trajectory.points[:point]
        pdb.set_trace()
        self.publishDisplayTrajectory(newPlan) # Publish part of the new plan

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
            return self.pose_from_AR(id)[0]
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

    def gazebo_plot(self, positions, reference_frame=''):
        if self.plotter_xml is None:
            with open('/home/gwthomas/.gazebo/models/cube_20k/model-smol.sdf', 'r') as f:
                self.plotter_xml = f.read()

        for i in range(self.T_interpolation):
            try:
                self.delete_model('point%d' % i)
            except: pass

        for i, position in enumerate(positions):
            name = 'point%d' % i
            pose = Pose(Point(*position), Quaternion(0,0,0,0))
            self.spawn_model(name, self.plotter_xml, pose, reference_frame=reference_frame)
            self.plotted_names.append(name)

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

    def set_gripper(self, position, wait, max_effort):
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

    def grip(self, wait, max_effort=10000):
        self.set_gripper(0.0, wait, max_effort)

    def ungrip(self, wait, max_effort=10000):
        self.set_gripper(0.08, 50.0, max_effort)

    def compute_plan_cost(self, plan):
        ref_traj = self.compute_reference_trajectory(plan)
        ee_pos = ref_traj['ee']
        diff_magnitudes = [np.sum((ee_pos[i] - ee_pos[i-1])**2) for i in range(1, len(ee_pos))]
        return max(diff_magnitudes)
        # return sum(diff_magnitudes)
        # return len(plan.joint_trajectory.points)

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
        ja = normalize_ja_moveit(target_joint_positions)
        self.group.set_joint_value_target(ja)
        return self.plan(attempts)

    def forward_kinematics1(self, joint_angles, frame):
        header = Header(0, rospy.Time.now(), frame)
        rs = moveit_msgs.msg.RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = joint_angles
        response = self.fk(header, [self.ee_link], rs)
        pose = response.pose_stamped[0].pose
        return np.array(listify(pose.position) + listify(pose.orientation))

    def forward_kinematics_pose(self, joint_angles, frame):
        header = Header(0, rospy.Time.now(), frame)
        rs = moveit_msgs.msg.RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = joint_angles
        response = self.fk(header, [self.ee_link], rs)
        pose = response.pose_stamped[0].pose
        return response.pose_stamped[0]

    def forward_kinematics(self, joint_angles, frame):
        return [self.forward_kinematics1(angles, frame) for angles in joint_angles]

    # Get the inverse kinematics for this
    def inverse_kinematics1(self, pose_stamped):
        request = PositionIKRequest() # Create a new request
        request.ik_link_name = self.ee_link # We want the end effector
        request.pose_stamped = pose_stamped # Desired pose and what not
        request.timeout = Duration(5.0) # Here have another duration or something
        # This is to seed the IK service 
        rs = moveit_msgs.msg.RobotState() # Getting the robot state
        rs.joint_state.name = JOINT_NAMES # This is the current joint value thing
        # I dunno get the current joint values
        rs.joint_state.position = self.group.get_current_joint_values()
        # Not really sure so am just gonna put this down 
        #request.ik_seed_state = rs
        request.robot_state = rs

        # let's call it now and get a response
        response = self.ik(request)
        return response.solution # Return the solution found (hopefully something)

    # Change the condition joint angles to the goal positions for the hyperparams
    def change_conds_to_goals(self):
        # For as many conditions there are
        for i in range(self.conditions):
            joint_val = self.get_initial(i, arm=TRIAL_ARM) # The initial joint values
            the_list = self.forward_kinematics1(joint_val, 'base_footprint')
            # Change the hyperparams so the conditions are now the goals
            self._hyperparams['targets'][i]['position'] = the_list[:3]
            self._hyperparams['targets'][i]['orientation'] = the_list[3:]
            print("This is now the target: " + str(the_list))


    # Get the joint angles of a pose that is some meters above the current pose (of the end
    # effector) that we have 
    def get_ja_above(self, cur_pose, m_above):
        cur_pose.pose.position.z += m_above # Increase the z value by some amount
        solution = self.inverse_kinematics1(cur_pose)
        # Return the array of values lmao
        return solution.joint_state.position

    def get_joint_angles(self):
        if self.current_controller == 'MoveIt':
            state = self.robot.get_current_state().joint_state
            joints = []
            for joint in JOINT_NAMES:
                index = state.name.index(joint)
                joints.append(state.position[index])
            return np.array(joints)
        elif self.current_controller == 'GPS':
            sample = self.get_data(arm=TRIAL_ARM)
            ja = sample.get(JOINT_ANGLES)
            return ja.flatten()
        else:
            raise NotImplementedError

    def close_to_ja(self, ja):
        return np.allclose(self.get_joint_angles(), ja, atol=1e-2)

    def get_initial(self, condition, arm=TRIAL_ARM):
        condition_data = self._hyperparams['reset_conditions'][condition]
        return condition_data[arm]['data']

    def issue_position_command(self, ja, timeout, arm=TRIAL_ARM):
        self.use_controller('GPS')
        if self.close_to_ja(ja):
            print 'Ignoring position command (within tolerance)'
            return

        pos_command = PositionCommand()
        pos_command.mode = JOINT_SPACE
        pos_command.data = ja
        pos_command.pd_gains = self._hyperparams['pid_params']
        pos_command.arm = arm
        pos_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(pos_command, timeout=timeout)

    def reset_arm(self, ja, arm=TRIAL_ARM):
        self.use_controller('GPS')
        if self.close_to_ja(ja):
            print 'Ignoring reset (within tolerance)'
            return

        timeout = self._hyperparams['reset_timeout']
        self.issue_position_command(ja, timeout, arm=arm)

    def reset(self, condition):
        print 'Resetting to condition {} (actually {})'.format(condition, self.actual_conditions[condition])
        for arm in [TRIAL_ARM, AUXILIARY_ARM]:
            ja = self.get_initial(condition, arm=arm)
            try:
                self.reset_arm(ja, arm=arm)
            except TimeoutException:
                print 'Reset timed out for arm', arm

    def move_to(self, position, orientation):
        plan = self.plan_end_effector(position, orientation, attempts=1)
        plan_exists = plan is not None
        if plan_exists:
            self.execute(plan)
        return plan_exists # n.b. this doesn't check if it actually reached the target

    def plan_for_condition(self, condition):
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        return self.plan_end_effector(target['position'], target['orientation'], attempts=self.planning_attempts)

    def _plan_file(self, condition):
        return osp.join(self._hyperparams['exp_dir'], 'plans', 'cond{}.pkl'.format(condition))

    # Creates a RobotState with the specified joint values or something like that
    def create_rs(self, joint_angles):
        rs = moveit_msgs.msg.RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = joint_angles
        return rs # Return the robot state nice

    # Do the initialization of the reset trajectory and policy 
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
    def set_current_as_goal(self, cond):
        pose = self.get_ee_pose() # Get the current pose of the group
        pose = self.get_ee_pose() # Get the current pose of the group

        # Set the goal to the current position for the specified condition
        self._hyperparams['targets'][cond]['position'] = listify(pose.position)
        self._hyperparams['targets'][cond]['orientation'] = list(tf.transformations.euler_from_quaternion( \
            listify(pose.orientation)))

    # Lmao for when using the real robot and you have set the pose
    def get_ee_pose(self):
        return self.group.get_current_pose().pose

    # This is if you want to set the ultimate destination from the
    def set_real_goal(self):
        self.ee_goal = self.get_ee_pose() # Use what is happening in real world 
        self.ja_goal = self.group.get_current_joint_values() # Get the current joint values\

    # Offset the plan depending on the differences
    def offset_whole_plan(self, thePlan):
        if self.ee_goal is None:
            return # If there is no special end goal just end it lol
        goalJoint = np.array(self.ja_goal)
        endGoal = thePlan.joint_trajectory.points[-1].positions # This is the current goal
        diff = np.array(goalJoint) - np.array(endGoal) # This is the difference
        print("This is the difference: " + str(diff)) # Print this
        plen = len(thePlan.joint_trajectory.points)
        for i in range(plen):
            # Add the difference to the plan and hope for the best 
            #np.array(thePlan.joint_trajectory.points[i].positions) + diff
            thePlan.joint_trajectory.points[i].positions = \
            np.array(thePlan.joint_trajectory.points[i].positions) + ((i / float(plen - 1)) * diff)
            print("Offsetting by: " + str(((i / float(plen - 1)) * diff)))

    # Edit the plan according to the different joint angle plan lmao
    def edit_plan_if_necessary(self, thePlan):
        if self.ja_goal is None: # If there is no change to the end goal
            return thePlan # Just return the normal plan
        # Otherwise, we must change the very last point to be the new goal or something?
        thePlan.joint_trajectory.points[-1].positions = self.ja_goal

    # This is if you start at the ending location and then want to move away
    def reverse_plan(self, thePlan):
        newPlan = copy.deepcopy(thePlan)
        newPlan.joint_trajectory.points.reverse() # Reverse the points
        # Lmao just use the list reversing thingq
        numPoints = len(thePlan.joint_trajectory.points) # How many points
        for i in range(numPoints): # Just flip everything around or something
            newPlan.joint_trajectory.points[i].time_from_start = \
                thePlan.joint_trajectory.points[i].time_from_start
            # The velocities have to be reversed too maybe?? no ideas
            newPlan.joint_trajectory.points[i].velocities = \
                np.array(newPlan.joint_trajectory.points[i].velocities) * -1
        return newPlan

    def get_existing_plan(self, condition):
        return self.condition_info[condition].plan
        condition_info[condition].save()

    def execute(self, plan):
        self.use_controller('MoveIt')
        self.group.execute(plan)

    def run_moveit(self, condition):
        self.reset(condition)
        self.execute(self.condition_info[condition].plan)

    def execute(self, plan):
        self.use_controller('MoveIt')
        self.group.execute(plan)

    def run_moveit(self, condition):
        self.reset(condition)
        self.execute(self.condition_info[condition].plan)

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
        # If dumb ilqr, just use target position
        if self.dumb_ilqr:
            # For all the time steps or something
            for i in range(ref_ja_pos.shape[0]):
                ref_ja_pos[i, :] = ref_ja_pos[-1, :]
                ref_ja_vel[i, :] = ref_ja_vel[-1, :]
                ref_ee[i, :] = ref_ee[-1, :]
            pdb.set_trace()

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
            plan = self.plan_for_condition(condition)
            info = self.condition_info[condition]
            info.plan = plan
            info.save()
        self.trajectories[condition] = self.compute_reference_trajectory(plan)
        # Copy these real quick or something like that
        self.full_ref_ee[condition] = np.copy(self.trajectories[condition]['ee'])
        self.full_ref_ja[condition] = np.copy(self.trajectories[condition]['ja_pos'])
        self.full_ref_vel[condition] = np.copy(self.trajectories[condition]['ja_vel'])

    def initialize_controller(self, condition, policy):
        # If we already initialized the controller or we are loading
        if condition in self.initialized or not isinstance(policy, LinearGaussianPolicy) or self.itr_load:
            return
        print("Initializing the controller and what not")
        ref_traj_info = self.trajectories[condition]
        ref_ja_pos = ref_traj_info['ja_pos']
        ref_ja_vel = ref_traj_info['ja_vel']
        ref_ee = ref_traj_info['ee']
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], ref_ja_pos, ref_ja_vel, ref_ee))
        self.initialized.add(condition)

    # Does something different if reset is false
    def sample(self, policy, condition, verbose=True, save=True, noisy=True, otherPol=None):
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
        self.otherPol = otherPol

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

        if not self.reset_time: # If it's not reset time or something
            if self.samples_taken[condition] % (self.num_samples * self.iter_per_seg) == 0 \
                and (self.T != self.final_T or self.varying_T or self.chosen_parts is not None):
                self.update_T_then_policy(policy, condition)
            self.T = self.cur_T[condition] # Make sure the T is correct for the condition we are on
            ref_traj_info = trajectories[condition]
            # This is how long the current trajectory we're using is - self.T
            if self.T == self.final_T: # If we have gotten to the whole trajectory
                ref_ee = trajectories[condition]['ee'] # Current reference trajectory
            else: # Otherwise pad the reference trajectory as well
                ref_ee = list(trajectories[condition]['ee'][:self.T - self.padding])
                ref_ee.extend([trajectories[condition]['ee'][self.T-self.padding-1]] * self.padding)
        else:
            self.T = self.final_T # Reset trajectories always have the full T or something

        print('The length of the trajectory we are currently using is ' + str(self.T))
        #pdb.set_trace()

        if self.traj_plot_period is not None:
            positions = []
            for i, ee in enumerate(ref_ee):
                if i % self.traj_plot_period == 0 and i < self.T_interpolation:
                    position = (ee[3:6] + ee[6:9]) / 2
                    position[2] -= 0.5
                    positions.append(position)
            self.gazebo_plot(positions, reference_frame='torso_lift_link')

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
            self._trial_service.publish(trial_command, wait=True)
            self.trial_manager.run(self._hyperparams['trial_timeout'])
            while self._trial_service._waiting:
                print 'Waiting for sample to come in'
                rospy.sleep(1.0)
            sample_msg = self._trial_service._subscriber_msg

        sample = msg_to_sample(sample_msg, self)
        sample.set(NOISE, noise)
        sample.set(TIMESTEP, np.arange(self.T).reshape((self.T,1)))

        if self.dumb_ilqr: # If we are going to use the dumb ilqr cost fns
            print("Doing the dumb ilqr thing")
            sample.set(REF_OFFSETS, np.zeros((self.T, 9)))
            sample.set(REF_TRAJ, np.zeros(self.T * 9))
        else:
            sample.set(REF_OFFSETS, ref_traj_info['offsets'])
            sample.set(REF_TRAJ, np.array([ref_traj_info['flattened']]*self.T))

        if save and not self.reset_time: # If we are not gonna save this sample as reset
            self._samples[condition].append(sample)
            if self.varying_T: # Save if using varying T
                self.saved_samples[condition].append(sample)
        if save and self.reset_time: # If we are going to save the reset sample
            self._reset_samples[condition].append(sample)

        if not self.reset_time: # If it's not reset time or something weird like that
            self.samples_taken[condition] += 1 # Increment number of samples taken
            if self.samples_taken[condition] % self.num_samples == 0 and self.samples_taken[condition] != 0:
                self.iter_count += 1 # This is the full count

        self.reset_time = False # It's not reset time after this lmaoo???

        return sample

    def get_action(self, policy, obs):
        # extra = ['distances', 'coeffs', 'ee_pos', 'attended', 'direction']
        # extra = ['centered_traj', 'coeffs', 'ee_pos', 'attended']
        extra = []
        action, debug = policy.act(None, obs, None, None, extra=extra)
        # If we have another policy we want to compute the actions for
        if self.otherPol is not None:
            # This is gonna be the linear gaussian policy lmao
            otherAct = self.otherPol.act(obs[:32], None, int(obs[-1] - 1), noise=np.zeros(7))

        f.write("NN: " + str(list(action)) + "\n")
        f.write("ILQG: " + str(list(otherAct)) + "\n")
        if np.any(np.isnan(action)):
            pdb.set_trace()
        return action

    def close_file(self):
        try:
            f.close()
        except:
            return

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
