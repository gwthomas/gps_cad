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

try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None

B4_PEG_JA =  np.array([-0.07127360764327229, 0.09295724750611623, 1.1055297826139094, 
    -0.8294466322295317, -9.253024883350639, -0.9218553679154435, -2.803443744403202])
B4_PEG_EE = np.array([ 0.7069959,  -0.07505008,  0.04737351,  0.85666558, -0.06439661,  0.0972282,
  0.85657629, -0.06450655, -0.0027717 ])

ar = {'gear_box': 0, 'the_gear': 1} # Number of the AR tag they have on them

GEAR_BOX, THE_GEAR = 0, 1 # Number of the AR tag they have on them

class GearExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):

        AgentCAD.__init__(self, hyperparams, init_node)
        self.dists = [] # Get a list of the distances in the trajs
        self.sample_dists = [] # For collecting the distances for each sample
        self.attempts = 5
        self.samples_taken = [-1] * 5 # How many samples have been taken
        self.num_samples = 5
        self.cur_frac = [1] * 5 # What fraction
        self.full_ref_ee = [0] * 5 # Lol for the reference ee
        self.full_ref_ja = [0] * 5 # For the full reference ja
        self.prev_pose = None # For storing previous pose of gear lmao

        self.cur_T = [0] * 5 # For storing the T of each of the conditions!
        self.final_T = self.T # This is the original T WOWOWOWOWOWOWOWWOW 
        self.varying_T = False # If you want T to vary depending on a whole bunch of stuff
        self.the_tolerance = 0.03 # If the difference is that large it's concerning
        self.segments = 1.0 # How many segments to break the trajectory into
        self.iter_per_seg = 3 # Let's train this many iterations per segment
        self.iter_count = 0 # Count iterations
        self.padding = 20 # How many timesteps to put as padding for each of the segments
        self.chosen_parts = None #[145, 230] # SET THIS TO NONE IF YOU WANT TO USE FRACTIONS LMAO

        self.use_saved_traj = False # If we already have one saved (pickled DisplayTrajectory)
        self.pickled_traj_filename = 'pickled_robot_traj_cond0' # Where you saved the DisplayTrajectory
        self.plan_filename = 'new_saved_robot_plan' 
        self.data_files_dir = hyperparams['data_files_dir']
        self.saved_samples = [[] * 5] # Just for storing this real quick

        self.unpickle_and_set(hyperparams) # Check if we are resuming from some state

        pdb.set_trace()     # for optional setup, not debugging

    def __getstate__(self):
        # Saving all these nice important things or something lmaooo
        d = {} 
        d['dists'] = self.dists 
        d['attempts'] = self.attempts 
        d['samples_taken'] = self.samples_taken 
        d['cur_frac'] = self.cur_frac
        d['full_ref_ee'] = self.full_ref_ee 
        d['full_ref_ja'] = self.full_ref_ja 
        d['prev_pose'] = self.prev_pose 
        d['final_T'] = self.final_T 
        d['segments'] = self.segments
        d['iter_per_seg'] = self.iter_per_seg
        d['iter_count'] = self.iter_count
        d['padding'] = self.padding 
        d['chosen_parts'] = self.chosen_parts 
        d['sample_dists'] = self.sample_dists
        d['use_saved_traj'] = self.use_saved_traj
        d['pickled_traj_filename'] = self.pickled_traj_filename 
        d['plan_filename'] = self.plan_filename
        d['best_saved_traj'] = self.best_saved_traj 
        d['saved_traj'] = self.saved_traj
        d['trajectories'] = self.trajectories
        d['current_controller'] = self.current_controller
        d['varying_T'] = self.varying_T
        d['the_tolerance'] = self.the_tolerance
        d['cur_T'] = self.cur_T
        d['_samples'] = self._samples
        d['num_samples'] = self.num_samples
        d['saved_samples'] = self.saved_samples
        d['ar_functions'] = self.ar_functions
        return d

    # Check if we need to unpickle anything (if we are resuming from state or not)
    def unpickle_and_set(self, hyperparams):
        if 'resume_itr' not in hyperparams: # If we are not resuming from some state
            return # Just leave this method
        agt = self.unpickle_agent(hyperparams['resume_itr']) # Unpickle!
        self.dists = agt.dists # Get a list of the distances in the trajs
        self.attempts = agt.attempts
        self.samples_taken = agt.samples_taken # How many samples have been taken
        self.num_samples = agt.num_samples
        self.cur_frac = agt.cur_frac # What fraction
        self.full_ref_ee = agt.full_ref_ee # Lol for the reference ee
        self.full_ref_ja = agt.full_ref_ja # For the full reference ja
        self.prev_pose = agt.prev_pose # For storing previous pose of gear lmao

        self.cur_T = agt.cur_T 
        self.final_T = agt.final_T # This is the original T WOWOWOWOWOWOWOWWOW 
        self.varying_T = agt.varying_T
        self.the_tolerance = agt.the_tolerance
        self.segments = agt.segments # How many segments to break the trajectory into
        self.iter_per_seg = agt.iter_per_seg # Let's train this many iterations per segment
        self.iter_count = agt.iter_count # Count iterations
        self.padding = agt.padding # How many timesteps to put as padding for each of the segments
        self.chosen_parts = agt.chosen_parts # SET THIS TO NONE IF YOU WANT TO USE FRACTIONS LMAO

        self.use_saved_traj = agt.use_saved_traj # If we already have one saved (pickled DisplayTrajectory)
        self.pickled_traj_filename = agt.pickled_traj_filename # Where you saved the DisplayTrajectory
        self.plan_filename = agt.plan_filename
        self.best_saved_traj = agt.best_saved_traj
        self.saved_traj = agt.saved_traj
        self.trajectories = agt.trajectories
        self.current_controller = agt.current_controller
        self.sample_dists = agt.sample_dists
        self._samples = agt._samples
        self.saved_samples = agt.saved_samples

    def setup(self):
        self.ar_functions[ar['the_gear']] = self.create_AR_function(ar['the_gear'], -0.0036, 0, -0.2222, 0, 0, 0)
        self.ar_functions[ar['gear_box']] = self.create_AR_function(ar['gear_box'], 0.023, 0.01, -0.17, 0, 0, 0)
        self.move_head(6, 0, -3) # Move head so we can see what we need to see
        # Move arm down so we can see what is going on!!!
        self.move_to(0.45, 0.25, 0.55, 0, 0, 0) 
        self.configure_scene()
        self.grasp_prep()

    def unpickle_agent(self, itr):
        pickle_filename = self.data_files_dir + 'agent_itr_' + str(itr) + '.pkl'
        with open(pickle_filename, 'r') as f: # Read to this pickled place
            gear_thing = pickle.load(f) # Read the pickled thing
            return gear_thing

    def configure_scene(self):
        print 'Clearing planning scene'
        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.ee_link)

        print 'Resetting piece'
        self.reset_piece()
        time.sleep(2) # Wait for a little bit of time

        print 'Adding objects to planning scene'
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.75,0.,0.42], size=[0.7,1.5,0.03], type='box')
        gear_box_pose = self.get_pose('gear_box')
        #gear_box_pose, euler = self.ar_functions[ar['gear_box']]()
        self.add_object('gear_box', position=listify(gear_box_pose.position), size=[0.04, 0.04, 0.392931], type='box')

        sizes = {'the_gear': (0.04, 0.04, 0.04)}

        for name in ['the_gear']:
            pose = self.get_pose(name)
            # Get the position of the gear by AR tag
            #pose, euler = self.ar_functions[ar['the_gear']]()
            self.add_object(name, position=listify(pose.position),
                    orientation=listify(pose.orientation),
                    size=sizes[name],
                    filename=self._hyperparams[name])

    def reset_piece(self):
        quat = Quaternion(*quaternion_from_euler(0, 0, 0))
        pose = Pose(Point(0.83431, 0.301764, 0.586395), quat)
        self.set_pose('the_gear', pose)

    def grasp_prep(self):
        self.use_controller('MoveIt')
        self.ungrip(15)
        self.ungrip(15) # UGHHHH DO IT AGAIN ITS SO DUMB
        time.sleep(2)
        #gear_pose, euler = self.ar_functions[ar['the_gear']]() # Gear has AR marker 1 on it
        gear_pose = self.get_pose('the_gear')
        pos = gear_pose.position
        # Move to the position depending on where the AR tag was
        self.move_to(pos.x - 0.284, pos.y, pos.z + 0.0122, 1.57, 0, 0) #euler[2])
        time.sleep(2) # Wait for a little bit
        self.reset_piece() # Put the gear back where it belongs lmao
        #self.move_to(0.555, 0.3, 0.595, 1.57, 0, 0)

    # We need to change this because the gear changes positions
    def grasp(self):
        self.grip(None)
        time.sleep(3)
        self.finish_prep()

    def finish_prep(self):
    	# Get the previous orientation of the gear
    	self.scene.remove_world_object('the_gear') # Remove it 
    	#pose, euler = self.ar_functions[ar['the_gear']]() # Get the pose now that it has changed
        pose = self.get_pose('the_gear')
        time.sleep(1) # Sleep for 2 seconds
    	# Now add the gear back to the scene with the new pose
    	self.add_object('the_gear', position=listify(pose.position), 
			orientation=listify(pose.orientation),
			size=(0.04, 0.04, 0.04), 
                        filename=self._hyperparams['the_gear'])
        self.attach('the_gear', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link'])# ,'l_gripper_l_finger_link'])

    def pickle_self(self):
        pickle_filename = self.data_files_dir + 'agent_itr_' + str(self.iter_count) + '.pkl'
        with open(pickle_filename, 'w') as f: # Write to this pickled place
            pickle.dump(self, f) # Dump this thing in there

    # Given a position and orientation, attempt to move there! :)
    def move_to_pose_and_ori(self, position, orientation):
        init_plan = self.plan_end_effector(position, orientation)
        self.group.execute(init_plan)

    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
    	target_position = [pos_x, pos_y, pos_z]
    	target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)
	
    # Made to change goals depending on the weird left rotation of the gear lololol
    def offset_rotation(self):
        self.use_controller('MoveIt')
        self.move_to(0.555, 0.3, 0.7, 1.57, 0, 0) # Move the gear up
        time.sleep(2) # Wait three seconds
        #gear_pose, euler = self.ar_functions[ar['the_gear']]() # Get the gear pose from AR
        #gear_pose, euler = self.ar_functions[ar['the_gear']]() # Get the gear pose from AR
        gear_pose = self.get_pose('the_gear')
        euler = list(tf.transformations.euler_from_quaternion(listify(gear_pose.orientation)))
        print(gear_pose)
        print(euler)
        euler2 = -1 * euler[2] # Weird and hacky but okay???

        self.move_to(0.555, 0.3, 0.7, 1.57, 0, euler2) # Rotate the gear now
        time.sleep(3)
        print('getting gear_pose')
        #gear_pose, euler = self.ar_functions[ar['the_gear']]() # Now get the full gear pose
        #gear_pose, euler = self.ar_functions[ar['the_gear']]() # Now get the full gear pose
        gear_pose = self.get_pose('the_gear')
        print('getting robot_pose')
        robot_pose = self.group.get_current_pose().pose
        robot_pose = self.group.get_current_pose().pose # Get the current pose
        robot_pos = robot_pose.position # Get the position of the link

        # Start with original pos, gonna calculate a new goal depending on rotation
        new_pos = list(self._hyperparams['targets'][0]['position']) 
        
        #gear_box_pose, euler = self.ar_functions[ar['gear_box']]() # Get the position of the box by AR tag
        #gear_box_pose, euler = self.ar_functions[ar['gear_box']]() # Get the position of the box by AR tag
        gear_box_pose = self.get_pose('gear_box')

        #new_pos[1] = robot_pose.pose.position.y - 0.372 + ((0.07 - euler2) * 0.09)
        #new_pos[0] = robot_pose.pose.position.x + 0.102
        #new_pos[0] = robot_pose.pose.position.x + gear_box_pose.position.x - 0.845
        #new_pos[1] = robot_pose.pose.position.y + ((0.07 - euler2) * 0.2) + \
        #    (gear_box_pose.position.y - 0.316)
        # Calculate where it needs to be depending on where the gear and gear box are currently
        #new_pos[0] = gear_box_pose.position.x - 0.2805
        #new_pos[1] = gear_box_pose.position.y - 0.00335
        new_pos[0] = robot_pos.x + (gear_box_pose.position.x - 0.0006 - gear_pose.position.x)
        new_pos[1] = robot_pos.y + (gear_box_pose.position.y + 0.001 - gear_pose.position.y) 

        # For as many conditions there are 
        for i in range(self._hyperparams['conditions']):
            the_tuple = self._hyperparams['targets'][i]['orientation']
            new_thing = [the_tuple[0], the_tuple[1], euler2]
            self._hyperparams['targets'][i]['orientation'] = new_thing
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))
            print("This is the new orientation " + str(new_thing))

    def redo_offset(self):
        gear_pose, euler = self.ar_functions[ar['the_gear']]() # Now get the full gear pose
        gear_pose, euler = self.ar_functions[ar['the_gear']]() # Now get the full gear pose
        print('getting robot_pose')
        robot_pose = self.group.get_current_pose().pose
        robot_pose = self.group.get_current_pose().pose # Get the current pose
        robot_pos = robot_pose.position # Get the position of the link

        # Start with original pos, gonna calculate a new goal depending on rotation
        new_pos = list(self._hyperparams['targets'][0]['position']) 
        
        gear_box_pose, euler = self.ar_functions[ar['gear_box']]() # Get the position of the box by AR tag
        gear_box_pose, euler = self.ar_functions[ar['gear_box']]() # Get the position of the box by AR tag

        new_pos[0] = robot_pos.x + (gear_box_pose.position.x - 0.0006 - gear_pose.position.x)
        new_pos[1] = robot_pos.y + (gear_box_pose.position.y + 0.001 - gear_pose.position.y) 

        # For as many conditions there are 
        for i in range(self._hyperparams['conditions']):
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))

    # Overrides the reset method because we want to move the pole, then reset
    # then move pole back so the PR2's hand doesn't get stuck in the pole
    def reset(self, condition):
        condition_data = self._hyperparams['reset_conditions'][condition]
        #pass
        goal_ja = self.get_ja_above(self.group.get_current_pose(), 0.05)
        self.reset_arm_and_wait(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
            goal_ja, 3.0)
        #self.move_pole_away() # Move the pole away
        #super(GearExperiment, self).reset(condition) # Call the super method
        #self.move_pole_back() # Move the box back into position lmao

    # Moves the pole in the scene away from the PR2
    def move_pole_away(self):
        quat = Quaternion(*quaternion_from_euler(0, 0, 0))
        pose = Pose(Point(0.935511, -0.054672, 0), quat)
        self.set_pose('gear_box', pose)

    # Moves the box in the scene back to the original experiment position
    def move_pole_back(self):
        quat = Quaternion(*quaternion_from_euler(0, 0, 0))
        pose = Pose(Point(0.935511, -0.054672, 0.626407), quat)
        self.set_pose('gear_box', pose)

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

    # Override lmao o o o o o o o 
    def compute_reference_trajectory(self, condition, policy):
        super(GearExperiment, self).reset(condition) # Call the super method
        target = self._hyperparams['targets'][condition]
        best_ref_ee, best_ref_ja = None, None # HAHAHAHAHA
        best_dist = float("inf") # Infinity itself
        best_plan = None 
        best_T = None # Set this up
        self.T = self.final_T # Gotta use the final T or something like that

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
            
            copy_plan = copy.deepcopy(best_plan) # Copy the plan
            self.reverse_plan(copy_plan) # Reverse the plan
            self.reset_plans[condition] = copy_plan # This is the plan to get out of here!

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

        if self.use_saved_traj: # If we already have one
            with open(self.pickled_traj_filename, 'r') as f: # Read from the pickled place
                self.best_saved_traj[condition] = pickle.load(f) # Load that pickled DisplayTrajectory!

        self.publishDisplayTrajectory(self.best_saved_traj[condition]) # Publish it so it is the last

        #with open('the_distances.txt', 'w') as f:
        #    f.write('These are the distances: \n')
        #    f.write(str(self.dists))
        #    f.write('\nThis is the mean distance: ' + str(np.mean(np.array(self.dists))) + '\n')
        #    f.write('Success rate: ' + str(len(self.dists)) + ' / ' + str(self.attempts) + '\n')

        self.save_plan(best_plan) # Save the trajectory
        if self.varying_T: # If we are going for the varying T plan
            start_ind = self.final_T # Start off with the full timestep length
        if self.chosen_parts is None: # If there isn't a specific plan segment chunk
	       # For now just take one / segments of the references to initialize
            start_ind = int(1.0 / self.segments * len(best_ref_ee))
        else: # If there is, just use it
            #closest_T = find_closest_T(best_ref_ja, best_ref_ee, B4_PEG_JA, B4_PEG_EE)
            closest_T = 139
            self.chosen_parts[0] = closest_T - 10 # Just a little bit of buffer lmao
            start_ind = self.chosen_parts[0] # Set the starting ind to what we have chosen
            print("This is the closest found T: " + str(closest_T))

	    # Provide copies of the reference ee and ja
        self.full_ref_ee[condition] = list(np.copy(np.array(best_ref_ee)))
        self.full_ref_ja[condition] = list(np.copy(np.array(best_ref_ja)))
        self.T = start_ind # How many there is to use

        # If we are just taking part of the trajectory
        if start_ind != self.final_T: # Add padding
            new_ref_ja, new_ref_ee = best_ref_ja[:start_ind], best_ref_ee[:start_ind]
            # This is adding more timesteps to the last step in the segment
            new_ref_ja.extend([best_ref_ja[start_ind-1]] * self.padding)
            new_ref_ee.extend([best_ref_ee[start_ind-1]] * self.padding)
            self.T += self.padding # Put in the padding lmao 
        else: # Otherwise, the goal already has padding
            new_ref_ja, new_ref_ee = best_ref_ja, best_ref_ee
	    # Initialize using the beginning part of the trajectory
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], new_ref_ja, new_ref_ee)) 

        with open('pickled_robot_traj_cond' + str(condition), 'w') as f:
            pickle.dump(self.best_saved_traj[condition], f)

        ref_offsets = np.array(best_ref_ee) - best_ref_ee[-1]

        traj_info = {
            'ja': np.array(best_ref_ja),
            'ee': np.array(best_ref_ee),
            'offsets': ref_offsets,
            'flattened': ref_offsets.flatten()
        }
        self.cur_T[condition] = self.T # Update this as well
        self.trajectories[condition] = traj_info
        return traj_info

    # Calculate the T depending on how much of the trajectory we are using
    def change_T(self, condition):   
        # Length of the trajectory
        traj_length = len(self.trajectories[condition]['ee'])
        cur_T = self.cur_T[condition] # What we have right now!!

        # If we are using varying T, and it's time to check if we need to change
        if self.varying_T:
            the_cutoff = self.calculate_cutoff(condition) # Calculate the cutoff!
            if the_cutoff is None: # If we don't need a cutoff or something
                self.T = (cur_T - self.padding) * 2 # Just double the length of traj or something
            else: # Otherwise let's use the cutoff suggested
                self.T = the_cutoff - 30 # LOL SOME PADDING
        elif self.chosen_parts is None:
            # Changing the timesteps currently used
            self.T = int(self.cur_frac[condition] * traj_length / self.segments) 
        else:
            self.T = self.chosen_parts[int(self.cur_frac[condition] - 1)] 

    # This is for updating the T and then the policy after we've passed a certain number
    # of iterations! 
    def update_T_then_policy(self, policy, condition):
        print(str(self.iter_per_seg * 5) + ' samples have passed lmao')
        self.cur_frac[condition] = self.cur_frac[condition] + 1 # Increase the current fraction of samples used
        self.change_T(condition) # Call this now that everything has changed

        if self.T >= self.T_interpolation: # If we've reached T_interpolation already
            self.T = self.final_T # Just use the whole trajectory lmao
            if self.chosen_parts is None:
                self.cur_frac[condition] = self.segments # The full thing lololol
            else:
                self.cur_frac[condition] = len(self.chosen_parts) # Whole thing??
            ref_ja, ref_ee = self.full_ref_ja[condition], self.full_ref_ee[condition]
        else:
            ref_ja, ref_ee = list(self.full_ref_ja[condition][:self.T]), list(self.full_ref_ee[condition][:self.T])
            # This is adding more timesteps to the last step in the segment
            ref_ja.extend([self.full_ref_ja[condition][self.T-1]] * self.padding)
            ref_ee.extend([self.full_ref_ee[condition][self.T-1]] * self.padding)
            self.T += self.padding # If not the full trajectory, add the padding

        self.cur_T[condition] = self.T # Update! 

        # We're gonna change policies - initialize with the old stuff learned tho
        self.change_policy(condition, policy, ref_ja, ref_ee)

    def calculate_cutoff(self, condition):
        use_prev = 5 # Randomly use the last 5 samples to figure this out
        check_every = 10 # Check every 10 samples for the proper cutoff

        T = self.saved_samples[condition][-1].T # Use the T of the samples
        chosen_samples = self.saved_samples[condition][-1 * use_prev:] # Get the last couple of samples
        avg = np.zeros(self.saved_samples[condition][-1].get(REF_OFFSETS).shape) # Get the shape

        for sample in chosen_samples: # For all the samples we are using
            tgt = sample.get(REF_OFFSETS)
            pt = sample.get(END_EFFECTOR_POINTS)
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
        for i in range(5 * check_every, T, check_every):
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
        pdb.set_trace()

        self.saved_samples[condition] = [] # Clear this as well

        if largest_diff >= self.the_tolerance:
            return best_cutoff
        else:
            return None

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
                self.compute_reference_trajectory(condition, policy)

        self.samples_taken[condition] += 1 # Increment number of samples taken

        # Every some many samples, take more of the trajectory
        # Unless we have reached the end or we are using varying T
        if self.samples_taken[condition] % (self.num_samples * self.iter_per_seg) == 0 and self.iter_count != 0 \
        and (self.T != self.final_T or self.varying_T):
            self.update_T_then_policy(policy, condition)

        self.T = self.cur_T[condition] # Make sure the T is correct for the condition we are on

        ref_traj_info = trajectories[condition]
        # Length of the trajectory
        traj_length = len(self.full_ref_ee[condition])
        # This is how long the current trajectory we're using is - self.T
        if self.T == self.final_T: # If we have gotten to the whole trajectory
            ref_traj = ref_traj_info['ee'] # Current reference trajectory
        else: # Otherwise pad the reference trajectory as well
            ref_traj = list(ref_traj_info['ee'][:self.T - self.padding])
            ref_traj.extend([ref_traj_info['ee'][self.T-self.padding-1]] * self.padding)

        print('The length of the trajectory we are currently using is ' + str(self.T))
        print 'Sampling, condition', condition
        self.reset(condition)

        #added from agent_ros.py of public gps codebase
        if TfPolicy is not None:  # user has tf installed.
            if isinstance(policy, TfPolicy):
                print('well this got called')
                self._init_tf(policy.dU)

        # Generate noise.
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))

        # Execute trial.
        trial_command = TrialCommand()
        #trial_command.id = self._get_next_seq_id()
        trial_command.controller = policy_to_msg(policy, noise)

        if self.samples_taken[condition] % self.num_samples == 0 and self.samples_taken[condition] != 0:
            self.iter_count += 1 # This is the full count
            self.pickle_self() # Pickle self and send to data files lmaooo
            with open('iter' + str(self.iter_count) + '_cond' + str(condition) + '.txt', 'w') as f:
                the_noise = np.zeros((self.T, self.dU))
                f.write(str(policy_to_msg(policy, the_noise)))  

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

        sample.set(REF_OFFSETS, ref_traj_info['offsets'][:self.T])
        sample.set(REF_TRAJ, np.array([ref_traj_info['offsets'].flatten()]*self.T))

        if save:
            self._samples[condition].append(sample)
        if self.varying_T: # Only save this if you are gonna use varying T
            self.saved_samples[condition].append(sample) # Save it here too just in case
        self.reset(condition) # Might as well reset for the heck of it
        return sample
                                            
    # This is initializing a new policy with more timesteps using the
    # information from the old policy! :D	 
    def change_policy(self, condition, policy, new_ref_ja, new_ref_ee):

        # Save all of these because we're about to initialize with something newww
        old_K, old_k = policy.K, policy.k 
        # This was how many timesteps there were before, not including the padding!
        old_T = old_K.shape[0] - self.padding 

        with open('old_policy.txt', 'w') as f:
            noise = np.zeros((old_K.shape[0], self.dU))
            f.write(str(policy_to_msg(policy, noise)))

        old_pol_covar, old_chol_pol_covar = policy.pol_covar, policy.chol_pol_covar
        old_inv_pol_covar = policy.inv_pol_covar
        old_T = min(old_T, self.T) # Make sure to choose the smaller one I think (???)
        # Now we change the initial values to match what we learned before
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], new_ref_ja, new_ref_ee))
        policy.K[:old_T, :, :], policy.k[:old_T, :] = old_K[:old_T, :, :], old_k[:old_T, :]
        policy.pol_covar[:old_T, :, :], policy.chol_pol_covar[:old_T, :, :] = old_pol_covar[:old_T, :, :], old_chol_pol_covar[:old_T, :, :]
        policy.inv_pol_covar[:old_T, :, :] = old_inv_pol_covar[:old_T, :, :]
        
        # Writing the new policy to a text file just so we can examine
        with open('new_policy.txt', 'w') as f:
            noise = np.zeros((self.T, self.dU))
            f.write(str(policy_to_msg(policy, noise)))
