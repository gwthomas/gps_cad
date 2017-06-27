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

try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None

B4_PEG_JA =  np.array([-0.07127360764327229, 0.09295724750611623, 1.1055297826139094, 
    -0.8294466322295317, -9.253024883350639, -0.9218553679154435, -2.803443744403202])
B4_PEG_EE = np.array([ 0.7069959,  -0.07505008,  0.04737351,  0.85666558, -0.06439661,  0.0972282,
  0.85657629, -0.06450655, -0.0027717 ])

class GearExperiment(AgentCAD):
    def __init__(self, hyperparams, init_node=True):
        AgentCAD.__init__(self, hyperparams, init_node)
        self.dists = [] # Get a list of the distances in the trajs
        self.attempts = 1
        self.samples_taken = [-1] * 5 # How many samples have been taken
        self.cur_frac = [1] * 5 # What fraction
        self.full_ref_ee = [0] * 5 # Lol for the reference ee
        self.full_ref_ja = [0] * 5 # For the full reference ja
        self.prev_pose = None # For storing previous pose of gear lmao

        self.final_T = self.T # This is the original T WOWOWOWOWOWOWOWWOW 
        self.segments = 1.0 # How many segments to break the trajectory into
        self.iter_per_seg = 8 # Let's train this many iterations per segment
        self.iter_count = -1 # Count iterations
        self.padding = 20 # How many timesteps to put as padding for each of the segments
        self.chosen_parts = [145, 230] # SET THIS TO NONE IF YOU WANT TO USE FRACTIONS LMAO

        self.use_saved_traj = True # If we already have one saved (pickled DisplayTrajectory)
        self.pickled_traj_filename = 'saved_robot_traj' # Where you saved the DisplayTrajectory

        self.plan_filename = 'saved_robot_plan' 


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
        # self.add_object('table', position=[0.75,0.,0.42], size=[0.9,1.5,0.03], type='box')
        self.add_object('table', position=[0.75,0.,0.42], size=[0.7,1.5,0.03], type='box')
        gear_box_pose = self.get_pose('gear_box')
        self.add_object('gear_box', position=listify(gear_box_pose.position), size=[0.045, 0.045, 0.392931], type='box')

	sizes = {'the_gear': (0.04, 0.04, 0.04)}

        for name in ['the_gear']:
            pose = self.get_pose(name)
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
	self.move_to(0.555, 0.3, 0.595, 1.57, 0, 0)
        #self.ungrip(15)

    # We need to change this because the gear changes positions
    def grasp(self):
        # Get the pose of the gear before we start grasping
        self.prev_pose = self.get_pose('the_gear') 
        self.grip(None)
        time.sleep(5)
        self.finish_prep()
    
    def finish_prep(self):
	# Get the previous orientation of the gear
	self.scene.remove_world_object('the_gear') # Remove it 
	pose = self.get_pose('the_gear') # Get the pose now that it has changed
	# Now add the gear back to the scene with the new pose
	self.add_object('the_gear', position=listify(pose.position), 
			orientation=listify(pose.orientation),
			size=(0.04, 0.04, 0.04), 
                        filename=self._hyperparams['the_gear'])
        self.attach('the_gear', touch_links=['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', 'l_gripper_l_finger_link'])

    def move_to(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z):
	target_position = [pos_x, pos_y, pos_z]
	target_pose = [orient_x, orient_y, orient_z]
        init_plan = self.plan_end_effector(target_position, target_pose)
        self.group.execute(init_plan)
	
    # Made to change goals depending on the weird left rotation of the gear lololol
    def offset_rotation(self):
        self.use_controller('MoveIt')
        self.move_to(0.555, 0.3, 0.7, 1.57, 0, 0) # Move the gear up
        time.sleep(3) # Wait three seconds
        gear_pose = self.get_pose('the_gear')
        gear_pose = self.get_pose('the_gear') # Get it twice I dunno
        # Get the rotation
        euler = tf.transformations.euler_from_quaternion(listify(gear_pose.orientation))
        euler2 = -1 * euler[2]
        # Start with original pos, gonna calculate a new goal depending on rotation
        new_pos = list(self._hyperparams['targets'][0]['position']) 
        # These are just some arbitrary numbers gotten from observation (not sure if right)
        new_pos[0] -=(abs(0.07 - euler2) * 0.28)
        new_pos[1] += ((0.07 - euler2) * 0.12)

        # For as many conditions there are 
        for i in range(self._hyperparams['conditions']):
            the_tuple = self._hyperparams['targets'][i]['orientation']
            new_thing = [the_tuple[0], the_tuple[1], euler2]
            self._hyperparams['targets'][i]['orientation'] = new_thing
            self._hyperparams['targets'][i]['position'] = new_pos
            print("This is the new position " + str(new_pos))
            print("This is the new orientation " + str(new_thing))


    # Sometimes the gears rotate when they are picked up
    # We need to change the target end-effector positions to compensate for dis
    def change_goals(self, pose, prev_pose=None):
	if prev_pose is None:
		prev_pose = self.prev_pose # Get the instance variable
	prev_ori = prev_pose.orientation # Get previous orientation of gear
	print("this is the previous orientation: " + str(prev_ori))
	ori = pose.orientation # Get the orientation of the gear
	print("this is the cur orientation: " + str(ori))
	quaternion = [ ori.x - prev_ori.x, ori.y - prev_ori.y, 
                       ori.z - prev_ori.z, ori.w - prev_ori.w ] 
	# Make it into the euler thing
	euler = tf.transformations.euler_from_quaternion(quaternion)
	euler_list = np.array([euler[0], euler[1], euler[2]])
	print("This is the difference " + str(euler_list))
	num_cond = self._hyperparams['conditions'] # number of conditions
	# For as many conditions there are 
	for i in range(num_cond):
		# Update the orientation
		new_orient = np.array(self._hyperparams['targets'][i]['orientation']) - euler_list 
		new_orient[2] += 3.14 # Gotta add this lmao
		if new_orient[2] >= 3.14: # Make sure it doesn't go overboard
			new_orient[2] -= 3.14
		self._hyperparams['targets'][i]['orientation'] = new_orient
		print("This is the new orientation " + str(new_orient))


    # Overrides the reset method because we want to move the pole, then reset
    # then move pole back so the PR2's hand doesn't get stuck in the pole
    def reset(self, condition):
        self.move_pole_away() # Move the pole away
        super(GearExperiment, self).reset(condition) # Call the super method
        self.move_pole_back() # Move the box back into position lmao

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
                self.save_plan(plan) # Save the trajectory
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
                    self.save_plan(plan) # Save the trajectory
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
        self.reset(condition)
        target = self._hyperparams['targets'][condition]
        best_ref_ee, best_ref_ja = None, None # HAHAHAHAHA
        best_dist = float("inf") # Infinity itself
        best_plan = None 
        best_T = None # Set this up
        self.T = self.final_T # Gotta use the final T or something like that

        while True:
            for attempt in range(self.attempts): # Let's do asked attempts on plan
                if self.plan_filename is None: # If there is no plan to read
                    plan = self.plan_end_effector(target['position'], target['orientation'])
                else: # Otherwise read the plan from the filename
                    plan = self.load_plan(self.plan_filename)

                if plan is not None:
                    cur_dist = self.get_dist(plan) # Get the distance of cur plan
                    #self.dists.append(cur_dist)
                    # If it beats it we need to use it! Woah!
                    if cur_dist < best_dist:
                        # Only continue on with these calculations if necessary
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

                        best_dist = cur_dist # This is the current best distance
                        best_ref_ee, best_ref_ja, best_plan = ref_ee, ref_ja, plan
                        self.best_saved_traj = self.saved_traj # Save the trajectory and stuff

            if not self.require_approval or yesno('Does this trajectory look ok?'):
                print("this is the distance of the best one: " + str(best_dist))
                break 

        if self.use_saved_traj: # If we already have one
            with open(self.pickled_traj_filename, 'r') as f: # Read from the pickled place
                self.best_saved_traj = pickle.load(f) # Load that pickled DisplayTrajectory!

        self.publishDisplayTrajectory(self.best_saved_traj) # Publish it so it is the last

        #with open('the_distances.txt', 'w') as f:
        #    f.write('These are the distances: \n')
        #    f.write(str(self.dists))
        #    f.write('\nThis is the mean distance: ' + str(np.mean(np.array(self.dists))) + '\n')
        #    f.write('Success rate: ' + str(len(self.dists)) + ' / ' + str(self.attempts) + '\n')

        self.trajectories[condition] = best_ref_ee
        self.save_plan(best_plan) # Save the trajectory
        if self.chosen_parts is None: # If there isn't a specific plan segment chunk
	       # For now just take one / segments of the references to initialize
            start_ind = int(1.0 / self.segments * len(best_ref_ee))
        else: # If there is, just use it
            closest_T = find_closest_T(best_ref_ja, best_ref_ee, B4_PEG_JA, B4_PEG_EE)
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
        with open('starting_traj.txt', 'w') as f:
            for item in best_ref_ee:
                f.write(str(item) + '\n')		
        with open('starting_part.txt', 'w') as f:
            for item in best_ref_ee[:start_ind]:
                f.write(str(item) + '\n')  
        with open('pickled_robot_traj', 'w') as f:
            pickle.dump(self.best_saved_traj, f)

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
        # Length of the trajectory
        traj_length = len(self.trajectories[condition])
        self.samples_taken[condition] += 1 # Increment number of samples taken

        # Every some many samples, take more of the trajectory
        if self.samples_taken[condition] % (self.iter_per_seg * 5) == 0 and self.samples_taken[condition] != 0 and self.T != self.final_T:
            print(str(self.iter_per_seg * 5) + ' samples have passed lmao')
            self.cur_frac[condition] = self.cur_frac[condition] + 1 # Increase the current fraction of samples used
            
            if self.chosen_parts is None:
                # Changing the timesteps currently used
                self.T = int(self.cur_frac[condition] * traj_length / self.segments) 
            else:
                self.T = self.chosen_parts[int(self.cur_frac[condition] - 1)] 

            if self.T >= self.T_interpolation: # If we've reached T_interpolation already
                self.T = self.final_T # Just use the whole trajectory lmao
                if self.chosen_parts is None:
                    self.cur_frac[condition] = self.segments # The full thing lololol
                else:
                    self.cur_frac[condition] = len(self.chosen_parts) # Whole thing??
                ref_ja, ref_ee = self.full_ref_ja[condition], self.full_ref_ee[condition]
            else:
                ref_ja, ref_ee = self.full_ref_ja[condition][:self.T], self.full_ref_ee[condition][:self.T]
                # This is adding more timesteps to the last step in the segment
                ref_ja.extend([self.full_ref_ja[condition][self.T-1]] * self.padding)
                ref_ee.extend([self.full_ref_ee[condition][self.T-1]] * self.padding)
                self.T += self.padding # If not the full trajectory, add the padding

            # We're gonna change policies - initialize with the old stuff learned tho
            self.change_policy(condition, policy, ref_ja, ref_ee)

        if self.chosen_parts is None: # If we're just using fractions and stuff
            # Dunno set this up real quick or something
            self.T = int(self.cur_frac[condition] * traj_length / self.segments) 
            self.T += (int(self.cur_frac[condition] != self.segments) * self.padding)
        else: # Otherwise use this chosen parts array thing
            self.T = self.chosen_parts[int(self.cur_frac[condition] - 1)] 
            self.T += (int(len(self.chosen_parts) != self.cur_frac[int(condition)]) * self.padding)


        # Length of the trajectory
        traj_length = len(self.trajectories[condition])
        # This is how long the current trajectory we're using is - self.T
        if self.T == self.final_T: # If we have gotten to the whole trajectory
            ref_traj = self.trajectories[condition] # Current reference trajectory
        else: # Otherwise pad the reference trajectory as well
            ref_traj = self.trajectories[condition][:self.T - self.padding]
            ref_traj.extend([self.trajectories[condition][self.T-self.padding-1]] * self.padding)

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

        if self.samples_taken[condition] % 5 == 0:
            self.iter_count += 1 # This is the full count
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

        if save:
            self._samples[condition].append(sample)
        self.reset(condition) # Might as well reset for the heck of it
        return sample
                                            
    # This is initializing a new policy with more timesteps using the
    # information from the old policy! :D	 
    def change_policy(self, condition, policy, new_ref_ja, new_ref_ee):
        with open('old_policy.txt', 'w') as f:
            noise = np.zeros((self.T, self.dU))
            f.write(str(policy_to_msg(policy, noise)))

        # Save all of these because we're about to initialize with something newww
        old_K, old_k = policy.K, policy.k 
        pdb.set_trace()
        # This was how many timesteps there were before, not including the padding!
        old_T = old_K.shape[0] - self.padding 
        old_pol_covar, old_chol_pol_covar = policy.pol_covar, policy.chol_pol_covar
        old_inv_pol_covar = policy.inv_pol_covar
        # Now we change the initial values to match what we learned before
        policy.__init__(*init_pd_ref(self._hyperparams['init_traj_distr'], new_ref_ja, new_ref_ee))
        policy.K[:old_T, :, :], policy.k[:old_T, :] = old_K[:old_T, :, :], old_k[:old_T, :]
        policy.pol_covar[:old_T, :, :], policy.chol_pol_covar[:old_T, :, :] = old_pol_covar[:old_T, :, :], old_chol_pol_covar[:old_T, :, :]
        policy.inv_pol_covar[:old_T, :, :] = old_inv_pol_covar[:old_T, :, :]
        
        # Writing the new policy to a text file just so we can examine
        with open('new_policy.txt', 'w') as f:
            noise = np.zeros((self.T, self.dU))
            f.write(str(policy_to_msg(policy, noise)))
