""" This file defines the main object that runs experiments. """

import matplotlib as mpl
mpl.use('Qt4Agg')

import logging
import imp
import os
import os.path
import sys
import copy
import argparse
import threading
import time
import traceback
import pdb
import pickle

# Add gps/python to path so that imports work.
sys.path.append('/'.join(str.split(__file__, '/')[:-2]))
from gps.gui.gps_training_gui import GPSTrainingGUI
from gps.utility.data_logger import DataLogger
from gps.sample.sample_list import SampleList
from gps.algorithm.algorithm_utils import IterationData
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.algorithm_mdgps import AlgorithmMDGPS

from gps.algorithm.policy_opt.policy_opt_tf import PolicyOptTf

class GPSMain(object):
    """ Main class to run algorithms and experiments. """
    def __init__(self, config, quit_on_end=False):
        """
        Initialize GPSMain
        Args:
            config: Hyperparameters for experiment
            quit_on_end: When true, quit automatically on completion
        """
        self.special_reset = False # L M A O
        self._quit_on_end = quit_on_end
        self._hyperparams = config
        self._conditions = config['common']['conditions']
        # There's gonna be as many reset conditions as conditions l m a o
        self._reset_conditions = self._conditions # Heh
        if 'train_conditions' in config['common']:
            self._train_idx = config['common']['train_conditions']
            self._test_idx = config['common']['test_conditions']
        else:
            self._train_idx = range(self._conditions)
            config['common']['train_conditions'] = config['common']['conditions']
            self._hyperparams = config
            self._test_idx = self._train_idx

        self._data_files_dir = config['common']['data_files_dir']

        self.agent = config['agent']['type'](config['agent'])
        self.data_logger = DataLogger()
        self.gui = GPSTrainingGUI(config['common']) if config['gui_on'] else None

        config['algorithm']['agent'] = self.agent
        self.algorithm = config['algorithm']['type'](config['algorithm'])

        import tensorflow as tf
        with tf.variable_scope('reset'): # to avoid variable naming conflicts
            # Gonna make the algorithm for the reset ones as well
            self.reset_algorithm = config['reset_algorithm']['type'](config['algorithm'])

        self.saved_algorithm = copy.deepcopy(self.algorithm) # Save this newly initialized thing or something

        self.diff_warm_start = True # If you want to warm start the algorithm BADMM with normal iLQG stuff
        self.nn_warm_start = False # If you want to warm start the neural network
        attention, structure = 'time', 'mlp' # Change these if you want to take the other policy type
        self.policy_path = os.path.join(self._data_files_dir, os.path.join('{}_{}'.format(attention, structure), 'policy'))
        try:
            self.old_policy_opt = copy.deepcopy(self.algorithm.policy_opt)
        except:
            pass

        pdb.set_trace()

    # Specially initialize the algorithm after we have loaded things or whatever
    def special_init_alg(self):

        resumed_alg = self.algorithm # We picked this up by resuming
        # SPECIFIC TO BADMM SO SORRY IF THIS BREAKS EVERYTHING
        if (type(self.saved_algorithm) is AlgorithmBADMM and not(type(resumed_alg) is AlgorithmBADMM)) or
        (type(self.saved_algorithm) is AlgorithmMDGPS and not(type(resumed_alg) is AlgorithmMDGPS)):
            self.algorithm = self.saved_algorithm # Return it to the new type we want to use
            # Keep a copy of these hyperparams and stuff
            theParams = copy.deepcopy(self.algorithm._hyperparams)
            # For all the instance variables in the resumed algorithm
            for item in resumed_alg.__dict__:
                # Set the attributes accordingly or something like that
                setattr(self.algorithm, item, resumed_alg.__dict__[item])

            # Except for the hyperparams, those need to be saved or something
            self.algorithm._hyperparams = theParams
            self.algorithm.re_init_pol_info(theParams) # Reinitialize this
            # Get rid of the prev data, this messes up the linear algebra stuff
            self.algorithm.prev = [IterationData() for _ in range(self.algorithm.M)]
            self.algorithm.iteration_count = 0 # Pretend this is the first iteration lmao

        pdb.set_trace()

    def run(self, itr_load=None):
        """
        Run training by iteratively sampling and taking an iteration.
        Args:
            itr_load: If specified, loads algorithm state from that
                iteration, and resumes training at the next iteration.
        Returns: None
        """
        try:
            itr_start = self._initialize(itr_load)

            for itr in range(itr_start, self._hyperparams['iterations']):
                for cond in self._train_idx:
                    for i in range(self._hyperparams['num_samples']):
                        self._take_sample(itr, cond, i)
                        if self.special_reset: # If there is a special reset
                            # Take a special sample
                            self._take_sample(itr, cond, i, reset=True)

                traj_sample_lists = [
                    self.agent.get_samples(cond, -self._hyperparams['num_samples'])
                    for cond in self._train_idx
                ]
                if self.special_reset: # Again, if there is a special reset
                    reset_traj_sample_lists = [
                    self.agent.get_reset_samples(cond, -self._hyperparams['num_samples'])
                    for cond in self._train_idx
                    ]
                    self._take_iteration(itr, reset_traj_sample_lists, reset=True)
                    #reset_pol_sample_lists = self._take_reset_policy_samples()
                    ##### PROBABLY NEED TO LOG THE DATA TOO BUT OH WELL


                # Clear agent samples. (Including the reset ones lmao)
                self.agent.clear_samples()
                #pdb.set_trace()
                self._take_iteration(itr, traj_sample_lists)
                pol_sample_lists = self._take_policy_samples()
                self._log_data(itr, traj_sample_lists, pol_sample_lists)
        except Exception as e:
            traceback.print_exception(*sys.exc_info())
        finally:
            self._end()

    # Unpickles trajectory information so we can train from it or something
    def unpickle_traj_and_train(self, itr):
        # Get the trajectory sample lists
        traj_sample_lists = self.data_logger.unpickle(self._data_files_dir +
            ('traj_sample_itr_%02d.pkl' % itr))
        # Save the policy we have
        saved_policy = self.algorithm.policy_opt
        # Restore new algorithm that we presumably want??
        self.algorithm.policy_opt = self.saved_policy
        self.algorithm._take_iteration(traj_sample_lists)
        pdb.set_trace()
        # Then let's take like one policy sample (??)
        pol_sample_lists = self._take_policy_samples(1)

    def test_policy(self, itr, N):
        """
        Take N policy samples of the algorithm state at iteration itr,
        for testing the policy to see how it is behaving.
        (Called directly from the command line --policy flag).
        Args:
            itr: the iteration from which to take policy samples
            N: the number of policy samples to take
        Returns: None
        """
        algorithm_file = self._data_files_dir + 'algorithm_itr_%02d.pkl' % itr
        self.algorithm = self.data_logger.unpickle(algorithm_file)
        if self.algorithm is None:
            print("Error: cannot find '%s.'" % algorithm_file)
            os._exit(1) # called instead of sys.exit(), since t
        traj_sample_lists = self.data_logger.unpickle(self._data_files_dir +
            ('traj_sample_itr_%02d.pkl' % itr))

        pol_sample_lists = self._take_policy_samples(N)
        self.data_logger.pickle(
            self._data_files_dir + ('pol_sample_itr_%02d.pkl' % itr),
            copy.copy(pol_sample_lists)
        )

        if self.gui:
            self.gui.update(itr, self.algorithm, self.agent,
                traj_sample_lists, pol_sample_lists)
            self.gui.set_status_text(('Took %d policy sample(s) from ' +
                'algorithm state at iteration %d.\n' +
                'Saved to: data_files/pol_sample_itr_%02d.pkl.\n') % (N, itr, itr))

    def _initialize(self, itr_load):
        """
        Initialize from the specified iteration.
        Args:
            itr_load: If specified, loads algorithm state from that
                iteration, and resumes training at the next iteration.
        Returns:
            itr_start: Iteration to start from.
        """
        if itr_load is None:
            if self.gui:
                self.gui.set_status_text('Press \'go\' to begin.')
            return 0
        else:
            print("HELLO THIS IS STARTING OFF FROM ITR_LOAD" + str(itr_load))
            algorithm_file = self._data_files_dir + 'algorithm_itr_%02d.pkl' % itr_load
            self.algorithm = self.data_logger.unpickle(algorithm_file)
            if self.algorithm is None:
                print("Error: cannot find '%s.'" % algorithm_file)
                os._exit(1) # called instead of sys.exit(), since this is in a thread
            if self.diff_warm_start: # If we are warm starting the algorithm
                self.special_init_alg() # Call the special initialization method lmao
            if self.nn_warm_start: # If we are warm starting the neural network
                # Restore the policy opt with the policy in the given policy path or something like that
                self.algorithm.policy_opt.restore_model(self.policy_path)

            self.agent.itr_load = itr_load # Set the iter load
            pdb.set_trace()
            # unpickle the agent and whatever
            #agent_file = self._data_files_dir + 'agent_itr_%02d.pkl' % itr_load
            #self.agent = self.data_logger.unpickle(agent_file)
            #if self.agent is None:
            #    print("Error: cannot find '%s.'" % agent_file)
            #    os._exit(1) # called instead of sys.exit(), since this is in a thread

            if self.gui:
                traj_sample_lists = self.data_logger.unpickle(self._data_files_dir +
                    ('traj_sample_itr_%02d.pkl' % itr_load))
                if self.algorithm.cur[0].pol_info:
                    pol_sample_lists = self.data_logger.unpickle(self._data_files_dir +
                        ('pol_sample_itr_%02d.pkl' % itr_load))
                else:
                    pol_sample_lists = None
                self.gui.set_status_text(
                    ('Resuming training from algorithm state at iteration %d.\n' +
                    'Press \'go\' to begin.') % itr_load)
            return itr_load + 1

    # The reset is if this sample is a reset sample
    def _take_sample(self, itr, cond, i, reset=False):
        """
        Collect a sample from the agent.
        Args:
            itr: Iteration number.
            cond: Condition number.
            i: Sample number.
        Returns: None
        """
        if self.algorithm._hyperparams['sample_on_policy'] \
                and self.algorithm.iteration_count > 0:
            # Use the reset algorithm policy if this is a reset sample
            if reset:
                pol = self.reset_algorithm.policy_opt.policy
            else: # Otherwise we are gonna use the primary algorithm
                pol = self.algorithm.policy_opt.policy
        else:
            if reset:
                pol = self.reset_algorithm.cur[cond].traj_distr
            else:
                pol = self.algorithm.cur[cond].traj_distr
        #pdb.set_trace()
        if self.gui:
            self.gui.set_image_overlays(cond)   # Must call for each new cond.
            redo = True
            while redo:
                while self.gui.mode in ('wait', 'request', 'process'):
                    if self.gui.mode in ('wait', 'process'):
                        time.sleep(0.01)
                        continue
                    # 'request' mode.
                    if self.gui.request == 'reset':
                        try:
                            self.agent.reset(cond)
                        except NotImplementedError:
                            self.gui.err_msg = 'Agent reset unimplemented.'
                    elif self.gui.request == 'fail':
                        self.gui.err_msg = 'Cannot fail before sampling.'
                    self.gui.process_mode()  # Complete request.
                if reset: # If we are doing a reset one
                    self.gui.set_status_text(
                        'Sampling reset: iteration %d, condition %d, sample %d.' %
                        (itr, cond, i)
                    )
                else: # Otherwise this is a normal sample or something
                    self.gui.set_status_text(
                        'Sampling: iteration %d, condition %d, sample %d.' %
                        (itr, cond, i)
                )
                if reset:
                    self.agent.reset_time = True # Set the agent reset_time to true lmao
                    # Then it will be a special sample hahaha
                self.agent.sample(pol, cond, verbose=(i < self._hyperparams['verbose_trials']))

                if self.gui.mode == 'request' and self.gui.request == 'fail':
                    redo = True
                    self.gui.process_mode()
                    self.agent.delete_last_sample(cond)
                else:
                    redo = False
        else:
            # If reset is true something I dunno.... T_T
            # if reset:
            #     self.agent.sample(
            #     pol, cond,
            #     verbose=(i < self._hyperparams['verbose_trials']), reset=True
            #     )
            # else:
            #     self.agent.sample(
            #         pol, cond,
            #         verbose=(i < self._hyperparams['verbose_trials'])
            #     )
            verbose = i < self._hyperparams['verbose_trials']
            self.agent.sample(pol, cond, verbose=verbose, reset=reset)

    def _take_iteration(self, itr, sample_lists, reset=False):
        """
        Take an iteration of the algorithm.
        Args:
            itr: Iteration number.
        Returns: None
        """
        if self.gui:
            self.gui.set_status_text('Calculating.')
            self.gui.start_display_calculating()
        self.agent.reset(0) # so the arm doesn't roll

        if reset: # If we are resetting, iterate for reset algorithm (??)
            pass
            # Actually I don't really want to learn
            #self.reset_algorithm.iteration(sample_lists)
        else: # Otherwise, iterate for normal algorithm
            self.algorithm.iteration(sample_lists)
        if self.gui:
            self.gui.stop_display_calculating()

    def _take_policy_samples(self, N=None):
        """
        Take samples from the policy to see how it's doing.
        Args:
            N  : number of policy samples to take per condition
        Returns: None
        """
        if 'verbose_policy_trials' not in self._hyperparams:
            # AlgorithmTrajOpt
            return None
        verbose = self._hyperparams['verbose_policy_trials']
        if self.gui:
            self.gui.set_status_text('Taking policy samples.')
        pol_samples = [[None] for _ in range(len(self._test_idx))]
        # Since this isn't noisy, just take one sample.
        # TODO: Make this noisy? Add hyperparam?
        # TODO: Take at all conditions for GUI?
        # CHANGED TO MAKE NOISY -- DOES THAT CHANGE ANYTHING??! NOPE
        for cond in range(len(self._test_idx)):
            pol_samples[cond][0] = self.agent.sample(
                self.algorithm.policy_opt.policy, self._test_idx[cond],
                verbose=verbose, save=False, noisy=True)
        return [SampleList(samples) for samples in pol_samples]

    # This is basically the same thing as the take policy samples method
    # but with the reset algorithm instead lmao
    def _take_reset_policy_samples(self, N=None):
        """
        Take samples from the policy to see how it's doing.
        Args:
            N  : number of policy samples to take per condition
        Returns: None
        """
        if 'verbose_policy_trials' not in self._hyperparams:
            # AlgorithmTrajOpt
            return None
        verbose = self._hyperparams['verbose_policy_trials']
        if self.gui:
            self.gui.set_status_text('Taking reset policy samples.')
        pol_samples = [[None] for _ in range(len(self._test_idx))]
        # Since this isn't noisy, just take one sample.
        # TODO: Make this noisy? Add hyperparam?
        # TODO: Take at all conditions for GUI?
        for cond in range(len(self._test_idx)):
            pol_samples[cond][0] = self.agent.sample(
                self.reset_algorithm.policy_opt.policy, self._test_idx[cond],
                verbose=verbose, save=False, noisy=False)
        return [SampleList(samples) for samples in pol_samples]

    def _log_data(self, itr, traj_sample_lists, pol_sample_lists=None):
        """
        Log data and algorithm, and update the GUI.
        Args:
            itr: Iteration number.
            traj_sample_lists: trajectory samples as SampleList object
            pol_sample_lists: policy samples as SampleList object
        Returns: None
        """
        if self.gui:
            self.gui.set_status_text('Logging data and updating GUI.')
            self.gui.update(itr, self.algorithm, self.agent,
                traj_sample_lists, pol_sample_lists)
            self.gui.save_figure(
                self._data_files_dir + ('figure_itr_%02d.png' % itr)
            )
        if 'no_sample_logging' in self._hyperparams['common']:
            return
        self.data_logger.pickle(
            self._data_files_dir + ('algorithm_itr_%02d.pkl' % itr),
            copy.copy(self.algorithm)
        )
        self.data_logger.pickle(
            self._data_files_dir + ('traj_sample_itr_%02d.pkl' % itr),
            copy.copy(traj_sample_lists)
        )
        # Maybe pickle the agent to help out?
        self.data_logger.pickle(
            self._data_files_dir + ('agent_itr_%02d.pkl' % itr),
            copy.copy(self.agent)
        )
        if pol_sample_lists:
            self.data_logger.pickle(
                self._data_files_dir + ('pol_sample_itr_%02d.pkl' % itr),
                copy.copy(pol_sample_lists)
            )

    def _end(self):
        """ Finish running and exit. """
        if self.gui:
            self.gui.set_status_text('Training complete.')
            self.gui.end_mode()
            if self._quit_on_end:
                # Quit automatically (for running sequential expts)
                os._exit(1)

def unpickle_agent(pathway, itr):
    pickle_filename = pathway + 'agent_itr_' + str(itr) + '.pkl'
    with open(pickle_filename, 'r') as f: # Read to this pickled place
        gear_thing = pickle.load(f) # Read the pickled thing
        return gear_thing

def main():
    """ Main function to be run. """
    parser = argparse.ArgumentParser(description='Run the Guided Policy Search algorithm.')
    parser.add_argument('experiment', type=str,
                        help='experiment name')
    parser.add_argument('-n', '--new', action='store_true',
                        help='create new experiment')
    parser.add_argument('-t', '--targetsetup', action='store_true',
                        help='run target setup')
    parser.add_argument('-r', '--resume', metavar='N', type=int,
                        help='resume training from iter N')
    parser.add_argument('-p', '--policy', metavar='N', type=int,
                        help='take N policy samples (for BADMM/MDGPS only)')
    parser.add_argument('-s', '--silent', action='store_true',
                        help='silent debug print outs')
    parser.add_argument('-q', '--quit', action='store_true',
                        help='quit GUI automatically when finished')
    args = parser.parse_args()

    exp_name = args.experiment
    resume_training_itr = args.resume
    test_policy_N = args.policy

    from gps import __file__ as gps_filepath
    gps_filepath = os.path.abspath(gps_filepath)
    gps_dir = '/'.join(str.split(gps_filepath, '/')[:-3]) + '/'
    exp_dir = gps_dir + 'experiments/' + exp_name + '/'
    hyperparams_file = exp_dir + 'hyperparams.py'
    if args.silent:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    else:
        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

    if args.new:
        from shutil import copy

        if os.path.exists(exp_dir):
            sys.exit("Experiment '%s' already exists.\nPlease remove '%s'." %
                     (exp_name, exp_dir))
        os.makedirs(exp_dir)

        prev_exp_file = '.previous_experiment'
        prev_exp_dir = None
        try:
            with open(prev_exp_file, 'r') as f:
                prev_exp_dir = f.readline()
            copy(prev_exp_dir + 'hyperparams.py', exp_dir)
            if os.path.exists(prev_exp_dir + 'targets.npz'):
                copy(prev_exp_dir + 'targets.npz', exp_dir)
        except IOError as e:
            with open(hyperparams_file, 'w') as f:
                f.write('# To get started, copy over hyperparams from another experiment.\n' +
                        '# Visit rll.berkeley.edu/gps/hyperparams.html for documentation.')
        with open(prev_exp_file, 'w') as f:
            f.write(exp_dir)

        exit_msg = ("Experiment '%s' created.\nhyperparams file: '%s'" %
                    (exp_name, hyperparams_file))
        if prev_exp_dir and os.path.exists(prev_exp_dir):
            exit_msg += "\ncopied from     : '%shyperparams.py'" % prev_exp_dir
        sys.exit(exit_msg)

    if not os.path.exists(hyperparams_file):
        sys.exit("Experiment '%s' does not exist.\nDid you create '%s'?" %
                 (exp_name, hyperparams_file))

    hyperparams = imp.load_source('hyperparams', hyperparams_file)
    if args.targetsetup:
        try:
            import matplotlib.pyplot as plt
            from gps.agent.ros.agent_ros import AgentROS
            from gps.gui.target_setup_gui import TargetSetupGUI

            # If we want to start halfway and stuff
            if resume_training_itr is not None:
                hyperparams.agent.update({'resume_itr': resume_training_itr})
            agent = AgentROS(hyperparams.config['agent'])
            TargetSetupGUI(hyperparams.config['common'], agent)

            plt.ioff()
            plt.show()
        except ImportError:
            sys.exit('ROS required for target setup.')
    elif test_policy_N:
        import random
        import numpy as np
        import matplotlib.pyplot as plt

        seed = hyperparams.config.get('random_seed', 0)
        random.seed(seed)
        np.random.seed(seed)

        data_files_dir = exp_dir + 'data_files/'
        data_filenames = os.listdir(data_files_dir)
        algorithm_prefix = 'algorithm_itr_'
        algorithm_filenames = [f for f in data_filenames if f.startswith(algorithm_prefix)]
        current_algorithm = sorted(algorithm_filenames, reverse=True)[0]
        current_itr = int(current_algorithm[len(algorithm_prefix):len(algorithm_prefix)+2])

        gps = GPSMain(hyperparams.config)
        if hyperparams.config['gui_on']:
            test_policy = threading.Thread(
                target=lambda: gps.test_policy(itr=current_itr, N=test_policy_N)
            )
            test_policy.daemon = True
            test_policy.start()

            plt.ioff()
            plt.show()
        else:
            gps.test_policy(itr=current_itr, N=test_policy_N)
    else:
        import random
        import numpy as np
        import matplotlib.pyplot as plt
        seed = hyperparams.config.get('random_seed', 0)
        random.seed(seed)
        np.random.seed(seed)

        # If we want to start halfway and stuff
        if resume_training_itr is not None:
            hyperparams.agent.update({'resume_itr': resume_training_itr})

        gps = GPSMain(hyperparams.config, args.quit)

        if hyperparams.config['gui_on']:
            run_gps = threading.Thread(
                target=lambda: gps.run(itr_load=resume_training_itr)
            )
            run_gps.daemon = True
            run_gps.start()

            plt.ioff()
            plt.show()
        else:
            gps.run(itr_load=resume_training_itr)


if __name__ == "__main__":
    main()
