""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import numpy as np
import os.path as osp
import cPickle as pickle

from gps import __file__ as gps_filepath
from gps.agent.ros.cad.util import ConditionInfo
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.agent.ros.cad.j_piece_experiment import JPieceExperiment
from gps.agent.ros.cad.gear_experiment import GearExperiment
from gps.agent.ros.cad.real_agent_cad_experiment import RealAgentCADExperiment
from gps.agent.ros.cad.real_gear_experiment import RealGearExperiment

from gps.algorithm.algorithm_traj_opt_smart import AlgorithmTrajOptSmart
from gps.algorithm.algorithm_traj_opt_pilqr import AlgorithmTrajOptPILQR
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.algorithm_mdgps import AlgorithmMDGPS
from gps.algorithm.algorithm_mdgps_pilqr import AlgorithmMDGPSPILQR
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.policy_opt.policy_opt_tf import PolicyOptTf
from gps.algorithm.policy_opt.policy_opt_caffe import PolicyOptCaffe
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM
from gps.algorithm.policy_opt.tf_model_example import tf_network
from gps.agent.ros.cad.ref_traj_network import *

from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_fkt import CostFKT
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY, RAMP_CONSTANT
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.traj_opt.traj_opt_pilqr import TrajOptPILQR
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.gui.target_setup_gui import load_pose_from_npz
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, REF_TRAJ, REF_OFFSETS, TIMESTEP

from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info


#T = 270
T = 200
#NN = False
#ALL_CONDITIONS = 1
#TRAIN = True
ALL_CONDITIONS = 1
TRAIN_CONDITIONS = 1
#MODE = 'GPS'
#MODE = 'iLQR subset'
MODE = 'GPS'
assert MODE in ('GPS', 'iLQR subset', 'NN test')


EE_POINTS = np.array([[0.0, 0.0, 0.0], [0.15, 0.05, 0.0], [0.15, -0.05, 0.0]])
PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])
#PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.456, 0.098])

SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 7,
    REF_TRAJ: 9*T,
    TIMESTEP: 1
}

EXP_DIR = osp.dirname(osp.realpath(__file__))

################################################################################
conditions, condition_info = [], []
if MODE == 'iLQR subset':
    # determine conditions to train on based on current status
    for cond in range(ALL_CONDITIONS):
        filename = osp.join(EXP_DIR, 'condition_info', 'info%02d.pkl' % cond)
        if osp.isfile(filename):
            with open(filename, 'rb') as f:
                info = pickle.load(f)
            if info.good:
                print 'Condition {} is good, skipping'.format(cond)
                continue
        else:
            print 'ERROR: No data for condition', cond
            exit()
        conditions.append(cond)
        condition_info.append(info)
        if len(conditions) == TRAIN_CONDITIONS:
            print "That's all folks"
            break
else:
    for cond in range(ALL_CONDITIONS):
        filename = osp.join(EXP_DIR, 'condition_info', 'info%02d.pkl' % cond)
        assert osp.isfile(filename)
        with open(filename, 'rb') as f:
            info = pickle.load(f)
        conditions.append(cond)
        condition_info.append(info)
################################################################################

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'iterations': 40,
    'data_files_dir': osp.join(EXP_DIR, 'data_files/'),
    'target_filename': osp.join(EXP_DIR, 'target.npz'),
    'log_filename': osp.join(EXP_DIR, 'log.txt'),
    'conditions': ALL_CONDITIONS,
    'train_conditions': range(TRAIN_CONDITIONS),
    'test_conditions': range(ALL_CONDITIONS),
}

x0s = []
ee_tgts = []
reset_conditions = []
condition_count = 0
# Just added one on top of the ones already there
manual_conds = [
[0.1638472530716777, 0.03519148801145314, 1.1599672616303516, -0.9974434082707643, 1.749642991906653, 
-1.9151435630987073, 1.0815418108712371],


[0.8475781264147393, 0.041536107244287246, 1.2431914165163225, -0.8929189546518572, 1.7288758876914019,
 -1.7518545859649293, 3.12685263583307],
[0.39822426033987646, -0.017511149082622118, 1.0637543658084199, 
            -0.6000478055229806, 1.9631565898411718, -1.927884425201205, 0.5613210093298342],
[-0.13105158486712065, -0.05287182694028418, 1.0186946595984816, -0.6175650616003875, 1.9622310364499629, 
-1.9276541904990117, 0.5613520871299047],
[0.5587314715430008, -0.07799651910230722, 1.1201992646906196, -1.2826417096963139, 1.5511117894923292,
 -1.9958326682950758, 0.07287744117175876]
,
[0.37733179276591605, -0.060062395404162826, 1.1362347473276082, -1.3030543800013914, 1.456936731936817, 
-1.9990523283824455, 0.0035242225330049948],
[0.38686605376196936, -0.13340619373572501, 1.0581419468854736, -0.8125712098339994, 1.7395775987772555, 
-2.0039688363537, -0.19352767659805803],
 [0.5587314715430008, -0.07799651910230722, 1.1201992646906196, -1.2826417096963139, 1.5511117894923292,
 -1.9958326682950758, 0.07287744117175876],
 [0.5587314715430008, -0.07799651910230722, 1.1201992646906196, -1.2826417096963139, 1.5511117894923292,
 -1.9958326682950758, 0.07287744117175876]]
'''
manual_conds = [[0.39822426033987646, -0.017511149082622118, 1.0637543658084199, 
            -0.6000478055229806, 1.9631565898411718, -1.927884425201205, 0.5613210093298342], 
            [0.9606627524896679, 0.01971061708333794, 1.030721271576223, 
            -0.7242610758900474, 1.8885916947618995, -1.9987169471233446, 0.12997357546245514],
            [1.1496898400635955, -0.009474631387698922, 0.8965042819046285, 
            -1.424227465429404, 0.8387827607831273, -1.998890982803743, 0.13006059330265463],
            [0.28654904676001663, -0.03789852555079569, 0.9200764413810018,
         -1.2131517682322206, 0.9956062135060968, -2.0045471424166905, -0.6143770296014055],
         [-0.18054683542924121, -0.03231526062590168, 0.8636315424988017, 
            -1.1579941271950966, 0.8340392996531814, -2.0045471424166905, -0.6143770296014055],
            [0.36017012297302, -0.34167889441889243, 0.8586605428813353, 
            -0.8290750709317215, 1.910862823237865, -2.0045471424166905, -0.6143770296014055],
            [0.7567953804088399, 0.25166989623575264, 0.9505438583912801, 
            -1.105442358962876, 2.055769776049019, -1.9951927245952774, -0.30367983117020225],
            [1.234503309619792, 0.4323646519868678, 1.10560697549096, 
            -1.2368941648991192, 2.3535665796705048, -0.45663029203342065, -0.4389925726799411],
            [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
            [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805],
                        [0.08666119231232383, 0.3757706484299877, 0.5905472731908856, 
            -1.5802902923008468, 1.1681640738796188, -0.793389333604281, 0.25523575642918805]

            ]
'''
#for info in condition_info:
for i in range(ALL_CONDITIONS):
    x0s.append(np.zeros(32))
    ee_tgts.append(np.zeros(9))
    reset_condition = {
        # POSITIONS FOR THE U SHAPE EXPERIMENT
        #TRIAL_ARM:      {'data': np.array([1.3250373357736205, 0.2747643102432688, 1.0486810121296506, 
        #    -1.1847767088010492, 0.610402461502314, -1.3894726951020813, 7.376906325091097]), 'mode': 1},

        #TRIAL_ARM:      {'data': np.array([0.6039984846199152, -0.06479971109801232, 0.8787048961775711,
        # -1.394839011018548, 0.6514160461502616, -0.9453336387254235, 3.817093500382482]), 'mode': 1},

        #TRIAL_ARM:      {'data': np.array([0.4324646933083116, 0.09220846618385553, 0.921038570339221, 
        #    -1.304067774981076, 0.2866901629269777, -1.3876018115378002, -2.124789190858826]), 'mode': 1},

        #TRIAL_ARM:      {'data': np.array([0.4324646933083116, 0.09144711187591541, 0.92248176377655,
        # -1.3052259406721443, 0.2911443886221706, -1.3912130519060666, -2.1243976105779296]), 'mode': 1},

        #TRIAL_ARM:      {'data': np.array([0.3861198942057566, -0.22908305176686322, 1.1936417751680277, 
        #    -0.8355897529439803, 1.5666148087950793, -1.9351615709911956, -3.6717488449999047]), 'mode': 1},
        # FOR THE PEG INSERTION
        #TRIAL_ARM:      {'data': np.array([0.41745859556669723, -0.26190588193139164, 1.1017584596580827, 
        #    -1.0237916777425664, -4.798994333418499 + 6.283185, -2.0049532256709526, 2.54619794427333]), 'mode': 1},
        # DOWN LOW FOR PEG INSERTION
        #TRIAL_ARM:      {'data': np.array([0.0561515571249529, 0.12756914404151762, 1.1925192913834384,
        # -0.9925212040837244, 1.7710464140783602, -1.8218241809717601, 2.2656089185510457]), 'mode': 1},
        #TRIAL_ARM:      {'data': np.array([0.8276805382490626, 0.22451492591922267, 1.13398977975843, 
        #    -1.2578859180497306, 1.846768250896644, -1.9356000070321997, 2.763568509090904]), 'mode': 1},

        # ALL POSITIONS FOR PEG INSERTION INTERESTING AHAHAHAH
        TRIAL_ARM:      {'data': np.array(manual_conds[i]), 'mode': 1},
        #TRIAL_ARM:     {'data': info.initial, 'mode': 1},
        AUXILIARY_ARM: {'data': np.array([-1.25, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]), 'mode': 1}
    }
    reset_conditions.append(reset_condition)
    condition_count += 1 # Increment by one

if not osp.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    #'type': AgentCADExperiment,
    #'type': RealAgentCADExperiment,
    #'type': JPieceExperiment,
    #'type': GearExperiment,
    #'type': FullGearExperiment,
    'type': RealGearExperiment,
    'dt': 0.05,
    'conditions': common['conditions'],
    'actual_conditions': conditions,
    'condition_info': condition_info,
    'T': T,
    #'T_interpolation': 230,
    'T_interpolation': 150,
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES, REF_TRAJ, TIMESTEP],
    #'planner': 'RRTStarkConfigDefault',
    'planner': 'RRTConnectkConfigDefault',
    # 'planner': 'PRMstarkConfigDefault',
    'planning_time': 30,
    'plan_attempts': 5,
    #'targets': [{'position': (0.5, 0.09, 0.55), 'orientation': (3.14, 0.0, -1.57)} for _ in conditions],

    ###### THIS IS FOR THE ORIGINAL EXPERIMENT #######################
    'targets': [{'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
        for _ in range(common['conditions'])],
    ##################################################################

    ###### THIS IS FOR THE J PIECE EXPERIMENT ########################
    #'targets': [{'position': (0.7, -0.11, 0.85), 'orientation': (1.57, 1.57, -1.57)}
    #    for _ in range(common['conditions'])],
    ##################################################################
    #0.657
    ###### THIS IS FOR THE GEAR EXPERIMENT ###########################
    #'targets': [{'position': (0.658, -0.074, 0.7), 'orientation': (1.57, 0, 0.07)}
    #    for _ in range(common['conditions'])],
    ###################################################################
    'cad_path': osp.join(EXP_DIR, 'piece.stl'),
    'j_piece': osp.join(EXP_DIR, 'j_piece.stl'),
    'j_box': osp.join(EXP_DIR, 'j_box.stl'),
    'the_gear': osp.join(EXP_DIR, 'gear_teeth.stl'),
    'compound_gear': osp.join(EXP_DIR, 'compound_gear.stl'),
    'base_plate': osp.join(EXP_DIR, 'base_plate.stl'),
    'shaft2': osp.join(EXP_DIR, 'shaft2.stl'),
    'use_AR_markers': True,
    'reset_timeout': 20,
    'trial_timeout': 30,
    'exp_dir': EXP_DIR,
}

algorithm_ilqr = {
    'type': AlgorithmTrajOptSmart,
    'iterations': common['iterations'],
    'conditions': agent['conditions'],
    'train_conditions': conditions,
    'actual_conditions': conditions,
    'condition_info': condition_info
}
reset_algorithm = {
    'type': AlgorithmTrajOpt,
    'iterations': common['iterations'],
    'conditions': agent['conditions'],
    'train_conditions': conditions,
    'actual_conditions': conditions,
    'condition_info': condition_info
}

algorithm_gps = {
    'type': AlgorithmBADMM,
    'iterations': common['iterations'],
    'conditions': agent['conditions'],
    'train_conditions': common['train_conditions'],
    'test_conditions': common['test_conditions'],
    'condition_info': condition_info,
    'lg_step_schedule': np.array([1e-4, 1e-3, 1e-2, 1e-1]),
    'policy_dual_rate': 0.1,
    'ent_reg_schedule': np.array([1e-3, 1e-3, 1e-2, 1e-1]),
    'fixed_lg_step': 3,
    'kl_step': 5.0,
    'init_pol_wt': 0.01,
    'min_step_mult': 0.01,
    'max_step_mult': 10.0,
    'sample_decrease_var': 0.05,
    'sample_increase_var': 0.1,
    'exp_step_increase': 2.0,
    'exp_step_decrease': 0.5,
    'exp_step_upper': 0.5,
    'exp_step_lower': 1.0,
    'max_policy_samples': 6,
    'policy_sample_mode': 'add',
}
algorithm = algorithm_gps if MODE == 'GPS' else algorithm_ilqr

algorithm['init_traj_distr'] = {
    'type': init_pd,
    'pos_gains': 50.0,
    'vel_gains_mult': 0.18,
    'init_var': 0.15,
    #'init_var': 0.01,

    'dQ': 7, # set this to action dim based on another file, but effect of changing unclear
    'dt': agent['dt'],
    'T': agent['T'],
}
agent['init_traj_distr'] = algorithm['init_traj_distr']

algorithm['dynamics'] = {
    'type': DynamicsLRPrior,
    'regularization': 1e-6,
    'prior': {
        'type': DynamicsPriorGMM,
        'max_clusters': 20,
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    }
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {
    'type': PolicyOptTf,
    'network_params': {
        'obs_include': agent['obs_include'],
        'obs_image_data': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, REF_TRAJ, TIMESTEP],
        'sensor_dims': SENSOR_DIMS,
        'hidden_attention': [100,100],
        'mlp_hidden_sizes': [100, 100],
        'resnet_n_hidden': 0,
        'T': T,
        'ee_pos_indices': (14,23),
        'ee_vel_indices': (23,32),
        'temperature': 0.1,
        'attention': None,
        'structure': mlp_structure,
        'regularization': 10.0,
        'state_dependent': False,
        'time_k': 10
    },
    'network_model': ref_traj_network_factory,
    # 'lr': 1e-4,
    'batch_size': 64,
    'max_iterations': 50000,
    # CHANGE MAX_ITERATIONS # Add some noise maybe
    'period': 500,
    'termination_history_length': 5,
    'termination_epsilon': 0.005,
    'weights_file_prefix': osp.join(EXP_DIR, 'policy'),
    'normalize': False
}

algorithm['policy_prior'] = {
    'type': PolicyPriorGMM,
    'max_clusters': 20,
    'min_samples_per_cluster': 40,
    'max_samples': 40,
}


# COSTS
torque_cost = {
    'type': CostAction,
    'wu': 5e-3 / PR2_GAINS,
}

fk_cost1 = {
    'type': CostFKT,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal is 0.
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 0.1,
    'l2': 0.0001,
    'ramp_option': RAMP_CONSTANT,
}

fk_cost2 = {
    'type': CostFK,
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 1.0,
    'l2': 0.0,
    'wp_final_multiplier': 1.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [torque_cost, fk_cost1, fk_cost2],
    'weights': [0.2, 1.0, 0.1],
    # 'costs': [torque_cost, fk_cost1],
    # 'weights': [0.1, 1.0],
}


config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 1,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
    'reset_algorithm': reset_algorithm,
    'num_samples': 5, # must be >1 to fit dynamics
}
if MODE == 'GPS':
    config['verbose_policy_trials'] = 1

common['info'] = generate_experiment_info(config)
