""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path as osp

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.agent.ros.cad.j_piece_experiment import JPieceExperiment
from gps.agent.ros.cad.gear_experiment import GearExperiment

from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.algorithm_traj_opt_pilqr import AlgorithmTrajOptPILQR
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.algorithm_mdgps_pilqr import AlgorithmMDGPSPILQR
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
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info


#NNLIB = 'caffe'
NNLIB = None
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, REF_TRAJ, REF_OFFSETS
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info

T = 200
NNLIB = 'tf'
PILQR = False
CONDITIONS = 4
assert NNLIB in ('tf', 'caffe', None)

# EE_POINTS = np.array([[0.02, -0.025, 0.05], [0.02, -0.025, -0.05],
#                       [0.02, 0.05, 0.0]])
EE_POINTS = np.array([[0.0, 0.0, 0.0], [0.15, 0.05, 0.0], [0.15, -0.05, 0.0]])

SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 7,
    REF_TRAJ: 9*T
}

PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

# BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
# EXP_DIR = BASE_DIR + '/../experiments/pr2_cad/'
EXP_DIR = '/home/gwthomas/workspace/gps/experiments/pr2_cad'

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'conditions': 2,
    'iterations': 50,
    'data_files_dir': osp.join(EXP_DIR, 'data_files/'),
    'target_filename': osp.join(EXP_DIR, 'target.npz'),
    'log_filename': osp.join(EXP_DIR, 'log.txt'),
    'conditions': CONDITIONS,
    'train_conditions': range(CONDITIONS - 1),
    'test_conditions': range(CONDITIONS),
    'iterations': 25,
}

train_joint_positions = np.array([
    [0.4, -0.25, 1.0, -0.5, 0.5, -0.5, 1.25],
    [-0.2, 0.0, 1.0, -0.75, 0.0, -0.6, 1.25]
])
test_joint_positions = np.load(osp.join(EXP_DIR, 'test_positions.npy'))
all_joint_positions = np.vstack([train_joint_positions, test_joint_positions])

x0s = []
ee_tgts = []
reset_conditions = []
for i in xrange(common['conditions']):
    x0s.append(np.zeros(32))
    ee_tgts.append(np.zeros(9))
    reset_condition = {
        TRIAL_ARM:     {'data': all_joint_positions[i], 'mode': 1},
        AUXILIARY_ARM: {'data': np.array([-1.25, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]), 'mode': 1}
    }
    reset_conditions.append(reset_condition)

if not osp.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentCADExperiment,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': T,
    'T_interpolation': int(0.75*T),
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES],
    #'planner': 'RRTStarkConfigDefault',
    'planner': 'RRTConnectkConfigDefault',
    # 'planner': 'PRMstarkConfigDefault',
    'planning_time': 15,
    'plan_attempts': 20,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, \
            END_EFFECTOR_POINT_VELOCITIES, REF_TRAJ],
    'targets': [{'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
        for _ in range(common['conditions'])],
    ###### THIS IS FOR THE ORIGINAL EXPERIMENT #######################
    #'targets': [{'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
    #    for _ in range(common['conditions'])],
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
    'cad_path': os.path.join(EXP_DIR, 'piece.stl'),
    'j_piece': os.path.join(EXP_DIR, 'j_piece.stl'),
    'j_box': os.path.join(EXP_DIR, 'j_box.stl'),
    'the_gear': os.path.join(EXP_DIR, 'gear_teeth.stl'),
    'reset_timeout': 15,
    'trial_timeout': 20,
    'exp_dir': EXP_DIR,
    'cad_path': osp.join(EXP_DIR, 'piece.stl')
}

if NNLIB is None:
    if PILQR:
        algorithm = {
            'type': AlgorithmTrajOptPILQR,
            'conditions': common['conditions'],
            'iterations': common['iterations'],
            'step_rule': 'res_percent',
            'step_rule_res_ratio_dec': 0.2,
            'step_rule_res_ratio_inc': 0.05
        }
    else:
        algorithm = {
            'type': AlgorithmTrajOpt,
            'conditions': common['conditions'],
            'iterations': common['iterations']
        }
else:
    if PILQR:
        algorithm = {
            'type': AlgorithmMDGPSPILQR,
            'step_rule': 'const',
            'conditions': common['conditions'],
            'iterations': common['iterations'],
            'policy_sample_mode': 'replace',
            'sample_on_policy': True,
            # 'kl_step': np.linspace(0.4, 0.2, T),
            # 'max_step_mult': np.linspace(10.0, 5.0, T),
            # 'min_step_mult': np.linspace(0.01, 0.5, T),
            # 'max_mult': np.linspace(5.0, 2.0, T),
            # 'min_mult': np.linspace(0.1, 0.5, T)
        }
    else:
        algorithm = {
            'type': AlgorithmBADMM,
            'conditions': common['conditions'],
            'train_conditions': common['train_conditions'],
            'test_conditions': common['test_conditions'],
            'iterations': common['iterations'],
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

algorithm['init_traj_distr'] = {
    'type': init_pd,
    'pos_gains': 10.0,
    'vel_gains_mult': 0.1,
    'init_var': 0.1,
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

if PILQR:
    algorithm['traj_opt'] = {
        'type': TrajOptPILQR,
        'covariance_damping': 10.0,
        'kl_threshold': 0.5,
    }
else:
    algorithm['traj_opt'] = {
        'type': TrajOptLQRPython,
    }

if NNLIB == 'tf':
    algorithm['policy_opt'] = {
        'type': PolicyOptTf,
        'network_params': {
            'obs_include': agent['obs_include'],
            'obs_image_data': [END_EFFECTOR_POINTS, REF_TRAJ],
            'sensor_dims': SENSOR_DIMS,
            'hidden_attention': [100,100],
            'mlp_hidden_sizes': [100,100],
            'resnet_n_hidden': 2,
            'T': T,
            'ee_pos_indices': (14,23),
            'fixed_scale': 100,
            'resnet': True
        },
        'network_model': fixed_distance_network,
        'max_iterations': 25000,
        'period': 500,
        'termination_history_length': 5,
        'termination_epsilon': 0.01,
        'weights_file_prefix': osp.join(EXP_DIR, 'policy'),
    }
elif NNLIB == 'caffe':
    algorithm['policy_opt'] = {
        'type': PolicyOptCaffe,
        'weights_file_prefix': osp.join(EXP_DIR, 'policy'),
        'iterations': 3000,
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
    'weights': [0.1, 5.0, 1.0],
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

    'num_samples': 2, # must be >1 to fit dynamics
}
if NNLIB is not None:
    config['verbose_policy_trials'] = 1

common['info'] = generate_experiment_info(config)
