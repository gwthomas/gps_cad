""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
<<<<<<< HEAD
from gps.agent.ros.cad.j_piece_experiment import JPieceExperiment
from gps.agent.ros.cad.gear_experiment import GearExperiment
=======
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec

from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.policy_opt.policy_opt_tf import PolicyOptTf
from gps.algorithm.policy_opt.policy_opt_caffe import PolicyOptCaffe
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM
from gps.algorithm.policy_opt.tf_model_example import tf_network
<<<<<<< HEAD
=======
from gps.agent.ros.cad.ref_traj_network import ref_traj_network
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec

from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_fkt import CostFKT
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY, RAMP_CONSTANT
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.gui.target_setup_gui import load_pose_from_npz
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
<<<<<<< HEAD
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info


#NNLIB = 'caffe'
NNLIB = None
=======
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, REF_TRAJ, REF_OFFSETS
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info

T = 150
NNLIB = 'tf'
ATTENTION = False
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
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
<<<<<<< HEAD
=======
    REF_TRAJ: 9*T
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}

PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/pr2_cad/'

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
<<<<<<< HEAD
    'conditions': 1,
=======
    'conditions': 2,
    'iterations': 50,
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}

x0s = []
ee_tgts = []
reset_conditions = []
for i in xrange(common['conditions']):
    x0s.append(np.zeros(32))
    ee_tgts.append(np.zeros(9))
    if i == 0:
<<<<<<< HEAD
        reset_condition ={
=======
        reset_condition = {
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
            TRIAL_ARM:     {'data': np.array([0.4, -0.25, 1.0, -0.5, 0.5, -0.5, 1.25]), 'mode': 1},
            # TRIAL_ARM:     {'data': np.array([-0.2, -0.0, 1.0, -0.75, -0.0, -0.6, 1.25]), 'mode': 1},
            AUXILIARY_ARM: {'data': np.array([-1.25, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]), 'mode': 1}
        }
    elif i == 1:
        reset_condition = {
            # TRIAL_ARM:     {'data': np.array([0.4, -0.25, 1.0, -0.5, 0.5, -0.5, 1.25]), 'mode': 1},
            TRIAL_ARM:     {'data': np.array([-0.2, -0.0, 1.0, -0.75, -0.0, -0.6, 1.25]), 'mode': 1},
            AUXILIARY_ARM: {'data': np.array([-1.25, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]), 'mode': 1}
        }
    reset_conditions.append(reset_condition)

# reset_conditions = [
#         {
#             TRIAL_ARM:     {'data': np.array([0.92857393, 0.43166243, 1.93682597,-2.02126038,-31.3224097,-0.09676463, 20.9937161]), 'mode': 1},
#             AUXILIARY_ARM: {'data': np.array([0.23164246, 0.23562515,-0.07458033,-0.14659051, 0.24475279,-0.09030182,-0.07800095]), 'mode': 1}
#         },
#         {
#             TRIAL_ARM:     {'data': np.array([0.92840811, 0.43107034, 1.93709342,-2.02039176,-31.3229882,-0.09411059, 20.9937596]), 'mode': 1},
#             AUXILIARY_ARM: {'data': np.array([0.21738252, 0.57079023,-0.05549811,-0.68021535, 0.33956417,-0.09004077,-0.48472234]), 'mode': 1}
#         }
# ]

if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
<<<<<<< HEAD
    #'type': AgentCADExperiment,
    #'type': JPieceExperiment,
    'type': GearExperiment,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': 230,
    'T_interpolation': 160,
=======
    'type': AgentCADExperiment,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': T,
    'T_interpolation': 100,
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
<<<<<<< HEAD
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES],
    #'planner': 'RRTStarkConfigDefault',
    'planner': 'RRTConnectkConfigDefault',
    'planning_schedule': [20],
    'indefatigable': False,
    'require_approval': True,

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
    'targets': [{'position': (0.658, -0.074, 0.7), 'orientation': (1.57, 0, 0.07)}
        for _ in range(common['conditions'])],
    ###################################################################

=======
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES] \
            + ([REF_TRAJ] if ATTENTION else []),
    'planner': 'RRTStarkConfigDefault',
    'planning_schedule': [5],
    'indefatigable': True,
    'require_approval': True,
    'targets': [{'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
        for _ in range(common['conditions'])],
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
    # 'targets': [
    #     # {'position': (0.55, 0.07, 0.55), 'orientation': (0.0, 0.0, -1.57)}
    #     {'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
    #
    #     #PREVIOUSLY:
    #     #{'position': (0.5, 0.09, 0.555), 'orientation': (0.0, 0.0, -1.57)}
    # ],
<<<<<<< HEAD
    'cad_path': os.path.join(EXP_DIR, 'piece.stl'), 
    'j_piece': os.path.join(EXP_DIR, 'j_piece.stl'),
    'j_box': os.path.join(EXP_DIR, 'j_box.stl'),
    'the_gear': os.path.join(EXP_DIR, 'gear_teeth.stl'),
    'reset_timeout': 15,
    'trial_timeout': 30
=======
    'cad_path': os.path.join(EXP_DIR, 'piece.stl'),
    'reset_timeout': 10,
    'attention': ATTENTION
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}

if NNLIB is None:
    algorithm = {
        'type': AlgorithmTrajOpt,
        'conditions': common['conditions'],
        'iterations': 50,
    }
else:
    algorithm = {
        'type': AlgorithmBADMM,
        'conditions': common['conditions'],
        'iterations': 50,
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
<<<<<<< HEAD
    #'pos_gains': 9,
    'pos_gains': 12,
    'vel_gains_mult': 0.05,
    'init_var': 0.2,
=======
    'pos_gains': 7.5,
    'vel_gains_mult': 0.0,
    'init_var': 0.1,
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
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
<<<<<<< HEAD
        'min_samples_per_cluster': 40, #added from hyperparams_lqr.py
        'max_samples': 20, #added from hyperparams_lqr.py
        }
=======
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    }
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

if NNLIB == 'tf':
    algorithm['policy_opt'] = {
        'type': PolicyOptTf,
        'network_params': {
<<<<<<< HEAD
            'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES],
            'obs_vector_data': [JOINT_ANGLES, JOINT_VELOCITIES],
            'sensor_dims': SENSOR_DIMS,
        },
        'network_model': tf_network,
        'iterations': 1000,
=======
            'obs_include': agent['obs_include'],
            'obs_image_data': [END_EFFECTOR_POINTS, REF_TRAJ],
            'sensor_dims': SENSOR_DIMS,
            'T': T,
            'ee_pos_indices': (14,23),
            'scale': 100,
        },
        'network_model': ref_traj_network if ATTENTION else tf_network,
        'iterations': 2500,
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
        'weights_file_prefix': EXP_DIR + 'policy',
    }
elif NNLIB == 'caffe':
    algorithm['policy_opt'] = {
        'type': PolicyOptCaffe,
        'weights_file_prefix': EXP_DIR + 'policy',
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
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 0.1,
    'l2': 0.0001,
    'ramp_option': RAMP_CONSTANT,
}

<<<<<<< HEAD
algorithm['cost'] = {
    'type': CostSum,
    'costs': [fk_cost1, torque_cost],
    'weights': [1.0, 0.4],
=======
fk_cost2 = {
    'type': CostFK,
    'target_end_effector': np.zeros(3 * EE_POINTS.shape[0]),
    'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
    'l1': 1.0,
    'l2': 0.0,
    'wp_final_multiplier': 10.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [torque_cost, fk_cost1],
    'weights': [0.5, 1.0],
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}


config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 1,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
<<<<<<< HEAD
    'num_samples': 5,
=======
    'num_samples': 2, # must be >1 to fit dynamics
>>>>>>> f7d301069f230ac6442abac95479a6b7c48479ec
}
if NNLIB is not None:
    config['verbose_policy_trials'] = 1

common['info'] = generate_experiment_info(config)


# common['info'] = (
#     'exp_name: ' + str(common['experiment_name'])              + '\n'
#     'alg_type: ' + str(algorithm['type'].__name__)             + '\n'
#     'alg_dyn:  ' + str(algorithm['dynamics']['type'].__name__) + '\n'
#     'alg_cost: ' + str(algorithm['cost']['type'].__name__)     + '\n'
#     'iterations: ' + str(config['iterations'])                   + '\n'
#     'conditions: ' + str(algorithm['conditions'])                + '\n'
#     'samples:    ' + str(config['num_samples'])                  + '\n'
# )
