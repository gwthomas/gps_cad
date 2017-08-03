""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.agent_cad import AgentCADExperiment

from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt

from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_fkt import CostFKT
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr, init_pd
from gps.gui.target_setup_gui import load_pose_from_npz
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points


# EE_POINTS = np.array([[0.02, -0.025, 0.05], [0.02, -0.025, -0.05],
#                       [0.02, 0.05, 0.0]])
EE_POINTS = np.array([[0.0, 0.0, 0.0], [0.15, 0.05, 0.0], [0.15, -0.05, 0.0]])

SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 7,
}

PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/pr2_cad/'

x0s = []
ee_tgts = []
reset_conditions = []

common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 1,
}

# TODO(chelsea/zoe) : Move this code to a utility functionece.stl'),
x0_info, target_info = [], []
for i in xrange(common['conditions']):

    ja_x0, ee_pos_x0, ee_rot_x0 = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'initial'
    )
    ja_aux, _, _ = load_pose_from_npz(
        common['target_filename'], 'auxiliary_arm', str(i), 'initial'
    )
    ja_tgt, ee_pos_tgt, ee_rot_tgt = load_pose_from_npz(
        common['target_filename'], 'trial_arm', str(i), 'target'
    )

    x0_info.append((ja_x0[0], ee_pos_x0[0]))
    target_info.append((ja_tgt[0], ee_pos_tgt[0]))

    x0 = np.zeros(32)
    x0[:7] = ja_x0
    x0[14:(14+3*EE_POINTS.shape[0])] = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_x0, ee_rot_x0).T
    )

    ee_tgt = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_tgt, ee_rot_tgt).T
    )

    aux_x0 = np.zeros(7)
    aux_x0[:] = ja_aux

    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[0:7],
        },
        AUXILIARY_ARM: {
            'mode': JOINT_SPACE,
            'data': aux_x0,
        },
    }

    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)


if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentCADExperiment,
    'dt': 0.05,
    'conditions': common['conditions'],
    'T': 100,
    'T_interpolation': 80,
    'x0': x0s,
    'x0_info': x0_info,
    'ee_points_tgt': ee_tgts,
    'target_info': target_info,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'planner': 'RRTStarkConfigDefault',
    'planning_schedule': [5, 10, 25],
    'targets': [
        # {'position': (0.55, 0.07, 0.55), 'orientation': (0.0, 0.0, -1.57)} # old block position. remove?
        # {'position': (0.5, 0.09, 0.555), 'orientation': (0.0, 0.0, -1.57)}
        {'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
    ],
    'cad_path': os.path.join(EXP_DIR, 'piece.stl'),
    'reset_timeout': 3
}

algorithm = {
    'type': AlgorithmTrajOpt,
    'conditions': common['conditions'],
    'iterations': 50,
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / PR2_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1.0,
    'stiffness': 0.5,
    'stiffness_vel': 0.25,
    'final_weight': 50,
    'dt': agent['dt'],
    'T': agent['T'],
}
# algorithm['init_traj_distr'] = {
#     'type': or init_pd
#     'pos_gains': 1.0,
#     'vel_gains_mult': 0.3,
#     'init_var': 0.1,
#     'dQ': 7, # set this to action dim based on another file, but effect of changing unclear
#     'dt': agent['dt'],
#     'T': agent['T'],
# }

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

algorithm['policy_opt'] = {}

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
    'ramp_option': RAMP_LINEAR,
}

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
    # 'costs': [torque_cost, fk_cost1, fk_cost2],
    # 'weights': [1.0, 1.0, 1.0],
    'costs': [fk_cost1, torque_cost],
    'weights': [1.0, 0.1],
}


config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 1,
    'verbose_policy_trials': 1,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
    'num_samples': 5,
}

common['info'] = (
    'exp_name: ' + str(common['experiment_name'])              + '\n'
    'alg_type: ' + str(algorithm['type'].__name__)             + '\n'
    'alg_dyn:  ' + str(algorithm['dynamics']['type'].__name__) + '\n'
    'alg_cost: ' + str(algorithm['cost']['type'].__name__)     + '\n'
    'iterations: ' + str(config['iterations'])                   + '\n'
    'conditions: ' + str(algorithm['conditions'])                + '\n'
    'samples:    ' + str(config['num_samples'])                  + '\n'
)
