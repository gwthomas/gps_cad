""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import numpy as np
import os.path as osp
import cPickle as pickle

from gps import __file__ as gps_filepath
from gps.agent.ros.cad.util import ConditionInfo
from gps.agent.ros.cad.pr2_cad_example_ex import AgentCADExample 

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


T = 200
ALL_CONDITIONS = 1
TRAIN_CONDITIONS = 1
MODE = 'iLQR subset'
assert MODE in ('GPS', 'iLQR subset', 'NN test')


EE_POINTS = np.array([[0.0, 0.0, 0.0], [0.15, 0.05, 0.0], [0.15, -0.05, 0.0]])
PR2_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

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
-1.9151435630987073, 1.0815418108712371]]

#for info in condition_info:
for i in range(ALL_CONDITIONS):
    x0s.append(np.zeros(32))
    ee_tgts.append(np.zeros(9))
    reset_condition = {
        TRIAL_ARM:      {'data': np.array(manual_conds[i]), 'mode': 1},
        #TRIAL_ARM:     {'data': info.initial, 'mode': 1},
        AUXILIARY_ARM: {'data': np.array([-1.25, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]), 'mode': 1}
    }
    reset_conditions.append(reset_condition)
    condition_count += 1 # Increment by one

if not osp.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentCADExample,
    'dt': 0.05,
    'conditions': common['conditions'],
    'actual_conditions': conditions,
    'condition_info': condition_info,
    'T': T,
    'T_interpolation': 150,
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'special_reset': False,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                    END_EFFECTOR_POINT_VELOCITIES, REF_TRAJ, TIMESTEP],
    'planner': 'RRTConnectkConfigDefault',
    'planning_time': 30,
    'plan_attempts': 5,
    'targets': [{'position': (0.5, 0.09, 0.555), 'orientation': (3.14, 0.0, -1.57)}
        for _ in range(common['conditions'])],
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
