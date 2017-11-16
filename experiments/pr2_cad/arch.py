import copy
import glob
import imp
import numpy as np
import os; osp = os.path
import pdb
import cPickle as pickle
import re

from gps.sample.sample_list import SampleList
from gps.utility.data_logger import DataLogger
from gps.agent.ros.cad.ref_traj_network import *
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.algorithm.algorithm_badmm import AlgorithmBADMM


def list_files(dir):
    assert osp.isdir(dir)
    return glob.glob(osp.join(dir, 'traj_sample_itr_*.pkl'))

def load_data(dir, load_algorithm_too=True):
    all_files = list_files(dir)
    itrs = {}
    for filename in all_files:
        itr = int(re.findall('[0-9]{2}', filename)[-1])
        itrs[itr] = filename

    if len(itrs) == 0:
        print 'No data found! Exiting.'
        exit()
    elif len(itrs) == 1:
        print 'Only one iteration found, so using that'
        itr = itr.keys()[0]
    else:
        print 'Here are the iterations for which data has been collected:'
        print sorted(itrs)
        itr = input('Which iteration would you like to train on? ')
        assert isinstance(itr, int)

    data_logger = DataLogger()
    # return [data_logger.unpickle(osp.join(dir, 'traj_sample_itr_%02d.pkl' % itr)) for itr in include]
    # adapted from gps_main.py
    traj_samples = data_logger.unpickle(osp.join(dir, 'traj_sample_itr_%02d.pkl' % itr))
    if load_algorithm_too:
        algorithm_state = data_logger.unpickle(osp.join(dir, 'algorithm_itr_%02d.pkl' % itr))
    else:
        algorithm_state = None
    return traj_samples, algorithm_state, itr

def setup_policy_opt(hyperparams, attention, structure, dO, dU):
    policy_opt = copy.copy(hyperparams.algorithm['policy_opt'])
    network_params = policy_opt['network_params']

    if attention == 'none':
        network_params['attention'] = None
    elif attention == 'fixed_distance':
        network_params['attention'] = fixed_distance_attention
    elif attention == 'distance':
        network_params['attention'] = distance_attention
    elif attention == 'distance_offset':
        network_params['attention'] = distance_offset_attention
    elif attention == 'ntm':
        network_params['attention'] = ntm_attention
    elif attention == 'time_fixed':
        network_params['attention'] = time_fixed_attention
    elif attention == 'time':
        network_params['attention'] = time_attention
    elif attention == 'centering':
        network_params['attention'] = centering_attention
    else:
        raise RuntimeError('Invalid attention: {}'.format(attention))

    if structure == 'mlp':
        network_params['structure'] = mlp_structure
    elif structure == 'factored_mlp':
        network_params['structure'] = factored_mlp_structure
    elif structure == 'mlp_resnet':
        network_params['structure'] = mlp_resnet_structure
    elif structure == 'linear':
        network_params['structure'] = linear_structure
    elif structure == 'corrected_linear':
        network_params['structure'] = corrected_linear_structure
    else:
        raise RuntimeError('Invalid structure: {}'.format(structure))

    from tensorflow.python.framework import ops
    ops.reset_default_graph()  # we need to destroy the default graph before re_init or checkpoint won't restore.
    return policy_opt['type'](policy_opt, dO, dU)


def main(arch, n):
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    data_dir = hyperparams.common['data_files_dir']
    output_dir = osp.join(data_dir, arch)
    traj_samples, algorithm, itr = load_data(data_dir)
    agent = AgentCADExperiment(hyperparams.config['agent'], trace=False)
    hyperparams.config['algorithm']['agent'] = agent

    # Install data and network
    for m in range(algorithm.M):
        algorithm.cur[m].sample_list = traj_samples[m]
    algorithm.policy_opt = setup_policy_opt(hyperparams, arch, algorithm.dO, algorithm.dU)

    algorithm._update_policy(0)
    print 'Finished training, will now take policy samples'
    sample_lists = take_policy_samples(agent, algorithm.policy_opt.policy, hyperparams.common['conditions'], n)
    save_data(output_dir, sample_lists, itr)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Evaluate results of policy trials')
    parser.add_argument('arch', metavar='ARCH')
    parser.add_argument('-n', metavar='N', type=int, default=1)
    args = parser.parse_args()
    main(args.arch, args.n)
