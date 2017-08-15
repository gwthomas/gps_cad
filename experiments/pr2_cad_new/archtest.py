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


def list_files(dir):
    assert osp.isdir(dir)
    return glob.glob(osp.join(dir, 'algorithm_itr_*.pkl'))

def load_data(dir):
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
        include = itrs
    else:
        print 'Here are the iterations for which data has been collected:'
        print sorted(itrs)
        include = raw_input('Which iterations would you like to include? ')
        if include == "all":
            include = itrs
        else:
            include = eval(include)
            if type(include) == int:
                include = [include]
            elif type(include) in (list, tuple):
                pass
            else:
                raise TypeError('Input should be an int or list/tuple thereof, or the keyword "all".')

    data_logger = DataLogger()
    algorithm_states, traj_sample_lists = [], []
    for itr in include:
        # adapted from gps_main.py
        algorithm_file = osp.join(dir, 'algorithm_itr_%02d.pkl' % itr)
        algorithm = data_logger.unpickle(algorithm_file)
        if algorithm is None:
            raise RuntimeError("Cannot find '%s'" % algorithm_file)
        traj_samples = data_logger.unpickle(osp.join(dir, 'traj_sample_itr_%02d.pkl' % itr))
        algorithm_states.append(algorithm)
        traj_sample_lists.append(traj_samples)
    return algorithm_states, traj_sample_lists

def save_data(output_dir, sample_lists, itr):
    if not osp.isdir(output_dir):
        os.makedirs(output_dir)
    DataLogger().pickle(osp.join(output_dir, 'pol_sample_itr_%02d.pkl' % itr), sample_lists)

def setup_policy_opt(algorithm, hyperparams, arch):
    policy_opt = copy.copy(hyperparams.algorithm['policy_opt'])
    if arch == 'mlp':
        policy_opt['network_model'] = mlp_network
    elif arch == 'distance':
        policy_opt['network_model'] = distance_network
    elif arch == 'distance_offset':
        policy_opt['network_model'] = distance_offset_network
    elif arch == 'ntm':
        policy_opt['network_model'] = ntm_network
    else:
        raise RuntimeError('Invalid architecture: {}'.format(arch))

    from tensorflow.python.framework import ops
    ops.reset_default_graph()  # we need to destroy the default graph before re_init or checkpoint won't restore.
    return policy_opt['type'](policy_opt, algorithm.dO, algorithm.dU)

def take_policy_samples(agent, policy, conditions, n):
    return [SampleList([agent.sample(policy, cond, save=False, noisy=False) for _ in range(n)]) for cond in range(conditions)]

def main(arch, n):
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    data_dir = hyperparams.common['data_files_dir']
    output_dir = osp.join(data_dir, arch)
    algorithm_states, traj_sample_lists = load_data(data_dir)
    agent = AgentCADExperiment(hyperparams.config['agent'], trace=False)
    for itr in range(len(algorithm_states)):
        algorithm, traj_samples = algorithm_states[itr], traj_sample_lists[itr]
        for m in range(algorithm.M):
            algorithm.cur[m].sample_list = traj_samples[m]
        algorithm.policy_opt = setup_policy_opt(algorithm, hyperparams, arch)
        for _ in range(algorithm._hyperparams['inner_iterations']):
            algorithm._update_policy(0)
        sample_lists = take_policy_samples(agent, algorithm.policy_opt.policy, hyperparams.common['conditions'], n)
        save_data(output_dir, sample_lists, itr)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Evaluate results of policy trials')
    parser.add_argument('arch', metavar='ARCH')
    parser.add_argument('-n', metavar='N', type=int, default=1)
    args = parser.parse_args()
    main(args.arch, args.n)
