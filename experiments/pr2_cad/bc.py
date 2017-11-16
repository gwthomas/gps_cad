import copy
import glob
import imp
import numpy as np
import os; osp = os.path
import pdb
import cPickle as pickle

from gps.sample.sample_list import SampleList
from gps.utility.data_logger import DataLogger
from gps.agent.ros.cad.ref_traj_network import *
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.agent.ros.cad.util import FakeAgent
from gps.algorithm.algorithm_utils import PolicyInfo


def load(path):
    with open(path, 'rb') as f:
        return pickle.load(f)

def load_augmentation():
    files = glob.glob('/home/gwthomas/workspace/gps/experiments/pr2_cad/augmentation/*/cond*.pkl')
    data = []
    for filename in files:
        data.append(load(filename))
    return data

def train_data(condition_data, T, dO, dU):
    pol_info = PolicyInfo({
        'init_pol_wt': 0.01,
        'T': T,
        'dU': dU,
        'dX': 32
    })

    obs_data, tgt_mu = np.zeros((0, T, dO)), np.zeros((0, T, dU))
    tgt_prc, tgt_wt = np.zeros((0, T, dU, dU)), np.zeros((0, T))

    # for m in condition_data:
        # samples, traj = condition_data[m]['samples'], condition_data[m]['traj_distr']
    for data in condition_data:
        samples, traj = data['samples'], data['traj_distr']
        X = samples.get_X()
        N = len(samples)
        mu = np.zeros((N, T, dU))
        prc = np.zeros((N, T, dU, dU))
        wt = np.zeros((N, T))
        # Get time-indexed actions.
        for t in range(T):
            # Compute actions along this trajectory.
            prc[:, t, :, :] = np.tile(traj.inv_pol_covar[t, :, :], [N, 1, 1])
            for i in range(N):
                mu[i, t, :] = \
                        (traj.K[t, :, :].dot(X[i, t, :]) + traj.k[t, :]) - \
                        np.linalg.solve(
                            prc[i, t, :, :] / pol_info.pol_wt[t],
                            pol_info.lambda_K[t, :, :].dot(X[i, t, :]) + \
                                    pol_info.lambda_k[t, :]
                        )
            wt[:, t].fill(pol_info.pol_wt[t])
        tgt_mu = np.concatenate((tgt_mu, mu))
        tgt_prc = np.concatenate((tgt_prc, prc))
        tgt_wt = np.concatenate((tgt_wt, wt))
        obs_data = np.concatenate((obs_data, samples.get_obs()))
    return obs_data, tgt_mu, tgt_prc, tgt_wt

def main(attention, structure, batchsize, n, resume, sgd, augment):
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    arch = imp.load_source('arch', 'arch.py')
    manage = imp.load_source('manage', 'manage.py')
    agent = FakeAgent(hyperparams.config['agent'])

    T = hyperparams.T
    dO, dU = 32 + 9*T + 1, 7
    output_dir = osp.join(hyperparams.common['data_files_dir'], '{}_{}'.format(attention, structure))

    condition_info = manage.load_all()
    n_cond = 10
    good_conditions = []
    for cond, info in condition_info.iteritems():
        if len(good_conditions) < n_cond and info.good:
            good_conditions.append(cond)

    if len(good_conditions) != n_cond:
        print 'Not enough good conditions on which to train! Exiting.'
        exit()

    print 'Training on conditions', good_conditions

    if sgd:
        hyperparams.algorithm['policy_opt']['solver_type'] = 'momentum'
    hyperparams.algorithm['policy_opt']['max_iterations'] = int(1e6)
    hyperparams.algorithm['policy_opt']['batch_size'] = batchsize
    hyperparams.algorithm['policy_opt']['termination_epsilon'] = 0.0

    policy_opt = arch.setup_policy_opt(hyperparams, attention, structure, dO, dU)
    policy_path = osp.join(output_dir, 'policy')

    if resume:
        try:
            policy_opt.restore_model(policy_path)
        except:
            print 'Failed to restore model'

    print 'Loading good data'
    good_data = [condition_info[cond].load_data() for cond in good_conditions]

    if augment:
        print 'Loading augmentation'
        augmentation = load_augmentation()
        all_data = good_data + augmentation
    else:
        all_data = good_data

    for data in all_data:
        # put the agent back
        for sample in data['samples'].get_samples():
            sample.agent = agent

    print 'Training!'
    try:
        policy_opt.update(*train_data(all_data, T, dO, dU))
    except KeyboardInterrupt:
        pass

    print 'Saving policy'
    policy_opt.save_model(policy_path)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Train policy with given attention and structure')
    parser.add_argument('attention', metavar='ATTENTION')
    parser.add_argument('structure', metavar='STRUCTURE')
    parser.add_argument('--batchsize', metavar='BATCHSIZE', type=int, default=32)
    parser.add_argument('-n', metavar='N', type=int, default=1)
    parser.add_argument('--resume', action='store_true') # resume training from last checkpoint
    parser.add_argument('--sgd', action='store_true') # use (momentum) SGD instead of Adam
    parser.add_argument('--augment', action='store_true') # use augmenting data
    args = parser.parse_args()
    main(args.attention, args.structure, args.batchsize, args.n, args.resume, args.sgd, args.augment)
