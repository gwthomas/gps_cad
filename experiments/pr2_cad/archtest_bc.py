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
from gps.agent.agent import Agent
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.algorithm.algorithm_utils import PolicyInfo


# Samples don't pickle their agents, but they need the indices (_x_data_idx and its ilk).
# We use this because it doesn't launch a ROS node upon __init__ (unlike descendants of AgentROS)
# so it can be instantiated without fucking up any training happening at the time.
class FakeAgent(Agent):
    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
        pass


def train_data(condition_data):
    dO, dU, T = 1832, 7, 200
    pol_info = PolicyInfo({
        'init_pol_wt': 0.01,
        'T': T,
        'dU': dU,
        'dX': 32
    })

    obs_data, tgt_mu = np.zeros((0, T, dO)), np.zeros((0, T, dU))
    tgt_prc, tgt_wt = np.zeros((0, T, dU, dU)), np.zeros((0, T))

    for m in condition_data:
        samples, traj = condition_data[m]['samples'], condition_data[m]['traj_distr']
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

def main(attention, structure, n, sgd):
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    archtest = imp.load_source('archtest', 'archtest.py')
    manage = imp.load_source('manage', 'manage.py')
    agent = FakeAgent(hyperparams.config['agent'])

    T = hyperparams.T
    dO, dU = 32 + 9*T, 7
    output_dir = osp.join(hyperparams.common['data_files_dir'], '{}_{}'.format(attention, structure))

    condition_info = manage.load_all()
    relevant_data = {}
    # put the agent back
    for cond, info in condition_info.iteritems():
        if info.good:
            data = info.load_data()
            for sample in data['samples'].get_samples():
                sample.agent = agent
            relevant_data[cond] = data

    if not relevant_data:
        print 'No good conditions on which to train! Exiting.'
        exit()
    print 'Training on conditions', sorted(relevant_data.keys())

    if sgd:
        hyperparams.algorithm['policy_opt']['solver_type'] = 'momentum'
    hyperparams.algorithm['policy_opt']['max_iterations'] = 1000000
    hyperparams.algorithm['policy_opt']['batch_size'] = 10000
    hyperparams.algorithm['policy_opt']['termination_epsilon'] = 0.0
    policy_opt = archtest.setup_policy_opt(hyperparams, attention, structure, dO, dU)
    try:
        policy_opt.update(*train_data(relevant_data))
    except:
        pass

    raw_input('Finished training. Press enter to begin taking policy samples')
    agent = AgentCADExperiment(hyperparams.config['agent'], trace=False)
    sample_lists = archtest.take_policy_samples(agent, policy_opt.policy, hyperparams.common['conditions'], n)
    archtest.save_data(output_dir, sample_lists)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Train and test policy with given attention and structure')
    parser.add_argument('attention', metavar='ATTENTION')
    parser.add_argument('structure', metavar='STRUCTURE')
    parser.add_argument('-n', metavar='N', type=int, default=1)
    parser.add_argument('--sgd', action='store_true') # use (momentum) SGD instead of Adam
    args = parser.parse_args()
    main(args.attention, args.structure, args.n, args.sgd)
