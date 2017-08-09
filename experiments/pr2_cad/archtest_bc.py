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
from gps.algorithm.algorithm_utils import PolicyInfo


def train_data(algorithm, traj_samples):
    dO, dU, T, M = algorithm.dO, algorithm.dU, algorithm.T, algorithm.M
    pol_info = PolicyInfo({
        'init_pol_wt': 0.01,
        'T': T,
        'dU': dU,
        'dX': 32
    })

    obs_data, tgt_mu = np.zeros((0, T, dO)), np.zeros((0, T, dU))
    tgt_prc, tgt_wt = np.zeros((0, T, dU, dU)), np.zeros((0, T))

    for m in range(M):
        samples = traj_samples[m]
        X = samples.get_X()
        N = len(samples)
        # traj, pol_info = algorithm.cur[m].traj_distr, algorithm.cur[m].pol_info
        traj = algorithm.cur[m].traj_distr
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

def main(arch, n, s):
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    archtest = imp.load_source('archtest', 'archtest.py')
    T = hyperparams.T
    dO, dU = 32 + 9*T, 7
    data_dir = hyperparams.common['data_files_dir']
    output_dir = osp.join(data_dir, arch)
    traj_samples, algorithm, itr = archtest.load_data(data_dir, load_algorithm_too=True)
    agent = AgentCADExperiment(hyperparams.config['agent'], trace=False)

    if s:
        hyperparams.algorithm['policy_opt']['solver_type'] = 'momentum'
        hyperparams.algorithm['policy_opt']['max_iterations'] = 100000
    policy_opt = archtest.setup_policy_opt(hyperparams, arch, dO, dU)
    policy_opt.update(*train_data(algorithm, traj_samples))

    raw_input('Finished training. Press enter to begin taking policy samples')
    sample_lists = archtest.take_policy_samples(agent, policy_opt.policy, hyperparams.common['conditions'], n)
    archtest.save_data(output_dir, sample_lists, itr)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Evaluate results of policy trials')
    parser.add_argument('arch', metavar='ARCH')
    parser.add_argument('-n', metavar='N', type=int, default=1)
    parser.add_argument('-s', action='store_true') # slow, serious, SGD
    args = parser.parse_args()
    main(args.arch, args.n, args.s)
