import copy
import numpy as np

from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt


class AlgorithmTrajOptSmart(AlgorithmTrajOpt):
    def __init__(self, hyperparams):
        AlgorithmTrajOpt.__init__(self, hyperparams)
        self.condition_info = hyperparams['condition_info']

    def iteration(self, sample_lists):
        AlgorithmTrajOpt.iteration(self, sample_lists)
        for m, info in enumerate(self.condition_info):
            print 'Saving data for condition', m
            print info.data_path
            info.save_data(sample_lists[m], self.cur[m].traj_distr)
