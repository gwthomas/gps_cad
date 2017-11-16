import glob
import matplotlib.pyplot as plt
import numpy as np
import os.path as osp
import pdb
import cPickle as pickle
import re

from gps.proto.gps_pb2 import END_EFFECTOR_POINTS


NORM = np.inf # which norm to use for computing distances
# a trial is considered successful if there exists a t
# such that the end effectors' norms (distances) are at most a given threshold

def ee_distances(X_ee):
    return np.array([np.linalg.norm(X_ee[:,3*i:3*(i+1)], ord=NORM, axis=1) for i in range(3)]).T

def distance(sample):
    X_ee = sample.get(END_EFFECTOR_POINTS)
    # dist = ee_distances(X_ee[-STEPS:])
    dist = ee_distances(X_ee)
    return dist

def success_fn(threshold):
    def success(sample):
        dists = ee_distances(sample.get(END_EFFECTOR_POINTS))
        return np.any([np.all(d_i <= threshold) for d_i in dists])
    return success

def unpickle(filename):
    with open(filename, 'r') as f:
        return pickle.load(f)

def list_files(dir, kind='traj'):
    # return glob.glob(osp.join(dir, 'pol_sample_itr_*.pkl'))
    return glob.glob(osp.join(dir, '{}_sample_itr_*.pkl'.format(kind)))

def load_data(filename, dir):
    if osp.isfile(filename):
        return {'test': unpickle(filename)}

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

    return {itr: unpickle(itrs[itr]) for itr in include}

# evaluate the same function on every sample, across all iterations/conditions
def evaluate_sample_function(data, fn):
    results = {}
    for itr in data:
        results[itr] = []
        for _, sample_list in enumerate(data[itr]):
            results[itr].append([fn(sample) for sample in sample_list])
    return results

def print_results(results):
    for itr in results:
        print 'Iteration', itr
        for cond in range(len(results[itr])):
            print '\tCondition', cond
            print '\tDistances', results[itr][cond]

def plot_results(values):
    for itr in successes:
        for cond in range(len(successes[itr])):
            pass


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Evaluate results of policy trials')
    parser.add_argument('-f', '--filename', metavar='FILENAME', default='')
    parser.add_argument('-d', '--dir', metavar='DIR', default='data_files')
    args = parser.parse_args()
    data = load_data(args.filename, args.dir)
    # distances = evaluate_sample_function(data, distance)
    # print_results(distances)
    for threshold in [0.01, 0.015, 0.02, 0.025]:
        successes = evaluate_sample_function(data, success_fn(threshold))
        # print_results(successes)
        for itr in successes:
            print 'Itr:', itr
            print 'Threshold:', threshold
            print '\tTotal successes:', np.sum(successes[itr])
            print '\tSuccess rate:', np.mean(successes[itr])
            for cond in range(len(successes[itr])):
                print '\t\tCondition', cond, ':', successes[itr][cond]
