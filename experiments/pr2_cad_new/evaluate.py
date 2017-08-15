import glob
import numpy as np
import os.path as osp
import pdb
import cPickle as pickle
import re

from gps.proto.gps_pb2 import END_EFFECTOR_POINTS


NORM = np.inf # which norm to use for computing distances
def ee_distances(X_ee):
    return np.array([np.linalg.norm(X_ee[:,3*i:3*(i+1)], ord=NORM, axis=1) for i in range(3)]).T

SUCCESS_EPSILON = 0.02
SUCCESS_STEPS = 5
# a trial is considered successful if the end effectors' norms (distances)
# are all at most SUCCESS_EPSILON for the last SUCCESS_STEPS timesteps
def success(sample):
    X_ee = sample.get(END_EFFECTOR_POINTS)
    dist = ee_distances(X_ee[-SUCCESS_STEPS:])
    # print dist
    return np.all(dist <= SUCCESS_EPSILON)

def unpickle(filename):
    f = open(filename, 'r')
    data = pickle.load(f)
    f.close()
    return data

def list_files(dir):
    return glob.glob(osp.join(dir, 'pol_sample_itr_*.pkl'))

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

    return {itr: unpickle(itrs[itr]) for itr in include}

# evaluate the same function on every sample, across all iterations/conditions
def evaluate_sample_function(data, fn):
    results = {}
    for itr in data:
        results[itr] = []
        for _, sample_list in enumerate(data[itr]):
            results[itr].append([fn(sample) for sample in sample_list])
    return results

def main(dir):
    data = load_data(dir)
    successes = evaluate_sample_function(data, success)
    for itr in successes:
        print 'Iteration', itr
        for cond in range(len(successes[itr])):
            print '\t', successes[itr][cond]

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Evaluate results of policy trials')
    parser.add_argument('-d', '--dir', metavar='DIR', default='data_files')
    args = parser.parse_args()
    main(args.dir)
