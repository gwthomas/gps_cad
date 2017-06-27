import collections as co
import hashlib
import numpy as np
import os.path as osp
import re

CATS = [
        'trial_arm__target_ee_pos',
        'trial_arm__target_ee_rot',
        'trial_arm__target_ja',
        'auxiliary_arm__initial_ee_pos',
        'auxiliary_arm__initial_ee_rot',
        'auxiliary_arm__initial_ja',
        'trial_arm__initial_ee_pos',
        'trial_arm__initial_ee_rot',
        'trial_arm__initial_ja'
]

def parse(s):
    m = re.search('[0-9]', s)
    start, end = m.span()
    return s[:start] + s[end:], int(s[start])

def recat(category, condition):
    m = re.search('__', category)
    start, end = m.span()
    return category[:start+1] + str(condition) + category[end-1:]

def equalish(a,b):
    return a.shape == b.shape and np.allclose(a,b)

class NumpySet:
    def __init__(self):
        self.arrays = []

    def add(self, array):
        for seen in self.arrays:
            if equalish(array, seen):
                return False
        self.arrays.append(array)
        return True

def main(in_dirs, out_dir):
    seen = NumpySet()
    all_data = co.defaultdict(list)
    for in_dir in in_dirs:
        dir_data = co.defaultdict(list)
        target = np.load(osp.join(in_dir, 'target.npz'))
        num_keys = len(target.keys())

        for key in target.keys():
            category, condition = parse(key)
            assert key == recat(category, condition)
            data = target[key]
            if seen.add(data):
                dir_data[category].append((condition, data))

        # add to all data, sorted by condition
        for key in dir_data:
            sorted_data = sorted(dir_data[key])
            all_data[key].extend([data for condition, data in sorted_data])

    # print all_data
    for key in all_data:
        print key, len(all_data[key])

if __name__ == '__main__':
    main([
        'pr2_badmm_example',
        'pr2_example',
        'pr2_tensorflow_example'
    ], 'pr2_cad')
