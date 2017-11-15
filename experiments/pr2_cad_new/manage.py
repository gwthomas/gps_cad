import glob
import numpy as np
import os.path as osp
import cPickle as pickle
import re

from gps.agent.ros.cad.util import ConditionInfo, yesno


JOINT_LIMITS = np.array([
        (-0.2,0.5),     # l_shoulder_pan_joint
        (-0.5,0.25),   # l_shoulder_lift_joint
        (-0.25,1.5),    # l_upper_arm_roll_joint
        (-1.5,0.0),     # l_elbow_flex_joint
        (-0.5,0.5),     # l_forearm_roll_joint
        (-1.0,0.0),     # l_wrist_flex_joint
        (-2.0,2.0)      # l_wrist_roll_joint
])
LOWER, UPPER = JOINT_LIMITS[:,0], JOINT_LIMITS[:,1]
DIM = len(JOINT_LIMITS)
DIR = osp.join(osp.dirname(osp.realpath(__file__)), 'condition_info')

def rand_initial():
    return np.random.uniform(low=LOWER, high=UPPER, size=[DIM])

def info_path(cond):
    return osp.join(DIR, 'info%02d.pkl' % cond)

def data_path(cond):
    return osp.join(DIR, 'data%02d.pkl' % cond)

def load(cond):
    with open(info_path(cond), 'rb') as f:
        return pickle.load(f)

def load_all():
    files = glob.glob(osp.join(DIR, 'info*.pkl'))
    condition_info = {}
    for filename in files:
        cond = int(re.findall('[0-9]{2}', filename)[-1])
        condition_info[cond] = load(cond)
    return condition_info

def save_all(condition_info):
    for cond in condition_info:
        condition_info[cond].save()

def summarize(condition_info):
    sorted_conds = sorted(condition_info.keys())
    for cond in sorted_conds:
        info = condition_info[cond]
        print 'Condition', cond
        print '\tPlan?', info.plan is not None
        print '\tGood?', info.good
    print 'Good conditions:', [cond for cond in sorted_conds if condition_info[cond].good]

def new_info(cond):
    return ConditionInfo(rand_initial(), info_path(cond), data_path(cond))

def fill(condition_info, conds):
    for cond in conds:
        condition_info[cond] = new_info(cond)

def create(condition_info):
    new_conds = input('Enter (a Python expression for) the conditions to be created: ')
    fill(condition_info, new_conds)

def regenerate(condition_info, cond):
    if cond not in condition_info:
        print 'Condition', cond, 'does not exist!'
    else:
        info = condition_info[cond]
        if not info.good or yesno('This condition has been marked good. Are you sure you want to regenerate?'):
            info.initial = rand_initial()

def mark_good_as(condition_info, cond, value):
    if cond not in condition_info:
        print 'Condition', cond, 'does not exist!'
    else:
        condition_info[cond].good = value

def mark_good(condition_info, cond):
    mark_good_as(condition_info, cond, True)

def mark_bad(condition_info, cond):
    mark_good_as(condition_info, cond, False)

def manual_initials(condition_info, conds, initials):
    assert len(conds) == len(initials)
    for i, cond in enumerate(conds):
        condition_info[cond].initial = np.array(initials[i])

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Manage condition info')
    parser.add_argument('-s', action='store_true', help='print a summary')
    parser.add_argument('-n', action='store_true', help='add new conditions')
    parser.add_argument('-r', metavar='COND', type=int, default=-1, help="regenerate COND's initial")
    parser.add_argument('-g', metavar='COND', type=int, default=-1, help='mark COND as good')
    parser.add_argument('-b', metavar='COND', type=int, default=-1, help='mark COND as bad')
    args = parser.parse_args()

    condition_info = load_all()

    if args.s:
        summarize(condition_info)
    if args.n:
        create(condition_info)
    if args.r != -1:
        regenerate(condition_info, args.r)
    if args.g != -1:
        mark_good(condition_info, args.g)
    if args.b != -1:
        mark_bad(condition_info, args.b)

    save_all(condition_info)
