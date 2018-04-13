import imp
import numpy as np
import os; osp = os.path
import pdb
import re

from gps.agent.ros.cad.agent_cad import AgentCAD
from gps.agent.ros.cad.agent_cad_experiment import AgentCADExperiment
from gps.algorithm.algorithm_traj_opt_smart import AlgorithmTrajOptSmart
from gps.sample.sample_list import SampleList
from gps.utility.data_logger import DataLogger
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, REF_TRAJ, REF_OFFSETS, TIMESTEP


#EE_LOW = (0.2, -0.2, 0.4)
EE_LOW = (0.42, -0.15, 1.0)
#EE_HIGH = (0.5, 0.6, 0.6)
EE_HIGH = (0.6, 0.5, 1.3)
JA_LOW = (-0.2, -0.5, -0.25, -1.5, -0.5, -1.0, -2.0)
JA_HIGH = (0.5, 0.0, 1.5, 0.0, 0.5, 0.0, 2.0)


def randstr(k):
    import random, string
    letters = 'abcdefghijklmnopqrstuvwxyz'
    #letters = string.ascii_lowercasse
    return ''.join(random.choice(letters) for _ in range(k))

DIR = osp.join('/home/melissachien/new_gps/experiments/pr2_cad_new/augmentation', randstr(8))
if not osp.isdir(DIR):
    os.mkdir(DIR)

def setup_agent(cfg):
    np.random.seed()
    condition_info = cfg['agent']['condition_info']
    targets = []
    for i, info in enumerate(condition_info):
        position = np.random.uniform(EE_LOW, EE_HIGH)
        rot_x = np.random.uniform(-np.pi/2, np.pi/2)
        rot_y = np.random.uniform(-np.pi/4, np.pi/4)
        rot_z = np.random.uniform(-np.pi, np.pi)
        orientation = [rot_x, rot_y, rot_z]
        targets.append({'position': position, 'orientation': orientation})

        info.initial = np.random.uniform(JA_LOW, JA_HIGH)
        cfg['agent']['reset_conditions'][i][TRIAL_ARM]['data'] = info.initial
        cond = int(re.findall('[0-9]{2}', info.path)[-1])
        info.path = osp.join(DIR, 'info%02d.pkl' % cond)
        info.data_path = osp.join(DIR, 'cond%02d.pkl' % cond)
        info.plan = None
    cfg['agent']['targets'] = targets
    return AgentCAD(cfg['agent'])

def setup_algorithm(cfg):
    return AlgorithmTrajOptSmart(cfg['algorithm'])

def iteration(itr, cfg, agent, algorithm):
    conditions = len(cfg['common']['train_conditions'])
    samples_per_cond = cfg['num_samples']
    sample_lists = []
    for cond in range(conditions):
        policy = algorithm.cur[cond].traj_distr
        sample_list = SampleList([agent.sample(policy, cond, save=False, noisy=True) for _ in range(samples_per_cond)])
        sample_lists.append(sample_list)
    algorithm.iteration(sample_lists)

def main():
    pdb.set_trace()
    hyperparams = imp.load_source('hyperparams', 'hyperparams.py')
    cfg = hyperparams.config
    arch = imp.load_source('arch', 'arch.py')
    manage = imp.load_source('manage', 'manage.py')
    agent = setup_agent(cfg)
    cfg['algorithm']['agent'] = agent
    algorithm = setup_algorithm(cfg)

    try:
       agent.delete_model('fixed_piece')
    except: pass
    # try:
    #    agent.delete_model('held_piece')
    # except: pass
    try:
       agent.delete_model('table')
    except: pass

    agent.scene.remove_world_object()
    # agent.add_object('table', position=[0.6,0.,0.42], size=[0.9,1.5,0.03], type='box')

    for itr in range(cfg['iterations']):
        iteration(itr, cfg, agent, algorithm)


if __name__ == '__main__':
    main()
