import cPickle as pickle
import imp
import numpy as np
import pdb
manage = imp.load_source('manage', 'manage.py')
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE, REF_TRAJ, REF_OFFSETS, TIMESTEP

modpath = lambda s: s.replace('condition_info', 'condition_info_original')
unmodpath = lambda s: s.replace('condition_info_original', 'condition_info')

manage.DIR = modpath(manage.DIR)
condition_info = manage.load_all()
for cond, info in condition_info.iteritems():
    info.data_path = modpath(info.data_path)
    data = None
    try:
        data = info.load_data()
    except: pass
    if data:
        samples, traj_distr = data['samples'], data['traj_distr']
        for sample in samples.get_samples():
            sample._data[TIMESTEP] = np.reshape(np.arange(sample.T), [sample.T,1])
            sample.dO += 1
            sample._obs = np.empty((sample.T, sample.dO))   # resize
            sample._obs.fill(np.nan)    # fill with nan because np.empty gives garbage
        info.data_path = unmodpath(info.data_path)
        info.save_data(samples, traj_distr)
    info.save()
