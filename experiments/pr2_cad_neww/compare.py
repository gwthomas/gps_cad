import numpy as np
import os; osp = os.path
import pdb
import cPickle as pickle
import matplotlib.pyplot as plt
import sys

if __name__ == '__main__':
	itrs = [int(arg) for arg in sys.argv[1:]]
	for itr in itrs:
		with open('data_files/algorithm_itr_%02d.pkl' % itr) as f:
			alg = pickle.load(f)
		with open('data_files/pol_sample_itr_%02d.pkl' % itr) as f:
			samples = pickle.load(f)
		with open('data_files/agent_itr_%02d.pkl' % itr) as f:
			agent = pickle.load(f)

		ilqr = alg.cur[0].pol_info.traj_distr()
		s = samples[0][0]
		s.agent = agent
		X = s.get_X()
		U_pol = s.get_U()
		U_ilqr = np.zeros_like(U_pol)
		for t in range(alg.T):
			U_ilqr[t] = ilqr.act(X[t], None, t, noise=np.zeros(7))
		U_diff = U_pol - U_ilqr
		for i in range(7):
			plt.figure()
			plt.plot(U_pol[:, i])
			plt.plot(U_ilqr[:, i])
			plt.plot(U_diff[:, i])
			plt.title("Joint: " + str(i))
			plt.legend(['pol', 'ilqr', 'diff'])

		'''
		U_pol_norm = np.linalg.norm(U_pol, axis=1)
		U_ilqr_norm = np.linalg.norm(U_ilqr, axis=1)
		U_diff_norm = np.linalg.norm(U_diff, axis=1)
		plt.figure()
		plt.title('Iteration %d' % itr)
		plt.plot(U_pol_norm)
		plt.plot(U_ilqr_norm)
		plt.plot(U_diff_norm)s
		'''

	plt.show()
