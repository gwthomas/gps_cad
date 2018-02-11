import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import pdb
import cPickle as pickle
import re
import rospy
from geometry_msgs.msg import Point, Quaternion
from gps_agent_pkg.srv import ProxyControl, ProxyControlResponse
from gps.agent.agent import Agent
from gps.agent.ros.ros_utils import TimeoutException


# Samples don't pickle their agents, but they need the indices (_x_data_idx and its ilk).
# We use this to avoid launching a ROS node upon __init__ (which AgentROS does)
# so it can be instantiated without fucking up any training happening at the time.
class FakeAgent(Agent):
    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
        pass


class ConditionInfo:
    def __init__(self, initial, path, data_path):
        self.initial = initial
        self.path = path
        self.data_path = data_path
        self.plan = None
        self.good = False
        self.actual = int(re.findall('[0-9]{2}', path)[-1])

    def save(self):
        with open(self.path, 'wb') as f:
            pickle.dump(self, f)

    def save_data(self, samples, traj_distr):
        with open(self.data_path, 'wb') as f:
            pickle.dump({'samples': samples, 'traj_distr': traj_distr}, f)

    def load_data(self):
        with open(self.data_path, 'rb') as f:
            return pickle.load(f)


class ProxyTrialManager(object):
    def __init__(self, agent, dt=0.01):
        self.agent = agent
        self.dt = dt
        self.service = rospy.Service('proxy_control', ProxyControl, self.handle_request)

    def handle_request(self, request):
        print("Request being handled!")
        try:
            self.t += 1
        except AttributeError:
            print 'ProxyTrialManager: must call prep before run'
            raise
        obs = self.agent.get_obs(request, self.condition)
        response = ProxyControlResponse()
        response.action = self.agent.get_action(self.policy, obs)
        print("T=" + str(self.t) + ": " + str(response.action))
        return response

    def prep(self, policy, condition):
        self.policy = policy
        self.condition = condition
        self.t = 0

    def run(self, time_to_run):
        time_elapsed = 0
        while self.t < self.agent.T:
            rospy.sleep(self.dt)
            time_elapsed += self.dt
            if time_elapsed > time_to_run:
                raise TimeoutException(time_elapsed)

        print("Time elapsed is this: " + str(self.t))


def center_trajectory(ee_pos, ref_traj):
    pass

def listify(o):
    if isinstance(o, Point):
        return [o.x, o.y, o.z]
    if isinstance(o, Quaternion):
        return [o.x, o.y, o.z, o.w]
    else:
        return list(o)


# def interpolate(points, T):
#     n = len(points)
#     if n > T:
#         raise RuntimeError('Cannot interpolate {} points over {} timesteps'.format(n,T))
#
#     interpolated_points = []
#     segment_lengths = []
#     num_segments = n - 1
#     base_pps = T // num_segments    # points per segment
#     leftover = T - base_pps * num_segments
#     for segment_idx in range(num_segments):
#         prev, next = points[segment_idx], points[segment_idx+1]
#         segment = []
#         if segment_idx < num_segments - 1:
#             if leftover:
#                 extra = np.random.binomial(leftover, 1.0/num_segments)
#                 leftover -= extra
#             else:
#                 extra = 0
#             points_in_this_segment = base_pps + extra
#             close = False
#         else:
#             points_in_this_segment = T - sum(segment_lengths)   # los demas
#             close = True
#
#         segment_lengths.append(points_in_this_segment)
#
#         for i in range(points_in_this_segment):
#             if close:
#                 progress = float(i) / (points_in_this_segment - 1)
#             else:
#                 progress = float(i) / points_in_this_segment
#             interpolated_point = prev + progress * (next - prev)
#             interpolated_points.append(interpolated_point)
#             segment.append(interpolated_point)
#
#     print 'Interpolation segment lengths', segment_lengths
#     return interpolated_points

def interpolate(points, T):
    n = len(points)
    if n > T:
        raise RuntimeError('Cannot interpolate {} points over {} timesteps'.format(n,T))

    interpolated_points = []
    num_segments = n - 1
    segment_lengths = np.array([T // num_segments] * num_segments)
    uniform = np.ones(num_segments) / num_segments
    for i in range(T - sum(segment_lengths)):
        segment_lengths[i] += 1
    for segment_idx in range(num_segments):
        prev, next = points[segment_idx], points[segment_idx+1]
        segment = []
        points_in_this_segment = segment_lengths[segment_idx]
        close = segment_idx == num_segments - 1

        for i in range(points_in_this_segment):
            if close:
                if points_in_this_segment > 1:
                    progress = float(i) / (points_in_this_segment - 1)
                else:
                    progress = 1.0
            else:
                progress = float(i) / points_in_this_segment
            interpolated_point = prev + progress * (next - prev)
            interpolated_points.append(interpolated_point)
            segment.append(interpolated_point)

    print 'Interpolation segment lengths', segment_lengths
    assert sum(segment_lengths) == T
    return interpolated_points

def find_closest_T(ref_ja, ref_ee, goal_ref_ja, goal_ref_ee):
    cur_count = 0 # For counting which timestep you are on
    closest_dist_ja, closest_dist_ee = float("inf"), float("inf")
    closet_T_ja, closest_T_ee = None, None

    for pt in ref_ja: # Go through all the items in the reference ja
        cur_dist = np.sqrt(np.sum(np.square(np.array(pt) - goal_ref_ja)))
        if cur_dist < closest_dist_ja: # Gota store this thing
            closest_dist_ja, closest_T_ja = cur_dist, cur_count
        cur_count += 1 # Increment the current count

    cur_count = 0 # Reset this to 0
    for pt in ref_ee: # Go through all the items in the reference ee
        cur_dist = np.sqrt(np.sum(np.square(np.array(pt) - goal_ref_ee)))
        if cur_dist < closest_dist_ee: # Gota store this thing
            closest_dist_ee, closest_T_ee = cur_dist, cur_count
        cur_count += 1 # Increment the current count

    # Average the closest things and it will be good
    return (closest_T_ee + closest_T_ja) // 2

def yesno(s):
    while True:
        response = raw_input(s + ' (y/n) ')
        if response in ('y', 'n'):
            return response == 'y'
        else:
            print 'Invalid input: expected y or n but got', response


def plot_trajectories(trajectories):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for traj in trajectories:
        xs, ys, zs = [], [], []
        for pose in traj:
            xs.append(pose[0])
            ys.append(pose[1])
            zs.append(pose[2])
        ax.plot(xs, ys, zs)
    plt.legend(['Traj {}'.format(i) for i in range(len(trajectories)-1)])
    plt.savefig('trajectories.png')
    # plt.show() # gives threading error
