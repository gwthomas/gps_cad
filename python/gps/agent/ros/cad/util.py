import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

from geometry_msgs.msg import Point, Quaternion


def listify(o):
    if isinstance(o, Point):
        return [o.x, o.y, o.z]
    if isinstance(o, Quaternion):
        return [o.x, o.y, o.z, o.w]
    else:
        return list(o)


def interpolate(points, T):
    n = len(points)
    if n > T:
        raise RuntimeError('Cannot interpolate {} points over {} timesteps'.format(n,T))

    interpolated_points = []
    segment_lengths = []
    num_segments = n - 1
    base_pps = T // num_segments    # points per segment
    leftover = T - base_pps * num_segments
    for segment_idx in range(num_segments):
        prev, next = points[segment_idx], points[segment_idx+1]
        segment = []
        if segment_idx < num_segments - 1:
            if leftover:
                extra = np.random.binomial(leftover, 1.0/num_segments)
                leftover -= extra
            else:
                extra = 0
            points_in_this_segment = base_pps + extra
            close = False
        else:
            points_in_this_segment = T - sum(segment_lengths)   # los demas
            close = True

        segment_lengths.append(points_in_this_segment)

        for i in range(points_in_this_segment):
            if close:
                progress = float(i) / (points_in_this_segment - 1)
            else:
                progress = float(i) / points_in_this_segment
            interpolated_point = prev + progress * (next - prev)
            interpolated_points.append(interpolated_point)
            segment.append(interpolated_point)

    print 'Interpolation segment lengths', segment_lengths
    return interpolated_points


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

    # Add mean trajectory
    T = len(trajectories[0])
    mean_trajectory = []
    for t in range(T):
        mean_trajectory.append(np.mean([traj[t] for traj in trajectories], axis=0))
    trajectories.append(mean_trajectory)

    for traj in trajectories:
        xs, ys, zs = [], [], []
        for pose in traj:
            xs.append(pose[0])
            ys.append(pose[1])
            zs.append(pose[2])
        ax.plot(xs, ys, zs)
    plt.legend(['Traj {}'.format(i) for i in range(len(trajectories)-1)] + ['Mean'])
    plt.savefig('trajectories.png')
    # plt.show() # gives threading error
