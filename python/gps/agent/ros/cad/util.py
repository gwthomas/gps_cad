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
