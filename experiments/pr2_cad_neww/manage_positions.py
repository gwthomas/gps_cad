import numpy as np

# Limit values taken from https://www.clearpathrobotics.com/wp-content/uploads/2014/08/pr2_manual_r321.pdf
# JOINT_LIMITS = np.array([
#         (-40,130),  # l_shoulder_pan_joint
#         (-30,80),   # l_shoulder_lift_joint
#         (-44,224),  # l_upper_arm_roll_joint
#         (0,133),    # l_elbow_flex_joint
#         (-180,180), # l_forearm_roll_joint
#         (0,130),    # l_wrist_flex_joint
#         (-180,180)  # l_wrist_roll_joint
# ], dtype=float) * np.pi/180.0 # convert to radians


# [0.4, -0.25, 1.0, -0.5, 0.5, -0.5, 1.25]
# [-0.2, 0.0, 1.0, -0.75, 0.0, -0.6, 1.25]
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

def gen(n):
    return np.random.uniform(low=LOWER, high=UPPER, size=[n,DIM])

def main(filename):
    positions = None
    try:
        positions = np.load(filename + '.npy')
    except: pass

    if positions is None:
        print 'No existing positions found'
        n = input('How many would you like to generate? ')
        assert isinstance(n, int) and n > 0
        positions = gen(n)
    else:
        print 'Found existing positions'
        change = input('Which index/indices would you like to re-generate? ')
        if isinstance(change, int):
            change = [change]
        positions[change] = gen(len(change))

    np.save(filename, positions)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Generate test positions')
    parser.add_argument('-f', '--filename', metavar='FILENAME', default='test_positions')
    args = parser.parse_args()
    main(args.filename)
