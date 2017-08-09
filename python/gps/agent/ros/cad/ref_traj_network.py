import numpy as np
import pdb
import tensorflow as tf

from gps.algorithm.policy_opt.tf_model_example import *
from gps.algorithm.policy_opt.tf_utils import TfSolver

DEFAULT_HIDDEN = 100

def _shift_indices(T, k):
    indices = np.arange(T, dtype='int32')
    if k < 0:
        k = -k
        indices[:T-k] = indices[k:T]
        indices[T-k:] = T - 1
    elif k > 0:
        indices[k:T] = indices[:T-k]
        indices[:k] = 0
    return indices

def _reorder_columns(matrix, idx):
    return tf.transpose(tf.gather(tf.transpose(matrix), idx))

def _shifted_matrices(network_config, W):
    T = network_config['T']
    k = network_config['max_shift']
    tape = np.concatenate([
            np.zeros(k),
            np.arange(T),
            np.full(k, T-1)
    ]).astype('int32')
    def shift_fn(w):
        # return tf.stack([w[tape[i:i+2*k+1]] for i in range(T)])
        return tf.stack([tf.gather(w, tape[i:i+2*k+1]) for i in range(T)])
    return tf.map_fn(shift_fn, W)

def _sharpen(w, gamma):
    # wpow = w ** gamma
    wpow = tf.pow(w, tf.reshape(gamma, [-1,1]))
    return wpow / tf.reshape(tf.reduce_sum(wpow, axis=1), [-1,1])

def _fixed_distance_coeffs(network_config, state, ee_pos, ref_traj, offset):
    T = network_config['T']
    scale = network_config['fixed_scale']
    distances = tf.norm(ref_traj - tf.reshape(ee_pos, [-1,1,9]), axis=2)
    if offset != 0:
        distances = _reorder_columns(distances, _shift_indices(T, offset))
    distances = tf.identity(distances, name='distances')
    coeffs = tf.nn.softmax(-scale * distances)
    return coeffs, []

def _distance_coeffs(network_config, state, ee_pos, ref_traj, offset):
    T = network_config['T']
    sizes = network_config['hidden_attention'] + [1] # one output (beta)
    with tf.variable_scope('distance_{}'.format(offset)):
        mlp_out, weights, biases = get_mlp_layers(state, len(sizes), sizes)
    beta = mlp_out[:,0]**2  # >= 0
    distances = tf.norm(ref_traj - tf.reshape(ee_pos, [-1,1,9]), axis=2)
    if offset != 0:
        distances = _reorder_columns(distances, _shift_indices(T, offset))
    distances = tf.identity(distances, name='distances')
    coeffs = tf.nn.softmax(-tf.reshape(beta, [-1,1]) * distances)
    return coeffs, weights + biases

def _ntm_coeffs(network_config, state, ee_pos, ref_traj, offset):
    T = network_config['T']
    max_shift = network_config['max_shift']
    n_shifts = 2*max_shift + 1
    sizes = ['hidden_attention'] + [2+n_shifts] # two outputs (beta, gamma) plus shifts
    with tf.variable_scope('ntm'.format(offset)):
        mlp_out, weights, biases = get_mlp_layers(state, len(sizes), sizes)
    beta = mlp_out[:,0]**2          # >= 0
    gamma = 1. + mlp_out[:,1]**2    # >= 1
    shifts = tf.nn.softmax(mlp_out[:,2:2+n_shifts])
    distances = tf.norm(ref_traj - tf.reshape(ee_pos, [-1,1,9]), axis=2)
    distances = tf.identity(distances, name='distances')
    dist_coeffs = tf.nn.softmax(-tf.reshape(beta, [-1,1]) * distances)
    Ws = _shifted_matrices(network_config, dist_coeffs)
    shifted_coeffs = tf.reshape(tf.matmul(Ws, tf.reshape(shifts, [-1,n_shifts,1])), [-1,T])
    coeffs = _sharpen(shifted_coeffs, gamma)
    return coeffs, weights + biases

def _linear_combo(network_config, coeffs, ref_traj):
    T = network_config['T']
    return tf.reduce_sum(tf.reshape(coeffs, [-1,T,1]) * ref_traj, axis=1)

def _head(coeff_fn, network_config, state, ee_pos, ref_traj, offset):
    coeffs, weights = coeff_fn(network_config, state, ee_pos, ref_traj, offset)
    coeffs = tf.identity(coeffs, name='coeffs')
    return _linear_combo(network_config, coeffs, ref_traj), weights

def fixed_distance_attention(network_config, state, ee_pos, ref_traj):
    attended, weights = _head(_fixed_distance_coeffs, network_config, state, ee_pos, ref_traj, 0)
    return attended, weights

def distance_attention(network_config, state, ee_pos, ref_traj):
    attended, weights = _head(_distance_coeffs, network_config, state, ee_pos, ref_traj, 0)
    return attended, weights

def distance_attention_offset(network_config, state, ee_pos, ref_traj):
    offset = 5 if 'offset' not in network_config else network_config['offset']
    attended0, weights0 = _head(_distance_coeffs, network_config, state, ee_pos, ref_traj, 0)
    attendedk, weightsk = _head(_distance_coeffs, network_config, state, ee_pos, ref_traj, offset)
    return tf.concat([attended0, attendedk], 1), weights0 + weightsk

def ntm_attention(network_config, state, ee_pos, ref_traj):
    network_config['max_shift'] = 1
    attended, weights = _head(_ntm_coeffs, network_config, state, ee_pos, ref_traj, None)
    return attended, weights

def fc_layer(input, size, id, nonlinearity=tf.nn.relu):
    sofar = input
    in_shape = sofar.get_shape().dims[1].value
    cur_weight = init_weights([in_shape, size], name='w_' + str(id))
    cur_bias = init_bias([size], name='b_' + str(id))
    sofar = tf.nn.xw_plus_b(sofar, cur_weight, cur_bias)
    sofar = nonlinearity(sofar) if nonlinearity is not None else sofar
    return sofar, [cur_weight], [cur_bias]

def resnet_layer(input, id, nonlinearity=tf.nn.relu, nonlinear_output=True):
    size = input.get_shape().dims[1].value
    cur_weight1 = init_weights([size, size], name='w1_' + str(id))
    cur_bias1 = init_bias([size], name='b1_' + str(id))
    cur_weight2 = init_weights([size,size], name='w2_' + str(id))
    cur_bias2 = init_bias([size], name='b2_' + str(id))
    f = nonlinearity(tf.nn.xw_plus_b(input, cur_weight1, cur_bias1))
    residual = tf.nn.xw_plus_b(f, cur_weight2, cur_bias2)
    output = residual + input
    if nonlinear_output:
        output = nonlinearity(output)
    return output, [cur_weight1, cur_weight2], [cur_bias1, cur_bias2]

def get_resnet_layers(input, num_layers, nonlinearity=tf.nn.relu, nonlinear_output=True):
    sofar = input
    size = sofar.get_shape().dims[1].value
    weights, biases = [], []
    for i in range(num_layers):
        nonlinear_output_i = nonlinear_output or i + 1 < num_layers
        sofar, weights_i, biases_i = resnet_layer(sofar, i, nonlinearity, nonlinear_output_i)
        weights.extend(weights_i)
        biases.extend(biases_i)
    return sofar, weights, biases

def ref_traj_network_factory(attention, dim_input=27, dim_output=7, batch_size=25, network_config=None):
    tf.reset_default_graph()

    T = network_config['T']
    ee_pos_indices = network_config['ee_pos_indices']
    assert ee_pos_indices[1] - ee_pos_indices[0] == 9
    nn_input, action, precision = get_input_layer(dim_input, dim_output)
    state = nn_input[:,:-9*T]
    ref_traj = tf.reshape(nn_input[:,-9*T:], [-1,T,9])
    ee_pos = nn_input[:,ee_pos_indices[0]:ee_pos_indices[1]]
    ee_pos = tf.identity(ee_pos, name='ee_pos')

    if attention:
        attended, attn_weights = attention(network_config, state, ee_pos, ref_traj)
        attended = tf.identity(attended, name='attended')
        attention_direction = attended - ee_pos
        attention_direction = tf.identity(attention_direction, name='direction')
        augmented_state = tf.concat([state, attended, attention_direction], 1)
    else:
        augmented_state, attn_weights = state, []

    with tf.variable_scope('final'):
        mlp_sizes = network_config['mlp_hidden_sizes']
        with tf.variable_scope('mlp'):
            mlp_out, mlp_weights, mlp_biases = get_mlp_layers(augmented_state, len(mlp_sizes), mlp_sizes, nonlinear_output=True)
        resnet_n = network_config['resnet_n_hidden']
        with tf.variable_scope('resnet'):
            resnet_out, resnet_weights, resnet_biases = get_resnet_layers(augmented_state, resnet_n, nonlinear_output=True)
        fc_input = tf.concat([mlp_out, resnet_out], 1)
        with tf.variable_scope('fc'):
            fc_out, fc_weights, fc_biases = fc_layer(fc_input, dim_output, 'fc', nonlinearity=None)
        final_weights = mlp_weights + resnet_weights + fc_weights
        final_biases = mlp_biases + resnet_biases + fc_biases

    final_out = fc_out
    all_vars = attn_weights + final_weights + final_biases
    loss_out = get_loss_layer(mlp_out=final_out, action=action, precision=precision, batch_size=batch_size)
    return TfMap.init_from_lists([nn_input, action, precision], [final_out], [loss_out]), all_vars, []

def mlp_network(*args, **kwargs):
    return ref_traj_network_factory(None, *args, **kwargs)

def fixed_distance_network(*args, **kwargs):
    return ref_traj_network_factory(fixed_distance_attention, *args, **kwargs)

def distance_network(*args, **kwargs):
    return ref_traj_network_factory(distance_attention, *args, **kwargs)

def distance_offset_network(*args, **kwargs):
    return ref_traj_network_factory(distance_attention_offset, *args, **kwargs)

def ntm_network(*args, **kwargs):
    return ref_traj_network_factory(ntm_attention, *args, **kwargs)


if __name__ == '__main__':
    T = 100
    dim_input = 32 + 9*T
    dim_output = 7
    batch_size = 3
    network_config = {
            'T': T,
            'max_shift': 1,
            'ee_pos_indices': (0,9)
    }

    def gen_inputs(batch_size, dim_input):
        return np.random.randn(batch_size, dim_input)

    def gen_actions(batch_size, dim_output):
        # return np.random.randn(batch_size, action_dim)
        return np.array([[0,1,2,3,4,5,6] for _ in range(batch_size)])

    def main(net_name):
        net_fn = eval(net_name)
        network_info = net_fn(network_config=network_config, dim_input=dim_input, dim_output=dim_output, batch_size=batch_size)
        net = network_info[0]
        solver = TfSolver(
                loss_scalar=net.get_loss_op(),
                base_lr=1e-3,
                lr_policy='fixed',
                momentum=0.9
        )
        precisions = np.array([np.eye(dim_output)] * batch_size)

        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            while True:
                inputs = gen_inputs(batch_size, dim_input)
                actions = gen_actions(batch_size, dim_output)
                feed_dict = {
                        'nn_input:0': inputs,
                        'action:0': actions,
                        'precision:0': precisions
                }
                print solver(feed_dict, sess)

    def reorder_columns(matrix, idx):
        return tf.transpose(tf.gather(tf.transpose(matrix), idx))

    def test_reorder_columns():
        idx = np.array([0,0,1,2,3])
        mat_in = tf.placeholder(tf.float32, [None,5])
        out = reorder_columns(mat_in, idx)
        mat = np.arange(25).reshape((5,5))
        print mat
        with tf.Session() as sess:
            result = sess.run(out, feed_dict={mat_in: mat})
            print result

    def test_shift():
        mat_in = tf.placeholder(tf.float32, [None,T])
        out = _shifted_matrices(network_config, mat_in)
        mat = np.arange(batch_size*T).reshape((batch_size,T))
        with tf.Session() as sess:
            result = sess.run(out, feed_dict={mat_in: mat})
            print result

    # test_reorder_columns()
    # test_shift()

    import sys
    assert len(sys.argv) > 1, 'Must pass in name of network'
    main(sys.argv[1])
