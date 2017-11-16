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

def _shifted_matrices(config, W):
    T = config['T']
    k = config['max_shift']
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

def _fixed_distance_coeffs(config, args, offset):
    T = config['T']
    temperature = config['temperature']
    ref_traj = args['ref_traj']
    ee_pos = args['ee_pos']
    distances = tf.norm(ref_traj - tf.reshape(ee_pos, [-1,1,9]), axis=2)
    if offset != 0:
        distances = _reorder_columns(distances, _shift_indices(T, offset))
    distances = tf.identity(distances, name='distances')
    coeffs = tf.nn.softmax(-distances / temperature)
    return coeffs, []

def _distance_coeffs(config, args, offset):
    T = config['T']
    sizes = config['hidden_attention'] + [1] # one output (beta)
    state = args['state']
    ref_traj = args['ref_traj']
    ee_pos = args['ee_pos']
    with tf.variable_scope('distance_{}'.format(offset)):
        mlp_out, weights, biases = get_mlp_layers(state, len(sizes), sizes)
    beta = mlp_out[:,0]**2  # >= 0
    distances = tf.norm(ref_traj - tf.reshape(ee_pos, [-1,1,9]), axis=2)
    if offset != 0:
        distances = _reorder_columns(distances, _shift_indices(T, offset))
    distances = tf.identity(distances, name='distances')
    coeffs = tf.nn.softmax(-tf.reshape(beta, [-1,1]) * distances)
    return coeffs, weights + biases

def _linear_combo(coeffs, points):
    n = points.shape[1].value
    return tf.reduce_sum(tf.reshape(coeffs, [-1,n,1]) * points, axis=1)

def _head(coeff_fn, config, args, offset):
    ref_traj = args['ref_traj']
    coeffs, weights = coeff_fn(config, args, offset)
    coeffs = tf.identity(coeffs, name='coeffs')
    return _linear_combo(coeffs, ref_traj), weights

def fixed_distance_attention(config, args):
    return _head(_fixed_distance_coeffs, config, args, 0)

def distance_attention(config, args):
    return _head(_distance_coeffs, config, args, 0)

def distance_attention_offset(config, args):
    offset = 5 if 'offset' not in config else config['offset']
    attended0, weights0 = _head(_distance_coeffs, config, args, 0)
    attendedk, weightsk = _head(_distance_coeffs, config, args, offset)
    return tf.concat([attended0, attendedk], 1), weights0 + weightsk

def centering_attention(config, args):
    T = config['T']
    state_dependent = config['state_dependent']
    temperature = config['temperature']
    ref_traj = args['ref_traj']
    ee_pos = args['ee_pos']
    state = args['state']
    centered_traj = ref_traj - tf.reshape(ee_pos, [-1,1,9])
    centered_traj = tf.identity(centered_traj, name='centered_traj')
    sizes = config['hidden_attention'] + [T]
    with tf.variable_scope('attention'):
        if state_dependent:
            mlp_out, mlp_weights, mlp_biases = get_mlp_layers(state, len(sizes), sizes)
            coeffs, weights = tf.nn.softmax(mlp_out / temperature), mlp_weights + mlp_biases
        else:
            w = tf.get_variable('w', shape=[T])
            coeffs, weights = tf.reshape(tf.nn.softmax(w / temperature), [1,T]), [w]
    coeffs = tf.identity(coeffs, name='coeffs')
    attended = _linear_combo(coeffs, centered_traj)
    return attended, weights

def time_fixed_attention(config, args):
    T, k, temperature = config['T'], config['time_k'], config['temperature']
    ref_traj, ee_pos, t = args['ref_traj'], args['ee_pos'], args['t']
    centered_traj = ref_traj - tf.reshape(ee_pos, [-1,1,9])
    centered_traj = tf.identity(centered_traj, name='centered_traj')
    centered_flat = tf.reshape(centered_traj, [-1, 9*T])

    t_range = tf.reshape(t, [-1, 1]) + tf.range(-k, k+1)
    t_range_clipped = tf.clip_by_value(t_range, 0, T-1)
    n_range = 2*k + 1

    def f(ref_and_idx):
        ref = tf.reshape(ref_and_idx[:-n_range], [T,9])
        idx = tf.cast(ref_and_idx[-n_range:], tf.int32)
        return tf.gather(ref, idx, axis=0)
    map_in = tf.concat([centered_flat, tf.cast(t_range_clipped, tf.float32)], axis=1)
    ref_range = tf.map_fn(f, map_in)
    ref_range = tf.identity(ref_range, name='ref_range')

    with tf.variable_scope('attention'):
        w = tf.get_variable('w', shape=[n_range])
        coeffs, weights = tf.reshape(tf.nn.softmax(w / temperature), [1, n_range]), [w]
    coeffs = tf.identity(coeffs, name='coeffs')
    attended = _linear_combo(coeffs, ref_range)
    return attended, []

def batch_flatten(input):
    return tf.reshape(input, [-1, int(np.prod(input.shape[1:]))])

def time_attention(config, args):
    T, k, temperature = config['T'], config['time_k'], config['temperature']
    ref_flat, ee_pos, ee_vel, t = args['ref_flat'], args['ee_pos'], args['ee_vel'], args['t']

    t_range = tf.reshape(t, [-1, 1]) + tf.range(-k, k+1)
    t_range_clipped = tf.clip_by_value(t_range, 0, T-1)
    n_range = 2*k + 1
    def f(ref_and_idx):
        ref = tf.reshape(ref_and_idx[:-n_range], [T,9])
        idx = tf.cast(ref_and_idx[-n_range:], tf.int32)
        return tf.gather(ref, idx, axis=0)
    map_in = tf.concat([ref_flat, tf.cast(t_range_clipped, tf.float32)], axis=1)
    ref_range = tf.map_fn(f, map_in)
    ref_range = tf.identity(ref_range, name='ref_range')

    centered_range = ref_range - tf.reshape(ee_pos, [-1,1,9])
    centered_range = tf.identity(centered_range, name='centered_range')
    centered_flat = tf.reshape(centered_range, [-1, 9*n_range])

    mlp_in = tf.concat([centered_flat, ee_vel], 1)
    mlp_sizes = config['hidden_attention'] + [n_range]
    with tf.variable_scope('attention'):
        mlp_out, mlp_weights, mlp_biases = get_mlp_layers(mlp_in, len(mlp_sizes), mlp_sizes, nonlinear_output=False)
    coeffs, weights = tf.nn.softmax(mlp_out / temperature), mlp_weights + mlp_biases
    attended = _linear_combo(coeffs, centered_range)
    return attended, weights

def _fc_layer(input, size, id, nonlinearity=tf.nn.relu):
    sofar = input
    in_shape = sofar.get_shape().dims[1].value
    cur_weight = init_weights([in_shape, size], name='w_' + str(id))
    cur_bias = init_bias([size], name='b_' + str(id))
    sofar = tf.nn.xw_plus_b(sofar, cur_weight, cur_bias)
    sofar = nonlinearity(sofar) if nonlinearity is not None else sofar
    return sofar, [cur_weight], [cur_bias]

def factored_mlp_structure(config, args):
    dim_output = args['dim_output']
    state, attended = args['state'], args['attended']
    with tf.variable_scope('structure'):
        mlp_sizes = config['mlp_hidden_sizes'] + [dim_output]
        with tf.variable_scope('state'):
            state_mlp_out, state_weights, state_biases = get_mlp_layers(state, len(mlp_sizes), mlp_sizes, nonlinear_output=False)
        with tf.variable_scope('attended'):
            attended_mlp_out, attended_weights, attended_biases = get_mlp_layers(attended, len(mlp_sizes), mlp_sizes, nonlinear_output=False)
    state_mlp_out = tf.identity(state_mlp_out, 'state_part')
    attended_mlp_out = tf.identity(attended_mlp_out, 'attn_part')
    final_out = state_mlp_out + attended_mlp_out
    weights = state_weights + attended_weights
    biases = state_biases + attended_biases
    reg = config['regularization'] * tf.nn.l2_loss(state_mlp_out)
    return final_out, weights + biases, [reg]

def mlp_structure(config, args):
    dim_output = args['dim_output']
    state, attended = args['state'], args['attended']
    augmented_state = tf.concat([state, attended], 1) if attended is not None else state
    with tf.variable_scope('structure'):
        mlp_sizes = config['mlp_hidden_sizes'] + [dim_output]
        mlp_out, weights, biases = get_mlp_layers(augmented_state, len(mlp_sizes), mlp_sizes, nonlinear_output=False)
    return mlp_out, weights + biases, []

def linear_structure(config, args):
    dim_output = args['dim_output']
    state, attended, ee_pos = args['state'], args['attended'], args['ee_pos']
    attention_direction = attended - ee_pos
    state_dim = attention_direction.shape[1].value
    with tf.variable_scope('structure'):
        K = tf.get_variable('K', [state_dim, dim_output])
    return tf.matmul(attention_direction, K), [K], []

def corrected_linear_structure(config, args):
    attention_direction = attended - ee_pos
    state_dim = attention_direction.shape[1].value
    with tf.variable_scope('structure'):
        K = tf.get_variable('K', [state_dim, dim_output])
        mlp_sizes = config['mlp_hidden_sizes'] + [dim_output]
        with tf.variable_scope('mlp'):
            mlp_out, mlp_weights, mlp_biases = get_mlp_layers(state, len(mlp_sizes), mlp_sizes)
    linear_term = tf.matmul(attention_direction, K)
    correction = mlp_out
    reg = config['regularization'] * tf.nn.l2_loss(correction)
    return linear_term + correction, [K] + mlp_weights + mlp_biases, [reg]

def ref_traj_network_factory(dim_input=27, dim_output=7, batch_size=25, network_config=None):
    config = network_config
    T = config['T']
    attention = config['attention']
    structure = config['structure']
    ee_pos_indices = config['ee_pos_indices']
    ee_vel_indices = config['ee_vel_indices']
    assert ee_pos_indices[1] - ee_pos_indices[0] == 9
    assert ee_vel_indices[1] - ee_vel_indices[0] == 9
    nn_input, action, precision = get_input_layer(dim_input, dim_output)
    state = nn_input[:,:-(9*T+1)]
    non_state = nn_input[:,-(9*T+1):]
    t = tf.cast(non_state[:,-1], tf.int32, name='t')
    ref_flat = non_state[:,:-1]
    ref_traj = tf.reshape(ref_flat, [-1,T,9])
    ee_pos = nn_input[:,ee_pos_indices[0]:ee_pos_indices[1]]
    ee_pos = tf.identity(ee_pos, name='ee_pos')
    ee_vel = nn_input[:,ee_vel_indices[0]:ee_vel_indices[1]]
    ee_vel = tf.identity(ee_vel, name='ee_vel')

    args = {
        'state': state,
        'non_state': non_state,
        't': t,
        'ref_flat': ref_flat,
        'ref_traj': ref_traj,
        'ee_pos': ee_pos,
        'ee_vel': ee_vel,
    }

    if attention:
        attended, attn_weights = attention(config, args)
        attended = tf.identity(attended, name='attended')
    else:
        attended, attn_weights = None, []

    args['attended'] = attended
    args['dim_output'] = dim_output

    final_out, structure_weights, extra_loss_terms = structure(config, args)
    all_vars = attn_weights + structure_weights
    loss_out = get_loss_layer(mlp_out=final_out, action=action, precision=precision, batch_size=batch_size)
    for term in extra_loss_terms:
        loss_out = loss_out + term

    return TfMap.init_from_lists([nn_input, action, precision], [final_out], [loss_out]), all_vars, []
