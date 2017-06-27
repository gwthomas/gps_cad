import pdb
import tensorflow as tf

from gps.algorithm.policy_opt.tf_model_example import *

def ref_traj_network(dim_input=27, dim_output=7, batch_size=25, network_config=None):
    """
    Specifying a fully-connected network in TensorFlow.

    Args:
        dim_input: Dimensionality of input.
        dim_output: Dimensionality of the output.
        batch_size: Batch size.
        network_config: dictionary of network structure parameters
    Returns:
        a TfMap object used to serialize, inputs, outputs, and loss.
    """
    T = network_config['T']
    ee_pos_indices = network_config['ee_pos_indices']
    scale = network_config['scale']
    assert ee_pos_indices[1] - ee_pos_indices[0] == 9
    n_layers = 2 if 'n_layers' not in network_config else network_config['n_layers'] + 1
    dim_hidden = (n_layers - 1) * [40] if 'dim_hidden' not in network_config else network_config['dim_hidden']
    dim_hidden.append(dim_output)

    nn_input, action, precision = get_input_layer(dim_input, dim_output)
    state = nn_input[:,:-9*T]
    ref_traj = tf.reshape(nn_input[:,-9*T:], [-1,T,9], 'ref_traj')
    ee_pos = nn_input[:,ee_pos_indices[0]:ee_pos_indices[1]]
    ee_pos = tf.identity(ee_pos, name='ee_pos')

    reshaped = tf.reshape(ee_pos, [-1,1,9], name='reshaped')
    distances = tf.norm(ref_traj - reshaped, axis=2)
    distances = tf.identity(distances, name='distances')
    weights = tf.nn.softmax(-distances * scale, name='weights')
    attended_points = tf.reduce_sum(tf.reshape(weights, [-1,T,1]) * ref_traj, axis=1, name='attended')
    augmented_state = tf.concat([state, attended_points], 1, name='augmented')
    mlp_applied, weights_FC, biases_FC = get_mlp_layers(augmented_state, n_layers, dim_hidden)

    all_vars = weights_FC + biases_FC
    loss_out = get_loss_layer(mlp_out=mlp_applied, action=action, precision=precision, batch_size=batch_size)

    return TfMap.init_from_lists([nn_input, action, precision], [mlp_applied], [loss_out]), all_vars, []
