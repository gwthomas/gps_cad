import tensorflow as tf
from gps.algorithm.policy_opt.tf_utils import TfMap
from gps.algorithm.policy_opt.tf_model_example import *


def traj_tf_network(dim_input=27, dim_output=7, batch_size=25, network_config=None):
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
    n_layers = 2 if 'n_layers' not in network_config else network_config['n_layers'] + 1
    dim_hidden = (n_layers - 1) * [40] if 'dim_hidden' not in network_config else network_config['dim_hidden']
    dim_hidden.append(dim_output)

    nn_input, action, precision = get_input_layer(dim_input, dim_output)
    mlp_applied, weights_FC, biases_FC = get_mlp_layers(nn_input, n_layers, dim_hidden)
    mlp_out = tf.identity(mlp_applied, name='nn_output')
    fc_vars = weights_FC + biases_FC
    loss_out = get_loss_layer(mlp_out=mlp_out, action=action, precision=precision, batch_size=batch_size)

    return TfMap.init_from_lists([nn_input, action, precision], [mlp_out], [loss_out]), fc_vars, []
