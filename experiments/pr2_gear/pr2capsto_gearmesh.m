% Script for training PR2 with capsules using trajectory optimization and
% settings that are as close as possible to those used on the real PR2
% (good luck with that...).
numIter = 100;%70;
% Run PR2-based insertion experiment in simulation.
options = mjpr2capsoptions(struct('append_pts_vel',9));
options.filename = 'pr2/pr2_arm3d_gearmesh.xml';

% Now set up some options that are specific for this test.

% Set up noise smoothing.
options.smooth_noise = 1;
options.smooth_noise_sigma = 2.0;
options.smooth_noise_renormalize = 1;

% Trajectory length and frequency.
options.T = 100;
options.dt = 1/50;%1/20;
options.substeps = options.dt/0.001; %options.dt/0.01;

% Set up state space configuration.
options.append_prev_state = 0;
options.append_pts_vel = 9; % There are three points to track -> 2 x 3 = 6 points to append to the state.
% options.append_pts_vel = 6; % There are two points to track -> 2 x 3 = 6 points to append to the state.

% Create environment.
environment = SimulatorMJ(options);

% Initialize cost function.
cost_options = struct();

% This must be set to zero if we are not running under EnvironmentROS.
cost_options.env_target = 0;

% Target weights and properties.
% cost_options.tgt = [0.0; 0.3; -0.5;  0.0; 0.3; -0.07];
tgt_center = repmat([-0.11; 0.41; -0.36],[3,1]);
cost_options.tgt = tgt_center+[0; 0; 0;  0; 0; -0.05;  0; 0; 0.05];
% tgt_center = repmat([-0.27; 0.25; -0.36],[3,1]);
% cost_options.tgt = tgt_center+[0.05; 0; 0;  -0.05; 0; 0;  0; -0.05; 0];

cost_options.cost_ramp = 'constant';
cost_options.wp_final_multiplier = 1.0;
cost_options.wp = ones(size(cost_options.tgt));
cost_options.penalty_type = 'joint';
cost_options.norm = 'logl2';
cost_options.alpha = 1e-5;
cost_options.l1 = 15.0;%.1;
cost_options.l2 = 10.0;

% Joint weights.
gains = (1.0)./[3.09;1.08;0.393;0.674;0.111;0.152;0.098];

% Torque and velocity penalties.
cost_options.wu = 1e-3;

cost_options.analytic_jacobian = 1;

% Create base cost function.
cost1 = CostFK(cost_options);

% Create second cost function for last step only.
cost_options.wp_final_multiplier = 10.0;
cost_options.cost_ramp = 'final_only';
cost_options.l1 = 1.0;
cost_options.l2 = 0.0;
cost_options.wu = 0;
cost2 = CostFK(cost_options);

% Create final sum cost.
cost = {CostSum(struct(),{cost1,cost2},[1 1])};

% Set up algorithm parameters for LQR-based trajectory optimization.
options = struct();

% Choose number of iterations.
options.iterations = numIter;

% Set up sample counts.
options.adaptive_sampling = 1;
options.max_samples = 5;
options.min_samples = 2;
options.samples = 4;

% Set up cost prior.
if ~cost_options.analytic_jacobian,
    options.costprior = @(dpp)CostPriorGMM(dpp);
    options.costpriorparams = struct('max_clusters',40,...
        'min_samples_per_cluster',40,'max_samples',20,...
        'fast_gmm',0);
end;

% Set up dynamics prior.
options.dynprior = @(dpp)DynPriorGMM(dpp);
options.dynpriorparams = struct('max_clusters',40,...
    'min_samples_per_cluster',40,'max_samples',20,...
    'fast_gmm',0);

% Set up step rate.
options.kl_step = 0.5;%1.0;
options.max_step_mult = 10.0;
options.min_step_mult = 0.01;
options.sample_decrease_var = 0.05;
options.sample_increase_var = 0.1;

% Set up initialization.
options.init_var = 5.0;
options.init_stiffness = 1.0;
options.init_stiffness_vel = 0.5;
options.init_final_weight = 50.0;
options.init_gains = gains;
options.init_type = 'lqr';

% Set up policy optimization parameters (this is unused unless we switch to
% TrajOptCGPS).
options.policy_max_samples = 20;
options.policy_sample_mode = 'replace'; % Don't keep old samples from prior iterations.
options.lagrange_step_schedule = [1e-2 1e-2 1e-1 1]; % Schedule for policy constraint dual vars.
options.init_polwt = 0.1; % Initial policy constraint dual vars.
options.ent_reg_schedule = [1e-3 1e-2 1e-1 1]; % Entropy minimizing regular weight.
options.policyparams.ent_reg_type = 1;
options.inner_iterations = 4; % How many times to alternate LQR solve with NN training per outer iteration.

% Number of LBFGS iterations for neural network training each time.
options.policyparams.lbfgs_iter = 200;
options.policyparams.init_lbfgs_iter = 200;
options.policyparams.init_inner_lbfgs_iter = 200;

% Set up two layer network.
options.policyparams.unit_count = [40, 40];
options.policyparams.layer_init = repmat({@(e,p,i,o)Layer(e,p,i,o)},[1 3]);
options.policyparams.layer_params = {struct(),struct(),struct('link',0)};

% Normalize inputs and outputs, but don't whiten (it helps a little, not
% much).
options.policyparams.normalize = 1;
options.policyparams.whiten = 0;

% Take 50 synthetic samples each time.
options.policy_synthetic_samples = 50;
options.policy_synthetic_samples_distro_fix_bound = 2.0;

% Set up debugging properties.
options.fid_debug = NaN;
options.fid_detail = NaN;
options.debug_filename = '';
options.detail_filename = '';

% Create optimization algorithm.
algorithm = TrajOptLQR(environment,options);
%algorithm = TrajOptCGPS(environment,options);

% Set up the experiment.
options = struct();
options.name = '/home/aviv/Data/Gears/pr2lqr';
options.save_state = 0;
experiment = Experiment(environment,cost,algorithm,options);

% Run the experiment.
tic;
[controllers,experiment] = experiment.run_training();
fprintf(1,'Total training time: %f\n',toc);

% Perform a single trial to observe the result.
experiment.run_test(controllers{1});
