
sample_time = 0.001;    % sec
DEG2RAD = pi/180.0;
n = 2;      % Number of joints
p = 5;      % Number of parameters
simulation_time = 360.0;   % sec

%% Motor limits
motor_position_limits = DEG2RAD*[60.0, 60.0];   % degrees
motor_velocity_limits = [11.5, 11.5];   % rad/s
motor_saturation_torque = [10.0, 10.0];
velocity_filter_cutoff = 0.95;

%% Control gains
% % Supported controllers: {'cl', 'ef', 'df'}
% % Look at Controller Type Class
% % 1: cl (Integral Concurrent Learning)
% % 2: ef (Exponential Forgetting)
% % 3: df (Directional Forgetting)
% controller_type = 2;
% % Filter constant of the filtered tracking error: r = e_dot + alpha*e
% alpha = 5.0;
% % Feedback gain (used in the control law: tau = r_gain * r + Y2*theta_hat + e)
% % One entry per joint, assembled into a diagonal matrix.
% %  1 entry: Base Joint
% %  2 entry: Second Joint
% k1 = diag([3.0 1.5]);
% % Adaptive gain matrix diagonal vector
% Gamma = 0.05*eye(p);
% % Initial condition of adaptive estimates
% initial_theta_hat = zeros(p,1);
% % Finite integration window for open-loop terms (seconds).
% delta_t = 0.2;
% % Sigma-modification constant:
% %   theta_hat_dot += -sigma * Gamma * theta_hat
% % Set to 0.0 to disable sigma-modification.
% sigma = 0.0;
% 
% % % DF Controller Parameters
% % Sampling time of minimum eigenvalue computation
% svd_sampling_time = 0.1;
% % Minimum Eigenvalue of Information Matrix to stop learning
% % Once this threshold is passed, the DF component of the adaptive update
% % law gets frozen (i.e., learning is stopped)
% min_df_eigenval_threshold = 1.0;
% % Minimum Eigenvalue of Information Matrix to activate forgetting
% varepsilon = 0.005;
% % Directional forgetting correction weight in the update law:
% %   theta_hat_dot += -k_df * Gamma * (Omega*theta_hat - Lambda)
% % Set to 0.0 to disable the DF correction (degenerates to plain
% % gradient-based adaptive law).
% k_df = 25.0;
% % Directional forgetting matrix multiplier:
% %   F = mu * Omega * Y1_int' * D * Y1_int
% % Only active when forgetting_algorithm == 'directional'.
% % Operational range (0.0, 1.0)
% % Initial time-varying forgetting factor
% % mu = 0.9;
% % Constant forgetting factor
% mu = 5.0;
% % Exponential forgetting constant (lambda).
% % Only active when forgetting_algorithm == 'exponential'.
% % Must be strictly positive
% l = 0.6;
% % Regularized inverse constant
% %   R = (\mathcal{Y}\Omega\mathcal{Y}^\top + \eta\mathrm{I}_p)^{-1}
% % Only active when controllerType is directional.
% eta = 5e-3;
% % Alertness-based forgetting factor
% mu_max = 5.0;   % set mu_max = mu to disable the boost
% e_lo   = 0.06;   % [N m s] boost starts
% e_hi   = 0.12;   % [N m s] boost saturates
% tau_pe = 0.5;    % [s] prediction-error filter
% normalize_df = true;
% 
% % % CL Controller Parameters
% % Integral Concurrent Learning correction weight in the update law:
% %   theta_hat_dot += -k_cl * Gamma * history_stack
% % Set to 0.0 to disable the CL correction (degenerates to plain
% % gradient-based adaptive law).
% k_cl = 10.0;
% % Threshold of integral regressor norm ratio
% % Guarantees there is enough information in the current regressor before it
% % gets stored in the history stack
% epsilon = 0.05;
% % Threshold of the minimum eigenvalue of the history stack
% % The CL part of the update law gets frozen when this threshold is reached
% min_cl_eigenval_threshold = 1.0;
% % acceptance threshold on the resulting sigma_min(history_stack). 
% % In the rolling phase a swap is committed only if it keeps sigma_min at or above
% % this value.
% min_sv_update_threshold = 0.01;
% % Size of the history stack
% stack_size = 20;

