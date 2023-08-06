function params = init_params()
% Initialize a struct containing the body lengths in meters.
params = struct;

% Main parameters
params.l01 = 0.8;
params.l02 = 0.25;
params.l03 = 0.2;
params.l04 = 0.1;
params.l11 = 0.45;
params.l21 = 0.3;
params.l22 = 0.15;
params.alpha = pi / 3;

% Initialize a random vector of joint positions.
% q = rand(3,1);

%% Link properties
R = 0.05; % link radius
params.m = cell(4,1);
params.k_r_ks = cell(4,1); 
params.k_I_s = cell(4,1);

% base
params.m{1} = 4.0;
params.k_r_ks{1} = [0.0; 0.0; 0.0];
params.k_I_s{1} = diag([params.m{1}*(params.l01^2 + params.l02^2)/12.0, ...
                        params.m{1}*(params.l02^2 + params.l02^2)/12.0, ...
                        params.m{1}*(params.l01^2 + params.l02^2)/12.0]);
% link1
params.m{2} = 1.0;
params.k_r_ks{2} = [0.0; 0.0; 0.5 * params.l11];
params.k_I_s{2} = diag([params.m{2}*(3 * R^2 + 4 * params.l11^2)/12.0, ...
                        params.m{2}*(3 * R^2 + 4 * params.l11^2)/12.0, ...
                        params.m{2}*R^2/2.0]);
% link2 
params.m{3} = 1.0;
params.k_r_ks{3} = [0.0; 0.0; 0.5 * params.l21];
params.k_I_s{3} = diag([params.m{2}*(3 * R^2 + 4 * params.l21^2)/12.0, ...
                        params.m{2}*(3 * R^2 + 4 * params.l21^2)/12.0, ...
                        params.m{2}*R^2/2.0]);

% link3 (ignore sensor weight)
params.m{4} = 1.0;
params.k_r_ks{4} = [0.0; 0.0; 0.0];
params.k_I_s{4} = diag([params.m{4}*(params.l22^2)*2.0/3.0, ...
                        params.m{4}*(params.l22^2)*2.0/3.0, ...
                        params.m{4}*(params.l22^2)*2.0/3.0]);
%% Gravity
params.I_g_acc = [0; 0; -9.81];

%% Gains
params.kp_joint = diag([50.0 6.0 6.0]);
params.kd_joint =  diag([30.0 2.5 2.5]);

params.kp_task = diag([55.0 55.0 55.0 60.0 20.0 20.0]);
params.kd_task = diag([5.0 5.0 5.0 10.0 4.0 4.0]);

params.tau_max = 2000.0;

%% Slope
params.slope_angle = pi/20.0;

%% Wall
params.wall_y = 1.4;
params.mu_s = 0.6;
params.mu_k = 0.6;

%% Simulation params
params.control_dt = 0.01;       % Control and visualization steps
params.N_sim_decimation = 2;    % Amount simulation steps called between control updates

% Sampling time used for discrete integration steps
params.simulation_dt = params.control_dt / params.N_sim_decimation; 


end