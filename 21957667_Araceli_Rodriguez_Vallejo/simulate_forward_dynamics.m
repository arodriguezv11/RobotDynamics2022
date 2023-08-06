clear; clc; close all;

init_workspace

%% Setup
use_solution = 0; % Use solution (1) or user implementation (0)

% generalized coordinates
gc = generate_gc;

% Initialize the parameters for the mid-term exam.
params = init_params;

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

% Simulation
T_sim = 3.0;
N_sim = round(T_sim / params.control_dt);

%% Gravity Compensation
disp('Forward dynamics...');
gc.q = [0.1; pi/3; pi/4];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

figure(1);

for sim_step = 1:N_sim
   %% Visualize
   gc =  draw_robot(gc, kin, params);
   drawnow()
   
   %% Simulator Loop
   for j = 1:params.N_sim_decimation
      if use_solution == 1
          [gc, ~] = Q2_forward_dynamics_solution(gc, tau, params);
      else
          [gc, ~] = Q2_forward_dynamics(gc, tau, params);
      end
   end
   pause(params.control_dt);
   refresh
end