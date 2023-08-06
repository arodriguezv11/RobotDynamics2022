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
T_sim = 5.0;
N_sim = round(T_sim / params.control_dt);

%% Gravity Compensation
disp('Gravity Compensation...');
gc.q = [0.5; pi/4; -pi/6];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

% reference
% q_des = [0.6; pi/4; -pi/10]; 
% dq_des = [0.0; 0.0; 0.0];

target_C = [1.0, 0.0, 0.0;
            0.0, 0.0, 1.0;
            0.0, -1.0, 0.0];
target_Fy = 2.0;
figure(1);


for sim_step = 1:N_sim

   target_p = [0.0; params.wall_y; 0.5 + 0.2 * sin(sim_step * 4*pi / N_sim)];
   target_v = [0.0; 0.0; 0.2 * 4*pi / N_sim / params.control_dt * cos(sim_step * 4*pi / N_sim)];
   
   [tau_wall, force, mode] = simulate_reaction_force(gc, kin, params);
   %% control input
   if use_solution == 1
   	tau = Q4_task_space_control_solution(params, gc, kin, target_p, target_v, target_C, target_Fy);
   else
    tau = Q4_task_space_control(params, gc, kin, target_p, target_v, target_C, target_Fy); 
   end
   tau = tau + tau_wall;
   %% Visualize
   plot(target_p(2), target_p(3), 'ro', 'MarkerSize', 10);
   hold on; 
   gc =  draw_robot(gc, kin, params);
   xline(params.wall_y);

   drawnow()
   
   %% Simulator Loop
   for j = 1:params.N_sim_decimation
      [gc, ~] = Q2_forward_dynamics_solution(gc, tau, params); 
   end

   pause(params.control_dt);
   refresh
end

