% generate equations of motion
function eom = Q1_generate_eom(gc, kin, params, jac)
% By calling:
%   eom = generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion.
%
% Inputs:
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : Struct containing symbolic expresses for the kinematics
%   - params    : Struct with parameters
%   - jac       : Struct containing symbolic expresses for the jacobians
%   
% Output:
%   - eom       : Struct with fields {M, b, g}, implementing the system dynamics
%

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (3x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (3x1 cell)->(3x3 sym)

k_I_s = params.k_I_s;      % Inertia tensor of body k in frame k (3x1 cell)->(3x3 sym)
m = params.m;              % Mass of body k (3x1 cell)->(1x1 double)
I_g_acc = params.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = params.k_r_ks;    % CoM location of body k in frame k (3x1 cell)->(3x1 double)

I_J_pos = jac.I_Jp;    % CoM Positional Jacobian in frame I (3x1 cell)->(3x6 sym)
I_J_rot = jac.I_Jr;        % CoM Rotational Jacobian in frame I (3x1 cell)->(3x6 sym)

eom.M = sym(zeros(3,3));
eom.g = sym(zeros(3,1));
eom.b = sym(zeros(3,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(3,3));

% TODO: Implement here
for k = 1:length(q)
    M = M + m{k}*I_J_pos{k}'*I_J_pos{k} ...
          + I_J_rot{k}'*R_Ik{k}*k_I_s{k}*R_Ik{k}'*I_J_rot{k};
end

M = simplify(M);
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(3,1));

% TODO: Implement here
g = sym(zeros(3,1));
for k=1:length(q)
    g = g - m{k}* I_J_pos{k}'* I_g_acc;
end

g = simplify(g); % Allow more time for more simplified solution
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(3,1));

% TODO: Implement here

for k=1:length(q)
    fprintf('b%i... ',k);
    dJp_s = simplify(dAdt(jac.I_Jp{k},gc.q, gc.dq));
    dJr_s = simplify(dAdt(jac.I_Jr{k},gc.q, gc.dq));
    omega_i = simplify(jac.I_Jr{k}*gc.dq);
    I_sk = simplify(R_Ik{k} * k_I_s{k} * R_Ik{k}');
        
    b = b + simplify(I_J_pos{k}' * m{k} * dJp_s * dq) + ...
            simplify(I_J_rot{k}' * I_sk * dJr_s * dq) + ...
            simplify(I_J_rot{k}' * cross( omega_i , I_sk * omega_i));
end

b = simplify(b);
fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end
