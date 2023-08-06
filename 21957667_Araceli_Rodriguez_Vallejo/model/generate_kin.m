% generate kinematics
function kin = generate_kin(q, params)
%% Create kinematics container
kin = struct();

% homogeneous transformations from frame k to the inertial frame
kin.T_Ik = cell(3,1);

% rotation matrices from frame k to the inertial frame
kin.R_Ik = cell(3,1);

%% Homogeneous transformations

% kinematic params
l01 = params.l01;
l02 = params.l02;
l03 = params.l03;
l11 = params.l11;
l21 = params.l21;
l22 = params.l22;

% Joint positions
q0 = q(1);
q1 = q(2);
q2 = q(3);

% compute T_IB
p_IB_I = [0; q0; l02/2.0];
C_IB = [1 0 0;
        0 1 0;
        0 0 1];

T_IB = [C_IB p_IB_I;
        0 0 0 1];

% Add slope
a = params.slope_angle;
C_IB = [1, 0, 0;
     0, cos(a), sin(a);
     0, -sin(a), cos(a)];
p_IB_I = C_IB * p_IB_I;
T_IB = [C_IB p_IB_I;
        0 0 0 1];

% base frame to link 1
p_B1_B = [0; l03; l02/2.0];
C_B1 = [1, 0, 0;
    0, cos(q1), sin(q1);
    0, -sin(q1), cos(q1)];
T_B1 = [C_B1 p_B1_B;
    zeros(1,3), 1];

% link 1 to link 2
p_12_1 = [0;0; l11];
C_12 = [1, 0, 0;
    0, cos(q2), sin(q2);
    0, -sin(q2), cos(q2)];
T_12 = [C_12 p_12_1;
    zeros(1,3), 1];

% link 2 to gripper head
p_2G0_2 = [0; 0; l21];
C_2G0 = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
T_2G0 = [C_2G0, p_2G0_2;
    zeros(1,3), 1];

% gripper head to gripper tail
p_G01_G0 = [0; l22; 0];
C_G01 = [1, 0, 0;
    0, 0, 1;
    0, -1, 0];
T_G01 = [C_G01, p_G01_G0;
    zeros(1,3), 1];

T_2G = T_2G0 * T_G01;

% final: inertia to gripper

% homogeneous transformations from frame k to frame I
kin.T_IB = T_IB;
kin.T_12 = T_12;
kin.T_2G = T_2G;
kin.T_Ik{1} = simplify(T_IB);
kin.T_Ik{2} = simplify(kin.T_Ik{1}*T_B1);
kin.T_Ik{3} = simplify(kin.T_Ik{2}*T_12);
kin.T_Ik{4} = simplify(kin.T_Ik{3}*T_2G);

% rotation matrices from frame k to frame I
kin.R_Ik{1} = kin.T_Ik{1}(1:3,1:3);
kin.R_Ik{2} = kin.T_Ik{2}(1:3,1:3);
kin.R_Ik{3} = kin.T_Ik{3}(1:3,1:3);
kin.R_Ik{4} = kin.T_Ik{4}(1:3,1:3);

%% Endeffector
% end-effector homogeneous transformation and position
kin.T_IG = T_IB * T_B1 * T_12 * T_2G0 * T_G01;
kin.R_IG =  kin.T_IG(1:3,1:3);
kin.I_r_IG = kin.T_IG(1:3,4);

% constant tranformations related to the base
kin.T_B1 = [eye(3,3), [0;-0.5*l01;-0.5*l02];
             zeros(1,3), 1];
kin.T_B2 = [eye(3,3), [0;-0.5*l01;0.5*l02];
             zeros(1,3), 1];
kin.T_B3 = [eye(3,3), [0;0.5*l01;0.5*l02];
             zeros(1,3), 1];
kin.T_B4 = [eye(3,3), [0;0.5*l01;-0.5*l02];
             zeros(1,3), 1];


% constant tranformations related to the gripper
lg = 0.05;
kin.T_2G0 = T_2G0;
kin.T_G01 = T_G01;
kin.T_GG1 = [eye(3,3), [0;lg;0];
             zeros(1,3), 1];
    
kin.T_GG2 = [eye(3,3), [0;lg;lg];
             zeros(1,3), 1];
         
kin.T_GG3 = [eye(3,3), [0;-lg;0];
             zeros(1,3), 1];
         
kin.T_GG4 = [eye(3,3), [0;-lg;lg];
             zeros(1,3), 1];


%% Matlab functions
% fname = mfilename;
% fpath = mfilename('fullpath');
% dpath = strrep(fpath, fname, '');
% 
% fprintf('Generating transformation file... ');
% matlabFunction(kin.T_IG, 'vars', {q}, 'file', strcat(dpath,'/T_IG_fun'));
% fprintf('done!\n')

end
