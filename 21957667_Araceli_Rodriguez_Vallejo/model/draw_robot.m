function gc_new = draw_robot(gc, kin, params)

% Compute kinematicss
T_IB = eval(subs(kin.T_IB, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_I1 = eval(subs(kin.T_Ik{2}, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_I2 = eval(subs(kin.T_Ik{3}, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_IG0 = eval(subs(kin.T_Ik{3} * kin.T_2G0, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_IG = eval(subs(kin.T_IG, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
% T_IK_3 = eval(subs(kin.T_Ik{3}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
% I_r_IG = eval(subs(kin.I_r_IG, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));

% p_3Croot_3 = [params.l31; 0; 0];
% T_3Croot = [eye(3) p_3Croot_3;
%         0 0 0 1];
% T_ICroot = T_IK_3 * T_3Croot;   
I_p_IB = T_IB(2:3,4);
I_p_I1 = T_I1(2:3,4);
I_p_I2 = T_I2(2:3,4);
I_p_IG0 = T_IG0(2:3,4);
I_p_IG = T_IG(2:3,4);
l01 = params.l01;
l02= params.l02;
l03 = params.l03;

l11 = params.l11;
l21 = params.l21;


% compute additional positions to visualize base
T_IB1 = T_IB * kin.T_B1;
I_p_IB1 = T_IB1(2:3, 4);

T_IB2 = T_IB * kin.T_B2;
I_p_IB2 = T_IB2(2:3, 4);

T_IB3 = T_IB * kin.T_B3;
I_p_IB3 = T_IB3(2:3, 4);

T_IB4 = T_IB * kin.T_B4;
I_p_IB4 = T_IB4(2:3, 4);

base_x = [I_p_IB1(1) I_p_IB2(1) I_p_IB3(1) I_p_IB4(1) I_p_IB1(1)];
base_y = [I_p_IB1(2) I_p_IB2(2) I_p_IB3(2) I_p_IB4(2) I_p_IB1(2)];


% compute additional positions to visualize gripper
T_IG1 = T_IG * kin.T_GG1;
I_p_IG1 = T_IG1(2:3, 4);

T_IG2 = T_IG * kin.T_GG2;
I_p_IG2 = T_IG2(2:3, 4);

T_IG3 = T_IG * kin.T_GG3;
I_p_IG3 = T_IG3(2:3, 4);

T_IG4 = T_IG * kin.T_GG4;
I_p_IG4 = T_IG4(2:3, 4);

plot(base_x, base_y, 'b', ...
           [I_p_I1(1) I_p_I2(1)], [I_p_I1(2) I_p_I2(2)], 'r', ...
           [I_p_I2(1) I_p_IG0(1)], [I_p_I2(2) I_p_IG0(2)], 'g', ...
           [I_p_IG0(1) I_p_IG(1)], [I_p_IG0(2) I_p_IG(2)], 'b', ...
           [I_p_IG(1) I_p_IG1(1)], [I_p_IG(2) I_p_IG1(2)], 'b', ...
           [I_p_IG1(1) I_p_IG2(1)], [I_p_IG1(2) I_p_IG2(2)], 'b', ...
           [I_p_IG(1) I_p_IG3(1)], [I_p_IG(2) I_p_IG3(2)], 'b', ...
           [I_p_IG3(1) I_p_IG4(1)], [I_p_IG3(2) I_p_IG4(2)], 'b','LineWidth', 1.5);

axis([-0.5, 2.5, -1.0, 1.0]);
daspect([1 1 1])

hold off

gc_new = gc;
end