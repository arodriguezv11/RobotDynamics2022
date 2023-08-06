function [tau, force, mode] = simulate_reaction_force(gc, kin, params)

mode = 0;
tau = zeros(3,1);
force = zeros(3,1);
% position = I_r_IF_fun(q);
% position = kin.I_r_IG
q = gc.q;
q_dot = gc.dq;
position = eval(subs(kin.I_r_IG, {'q0' 'q1' 'q2'}, {gc.q(1) gc.q(2) gc.q(3)}));
I_JpG = I_Jp_G_fun(q); % Position Jacobian of the end effector (3x3)
I_JrG = I_Jr_G_fun(q); % Rotational Jacobian of the end effector (3x3)
I_JG = [I_JpG; I_JrG];

velocity = I_JG * q_dot;

z_force = zeros(3,1);
y_force = zeros(3,1);

if(position(2) > params.wall_y) % in contact
    % z dir
    depth = abs(params.wall_y - position(2));
    y_force = -500.0 * depth * [0.0; 1.0; 0.0];
%     if(velocity(2) > 0.0)
%         y_force = y_force - 200.0 * velocity(2) * [0.0; 1.0; 0.0];
%     end
    y_force = y_force - 20.0 * velocity(2) * [0.0; 1.0; 0.0];

    
    % y dir is 0.
    f_s = params.mu_s * y_force(2);
    f_k = params.mu_k * y_force(2);
%     if(abs(interaction_force(2)) <= params.mu_s * abs(interaction_force(3)))
%         mode = 1;
%         z_force = - 100.0 * velocity(3) * [0.0; 1.0; 0.0] - interaction_force(3) * [0.0; 1.0; 0.0];
%     else
    z_force = f_k * sign(velocity(3)) * [0.0; 0.0; 1.0];
%     end
    force = z_force + y_force;
%     force = y_force;
    tau = I_JG(1:3,1:3)'*(force);
end

end