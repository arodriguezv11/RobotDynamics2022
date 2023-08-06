%% Generalized coordinates
function gc = generate_gc()

    % generate generalized coordinate symbolic variables
    syms q0 q1 q2 'real';
    syms dq0 dq1 dq2 'real';

    % Return struct with generalized coordinates and velocities
    gc.q = [q0 q1 q2]';
    gc.dq = [dq0 dq1 dq2]';

end