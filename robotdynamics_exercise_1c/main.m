
% Generate a joint state vector (array)
q = pi()./6*ones(6,1);

% End-effector position vector iR_IE and rotation matrix C_IE
iR_IE = jointToPosition(q)
C_IE = jointToRotMat(q)

% Conversion rotation matrix to quaternions
quat = rotMatToQuat(C_IE)

% Conversion quaternions to rotation matrix
C_IE_q = quatToRotMat(quat)

% Provide random joint state to the ABB IB120 object
abbRobot.setJointPositions(q);
