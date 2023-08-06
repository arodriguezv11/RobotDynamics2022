function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  
  % Extract the scalar part.
  q_w = q(1);
  
  % Extract the vector part.
  q_n = q(2:4);
  
  % Map the unit quaternion to a rotation matrix.
  R = (2*q_w^2-1)*eye(3) + 2.0*q_w*skewMatrix(q_n) + 2.0*(q_n*q_n');

function A = skewMatrix(q_n)
    A = [0, -q_n(3), q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end
end
