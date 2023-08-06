function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor. 
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m,n] = size(A);
% TODO: complete the computation of the pseudo-inverse.
% Hint: How should we account for both left and right pseudo-inverse forms?

if (m>n)
    % Compute the left pseudoinverse.
    pinvA = (A'*A + lambda*lambda*eye(n,n))\A';
elseif (n<m)
    % Compute the right pseudoinverse.
    pinvA = A'/(A*A' + lambda*lambda*eye(m,m));
end
end
