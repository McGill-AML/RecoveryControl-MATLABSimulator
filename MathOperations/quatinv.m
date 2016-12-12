function [ q_inv ] = quatinv( q )
% Returns conjugate of quaternion
q_inv = quatconj(q)/norm(q)^2;

end

