function [ q_conj ] = quatconj( q )
% Returns conjugate of quaternion
q_conj = [q(1);-q(2);-q(3);-q(4)];

end

