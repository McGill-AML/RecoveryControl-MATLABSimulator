function [ R ] = quat2rotmat( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% if ~strcmp(class(angle),'sym')
%     error('Input angle must be symbolic');
% end
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

Rx = [q0^2+q1^2-q2^2-q3^2;2*(q1*q2+q0*q3);2*(q1*q3-q0*q2)];
Ry = [2*(q1*q2-q0*q3);q0^2-q1^2+q2^2-q3^2;2*(q2*q3+q0*q1)];
Rz = [2*(q1*q3+q0*q2);2*(q2*q3-q0*q1);q0^2-q1^2-q2^2+q3^2];

% Rx = quatmultiply(quatmultiply(q,[0;1;0;0]),quatconj(q));
% Ry = quatmultiply(quatmultiply(q,[0;0;1;0]),quatconj(q));
% Rz = quatmultiply(quatmultiply(q,[0;0;0;1]),quatconj(q));
R = [Rx,Ry,Rz];

end

