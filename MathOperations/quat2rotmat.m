function [ R ] = quat2rotmat( q )
%quat2rotmat.m Converts rotation representation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Converts rotation representation from quaternion to 
%                rotation matrix
%-------------------------------------------------------------------------%
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

Rx = [q0^2+q1^2-q2^2-q3^2;2*(q1*q2+q0*q3);2*(q1*q3-q0*q2)];
Ry = [2*(q1*q2-q0*q3);q0^2-q1^2+q2^2-q3^2;2*(q2*q3+q0*q1)];
Rz = [2*(q1*q3+q0*q2);2*(q2*q3-q0*q1);q0^2-q1^2-q2^2+q3^2];

R = [Rx,Ry,Rz];

end

