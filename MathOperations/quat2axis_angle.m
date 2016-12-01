function [angle, axis] = quat2axis_angle(quat)
%outputs axis and angle in radians
angle = 2*acos(quat(1));

axis = quat(2:4)/sqrt(1-quat(1)^2);