function [ tilt ] = rpy2tilt( roll, pitch, yaw)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

axis_y1 = RotMat('Z',yaw)'*[0;1;0];
axis_x2 = RotMat('Y',pitch)'*RotMat('Z',yaw)'*[1;1;0];
axis_zw = [0;0;1];
angle = pitch*axis_y1 + roll*axis_x2 + yaw*axis_zw;
tilt = angle(2);
end

