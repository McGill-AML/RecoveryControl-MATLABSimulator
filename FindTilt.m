function [ tilt ] = FindTilt( R0, heading )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


axis_b = RotMat('Z',heading)*[1;0;0];

axis_w = R0'*axis_b;

tilt_sign = sign(acos([0 0 1]*axis_w)-pi/2);
tilt = tilt_sign*acos([1 0 0]*axis_w);

end

