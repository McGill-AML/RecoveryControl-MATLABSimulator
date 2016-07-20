function [ inclination] = getinclination( rotMat, heading )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% Inclination is positive if quadrotor tilted towards wall, and negative
% otherwise

axisHeadingBody = invar2rotmat('Z',heading)*[1;0;0]; %Axis in plane with body xy plane, in same direction as world X axis, represented in body frame

axisHeadingWorld = rotMat'*axisHeadingBody;

inclinationSign = sign(acos([0 0 1]*axisHeadingWorld)-pi/2);
inclination = inclinationSign*acos([1 0 0]*axisHeadingWorld);


end

