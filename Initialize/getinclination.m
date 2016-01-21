function [ inclination ] = getinclination( rotMat, heading )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


axisZBody = invar2rotmat('Z',heading)*[1;0;0];

axisZWorld = rotMat'*axisZBody;

inclinationSign = sign(acos([0 0 1]*axisZWorld)-pi/2);
inclination = inclinationSign*acos([1 0 0]*axisZWorld);

end

