function [ dx ] = VelFilter( t, x, a, b, vicon_time, vicon_pos )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


x = reshape(x,[max(size(x)),1]);
dx = zeros(size(x));

p_m = interp1(vicon_time,vicon_pos,t); 

dx(1) = x(2);
dx(2) = -a*b*x(1) -(a+b)*x(2) + p_m;

end

