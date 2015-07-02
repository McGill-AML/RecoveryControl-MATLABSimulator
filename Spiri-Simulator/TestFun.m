function [ dx ] = TestFun( t,x )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
dx = zeros(2,1);
dx(1) = x(1)*x(2);
dx(2) = x(1)+x(2);

end

