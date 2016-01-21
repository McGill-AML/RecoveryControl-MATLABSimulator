function [ idx ] = vlookup( array, val )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

[~, idx] = min(abs(array-val));

end

