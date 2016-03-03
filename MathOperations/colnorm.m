function [ norms ] = colnorm( array )
%COLNORM Find column-wise normal 
%   Detailed explanation goes here

norms = sqrt(sum(array.^2,1));


end

