function [ norms ] = colnorm( array )
%COLNORM Find column-wise normal 

norms = sqrt(sum(array.^2,1));


end

