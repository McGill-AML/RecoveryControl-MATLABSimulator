function x_cross = cross_mat(x)
% Cross matrix: 3 x 1 column matrix input, 3 x 3 skew-sym matrix output. 

x_cross = [ 0   -x(3)   x(2);
            x(3)   0   -x(1);
           -x(2) x(1)     0];