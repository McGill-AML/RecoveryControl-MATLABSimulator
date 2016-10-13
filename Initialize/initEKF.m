function [EKF] = initEKF(IC)
%initial states vel, pos, acc bias
EKF.X_hat.pos_hat = IC.posn;
EKF.X_hat.vel_hat = [0;0;0];
EKF.X_hat.bias_acc = [0;0;0];

% initial covariance pos, vel, acc_bias
EKF.P_hat = diag([0.1,0.1,0.1, 0.005,0.005,0.005, 0.0001, 0.0001, 0.0001]); 