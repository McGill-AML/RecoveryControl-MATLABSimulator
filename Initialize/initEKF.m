function [EKF] = initEKF(Est_ICs)
%initial states vel, pos, acc bias
EKF.X_hat.pos_hat = Est_ICs.posn;
EKF.X_hat.vel_hat = Est_ICs.linVel;
EKF.X_hat.bias_acc = Est_ICs.bias_acc;

% initial covariance pos, vel, acc_bias
EKF.P_hat = Est_ICs.P_init_pos; 