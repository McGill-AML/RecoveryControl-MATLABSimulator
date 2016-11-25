function [EKF_att] = initEKF_att(Est_ICs)

%initial states ang_vel, quat, gyro bias
EKF_att.X_hat.q_hat = Est_ICs.q;
EKF_att.X_hat.omega_hat = Est_ICs.omega;
EKF_att.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

EKF_att.P_hat = Est_ICs.P_init_att;


EKF_att.accel_bound = 1; % +/- how much larger thna gravity before not used in update
