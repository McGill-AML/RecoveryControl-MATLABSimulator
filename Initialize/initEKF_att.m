function [EKF_att] = initEKF_att(Est_ICs, useExpData)

%initial states ang_vel, quat, gyro bias
EKF_att.X_hat.q_hat = Est_ICs.q;
EKF_att.X_hat.omega_hat = Est_ICs.omega;
EKF_att.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

EKF_att.P_hat = Est_ICs.P_init_att;

if useExpData == 0 
    EKF_att.P_hat(5:7,5:7) = EKF_att.P_hat(5:7,5:7)*10;
else
    EKF_att.P_hat(1:4,1:4) = EKF_att.P_hat(1:4,1:4)*0.001;
end


if useExpData == 0 
    EKF_att.accel_bound = 0.5; % changed from 1  % +/- how much larger thna gravity before not used in update
else
    EKF_att.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end

EKF_att.innovation = zeros(6,1);
EKF_att.K_k = zeros(7,6);

