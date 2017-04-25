function [HINF] = initHINF(Est_ICs, useExpData)

%initial states ang_vel, quat, gyro bias
HINF.X_hat.q_hat = Est_ICs.q;
HINF.X_hat.omega_hat = Est_ICs.omega;
HINF.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

HINF.P_hat = Est_ICs.P_init_att;

HINF.P_hat_eps = HINF.P_hat;

HINF.P_hat(1:4,1:4) = HINF.P_hat(1:4,1:4)*0.1;
HINF.P_hat_eps(1:4,1:4) = HINF.P_hat(1:4,1:4)*0.1;

if useExpData == 0
HINF.P_hat(5:7,5:7) = HINF.P_hat(5:7,5:7)*10;
HINF.P_hat_eps(5:7,5:7) = HINF.P_hat(5:7,5:7)*10;
end

HINF.G_k = 0.1*[eye(4), zeros(4,3)];%0.1*[eye(4), zeros(4,3)];

if useExpData == 0
    HINF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update
else
    HINF.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end