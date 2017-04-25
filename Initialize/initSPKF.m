function [SPKF] = initSPKF(Est_ICs, useExpData)
%estimator constants
if useExpData == 1
    SPKF.kappa = -3; % SPKF scaling factor
else
    SPKF.kappa = -6; % SPKF scaling factor
end

%initial states ang_vel, quat, gyro bias

SPKF.X_hat.q_hat = Est_ICs.q;

SPKF.X_hat.omega_hat = Est_ICs.omega;
SPKF.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
SPKF.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7])*1; % initial covariance 

if useExpData == 0 % Yo I flipped these!!!!
    SPKF.P_hat(1:3,1:3)  = SPKF.P_hat(1:3,1:3)*0.01;
else
    SPKF.P_hat(1:3,1:3)  = SPKF.P_hat(1:3,1:3)*0.1;
end


if useExpData == 0 %
    SPKF.accel_bound = 0.5; % this was set to 1, +/- how much larger thna gravity before not used in update
else
    SPKF.accel_bound = 5; % was 4 previously % +/- how much larger thna gravity before not used in update
end

SPKF.use_acc = 1; % whether or not accelerometer reading is used in update


SPKF.alpha = 0.5; %dictates spread of sigma points

SPKF.beta = 2; %2 is optimal for gaussian noise

SPKF.innovation = zeros(6,1);
SPKF.innov_cov = zeros(6);
SPKF.X_hat.X_k_m = zeros(6,1);
SPKF.X_hat.DX_k = zeros(6,1);
SPKF.K_k = zeros(6);

SPKF.a = 1;