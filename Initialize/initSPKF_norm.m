function [SPKF_norm] = initSPKF_norm(Est_ICs, loop_no)
%estimator constants
SPKF_norm.kappa = 3; % SPKF_norm scaling factor

%initial states ang_vel, quat, gyro bias
SPKF_norm.X_hat.q_hat = Est_ICs.q;
SPKF_norm.X_hat.omega_hat = Est_ICs.omega;
SPKF_norm.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
SPKF_norm.P_hat = Est_ICs.P_init_att; % initial covariance 


SPKF_norm.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update


SPKF_norm.use_acc = 1; % whether or not accelerometer reading is used in update


SPKF_norm.alpha = 1; %dictates spread of sigma points

SPKF_norm.beta = 2; %2 is optimal for gaussian noise