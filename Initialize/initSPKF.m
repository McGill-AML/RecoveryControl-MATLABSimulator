function [SPKF] = initSPKF(Est_ICs, loop_no)
%estimator constants
SPKF.kappa = -3; % SPKF scaling factor

%initial states ang_vel, quat, gyro bias
SPKF.X_hat.q_hat = Est_ICs.q;
SPKF.X_hat.omega_hat = Est_ICs.omega;
SPKF.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
SPKF.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7]); % initial covariance 

SPKF.P_hat(1:3,1:3)  = SPKF.P_hat(1:3,1:3)*0.1;


SPKF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update


SPKF.use_acc = 1; % whether or not accelerometer reading is used in update


SPKF.alpha = 1; %dictates spread of sigma points

SPKF.beta = 2; %2 is optimal for gaussian noise