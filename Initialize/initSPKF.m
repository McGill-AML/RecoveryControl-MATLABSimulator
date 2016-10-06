function [SPKF] = initSPKF()
%initial states ang_vel, quat, gyro bias
SPKF.X_hat.q_hat = [1;0;0;0];
SPKF.X_hat.omega_hat = [0;0;0];
SPKF.X_hat.bias_gyr = [0;0;0];
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
SPKF.P_hat = diag([0.01,0.01,0.01, 0.0001,0.0001, 0.0001]); % initial covariance 

%estimator constants
SPKF.kappa = 0; % SPKF scaling factor

SPKF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update
