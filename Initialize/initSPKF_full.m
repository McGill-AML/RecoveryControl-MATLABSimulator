function [SPKF_full] = initSPKF_full(Est_ICs)
%estimator constants
SPKF_full.kappa = 0; % SPKF_full scaling factor

%initial states ang_vel, quat, gyro bias
SPKF_full.X_hat.pos_hat = Est_ICs.posn;
SPKF_full.X_hat.vel_hat = Est_ICs.linVel;
SPKF_full.X_hat.q_hat = Est_ICs.q;
SPKF_full.X_hat.omega_hat = Est_ICs.omega;
SPKF_full.X_hat.bias_acc = Est_ICs.bias_acc;
SPKF_full.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
SPKF_full.P_hat = blkdiag(Est_ICs.P_init_pos(1:6,1:6), Est_ICs.P_init_att(1:3,1:3), ...
                            Est_ICs.P_init_pos(7:9,7:9), Est_ICs.P_init_att(5:7,5:7)) ; % initial covariance 




SPKF_full.accel_bound = 1; % +/- how much larger thna gravity before not used in update

SPKF_full.alpha = 1; %dictates spread of sigma points

SPKF_full.beta = 2; %2 is optimal for gaussian noise