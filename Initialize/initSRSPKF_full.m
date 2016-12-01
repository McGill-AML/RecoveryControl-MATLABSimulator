function [SRSPKF_full] = initSRSPKF_full(Est_ICs)
%estimator constants
SRSPKF_full.kappa = 0; % SRSPKF_full scaling factor

%initial states ang_vel, quat, gyro bias
SRSPKF_full.X_hat.pos_hat = Est_ICs.posn;
SRSPKF_full.X_hat.vel_hat = Est_ICs.linVel;
SRSPKF_full.X_hat.q_hat = Est_ICs.q;
SRSPKF_full.X_hat.omega_hat = Est_ICs.omega;
SRSPKF_full.X_hat.bias_acc = Est_ICs.bias_acc;
SRSPKF_full.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
P_hat = blkdiag(Est_ICs.P_init_pos(1:6,1:6), Est_ICs.P_init_att(1:3,1:3), ...
                            Est_ICs.P_init_pos(7:9,7:9), Est_ICs.P_init_att(5:7,5:7)) ; % initial covariance 


SRSPKF_full.S_hat = chol(P_hat, 'lower'); %set initial cholesky factorization estimate

SRSPKF_full.accel_bound = 1; % +/- how much larger thna gravity before not used in update

SRSPKF_full.use_acc = 1; % whether or not accelerometer reading is used in update

SRSPKF_full.alpha = 1; %dictates spread of sigma points

SRSPKF_full.beta = 2; %2 is optimal for gaussian noise