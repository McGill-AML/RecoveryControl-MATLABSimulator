function [ASPKF_opt] = initASPKF_opt(Est_ICs)
ASPKF_opt.kappa = 3; % SPKF scaling factor

%initial states ang_vel, quat, gyro bias
ASPKF_opt.X_hat.q_hat = Est_ICs.q;
ASPKF_opt.X_hat.omega_hat = Est_ICs.omega;
ASPKF_opt.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
ASPKF_opt.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7]); % initial covariance 

ASPKF_opt.P_hat(1:3,1:3) = ASPKF_opt.P_hat(1:3,1:3)*0.1;
%estimator constants


ASPKF_opt.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update

ASPKF_opt.S_k = zeros(6); % R_k adaptation matrix

ASPKF_opt.Lam_k = zeros(6); % Q_k adaptation matrix

ASPKF_opt.moving_win = 15;

ASPKF_opt.innov_error = zeros(6,6,ASPKF_opt.moving_win); % create storage matrix for innov values 


ASPKF_opt.use_acc = zeros(1,ASPKF_opt.moving_win); %use accelerometer vector, to include accelerometer in adaptive

ASPKF_opt.alpha = 1; %dictates spread of sigma points

ASPKF_opt.beta = 0; %2 is optimal for gaussian noise




