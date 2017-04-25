function [ASPKF_opt] = initASPKF_opt(Est_ICs, useExpData)
ASPKF_opt.kappa = -6; % SPKF scaling factor


ASPKF_opt.X_hat.q_hat = Est_ICs.q;

ASPKF_opt.X_hat.omega_hat = Est_ICs.omega;
ASPKF_opt.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
ASPKF_opt.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7])*1; % initial covariance 

ASPKF_opt.P_hat(1:3,1:3) = ASPKF_opt.P_hat(1:3,1:3)*0.1;
%estimator constants

if useExpData == 0
    ASPKF_opt.accel_bound = 1; % +/- how much larger thna gravity before not used in update
else
    ASPKF_opt.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end

ASPKF_opt.S_k = zeros(6); % R_k adaptation matrix

ASPKF_opt.Lam_k = zeros(6); % Q_k adaptation matrix

ASPKF_opt.moving_win = 15;

ASPKF_opt.innov_error = zeros(6,6,ASPKF_opt.moving_win); % create storage matrix for innov values 


ASPKF_opt.use_acc = zeros(1,ASPKF_opt.moving_win); %use accelerometer vector, to include accelerometer in adaptive

ASPKF_opt.alpha = .5; %dictates spread of sigma points

ASPKF_opt.beta = 2; %2 is optimal for gaussian noise




