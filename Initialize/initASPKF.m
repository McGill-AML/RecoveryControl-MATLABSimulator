function [ASPKF] = initASPKF(Est_ICs, useExpData)
ASPKF.kappa = -6; % SPKF scaling factor

%initial states ang_vel, quat, gyro bias
ASPKF.X_hat.q_hat = Est_ICs.q;
ASPKF.X_hat.omega_hat = Est_ICs.omega;
ASPKF.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
ASPKF.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7]); % initial covariance 

if useExpData == 0
    ASPKF.P_hat(1:3,1:3) = ASPKF.P_hat(1:3,1:3)*0.01;
else
    ASPKF.P_hat(1:3,1:3) = ASPKF.P_hat(1:3,1:3)*0.1;
end

%estimator constants

if useExpData == 0
    ASPKF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update
else
    ASPKF.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end

% ASPKF.innov_tresh = 1.25; % 
ASPKF.innov_tresh = 0.6; % innovation sum threshold


% ASPKF.G_max = 5;
if useExpData == 0
    ASPKF.G_max = 10; % max adaptive gain
else
    ASPKF.G_max = 10; % max adaptive gain
end

% ASPKF.G_rate = 0.25;
ASPKF.G_rate = 0.25; % how fast the adaptive gain grows when innov sum above threshold

ASPKF.G_k = 1; %initial adaptive gain 1 = regular EKF


ASPKF.innov_k = zeros(15,1); % length of this vector decide how far back to look at innovation

% estimator measurement weights in innovation sum. scales so that they're equal
% ASPKF.gamma = diag([ones(1,3)*400,ones(1,3)]); % this one's old, no real
% testing, just kinda did it. 
ASPKF.gamma = diag([ones(1,3)*30,ones(1,3)]); % scale factor of 30 based on equating norm of accel with norm of mag in experiments 

ASPKF.use_acc = 1; %use accelerometer if within magnitude bounds

ASPKF.alpha = .5; %dictates spread of sigma points

ASPKF.beta = 2; %2 is optimal for gaussian noise




