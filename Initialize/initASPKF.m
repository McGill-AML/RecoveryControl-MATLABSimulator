function [ASPKF] = initASPKF(Est_ICs)
ASPKF.kappa = 3; % SPKF scaling factor

%initial states ang_vel, quat, gyro bias
ASPKF.X_hat.q_hat = Est_ICs.q;
ASPKF.X_hat.omega_hat = Est_ICs.omega;
ASPKF.X_hat.bias_gyr = Est_ICs.bias_gyr;
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
ASPKF.P_hat = Est_ICs.P_init_att([1:3,5:7],[1:3,5:7]); % initial covariance 

ASPKF.P_hat(1:3,1:3) = ASPKF.P_hat(1:3,1:3)*0.1;
%estimator constants


ASPKF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update

ASPKF.innov_tresh = 1; % innovation sum threshold

ASPKF.G_max = 5; % max adaptive gain

ASPKF.G_rate = 0.25; % how fast the adaptive gain grows when innov sum above threshold

ASPKF.G_k = 1; %initial adaptive gain 1 = regular EKF


ASPKF.innov_k = zeros(15,1); % length of this vector decide how far back to look at innovation

ASPKF.gamma = diag([ones(1,3),ones(1,3)*400]); % estimator measurement weights in innovation sum. might need to scale magnetometer?

ASPKF.use_acc = 1; %use accelerometer if within magnitude bounds

ASPKF.alpha = 1; %dictates spread of sigma points

ASPKF.beta = 0; %2 is optimal for gaussian noise




