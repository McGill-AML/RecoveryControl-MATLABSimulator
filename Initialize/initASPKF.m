function [ASPKF] = initASPKF(IC,ASPKF)
%initial states ang_vel, quat, gyro bias
ASPKF.X_hat.q_hat = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
ASPKF.X_hat.omega_hat = [0;0;0];
ASPKF.X_hat.bias_gyr = [0;0;0];
%initial covariance - contains variance for MRP and gyr bias. variance in
%ang vel is the noise value
ASPKF.P_hat = diag([0.01,0.01,0.01, 0.001,0.001, 0.001]); % initial covariance 

%estimator constants
% ASPKF.kappa = 3; % SPKF scaling factor

ASPKF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update

ASPKF.innov_tresh = 0.5; % innovation sum threshold

ASPKF.G_max = 10; % max adaptive gain

ASPKF.G_rate = 0.25; % how fast the adaptive gain grows when innov sum above threshold

ASPKF.G_k = 1; %initial adaptive gain 1 = regular EKF


ASPKF.innov_k = zeros(30,1); % length of this vector decide how far back to look at innovation

ASPKF.gamma = ones(6); % estimator measurement weights in innovation sum. might need to scale magnetometer?



