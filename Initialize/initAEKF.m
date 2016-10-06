function [AEKF] = initAEKF()
%initial states vel, pos, acc bias
AEKF.X_hat.pos_hat = [0;0;0];
AEKF.X_hat.vel_hat = [0;0;0];
AEKF.X_hat.bias_acc = [0;0;0];

% initial covariance pos, vel, acc_bias
AEKF.P_hat = diag([0.1,0.1,0.1, 0.005,0.005,0.005, 0.0001, 0.0001, 0.0001]); 

%estimator constants
AEKF.kappa = 0; % dont need in EKF either

AEKF.accel_bound = 0.5; % accelerometer bound - dont need in EKF <---***

AEKF.innov_tresh = 0.05; % innovation sum threshold

AEKF.G_max = 4; %max adaptive gain

AEKF.G_rate = 0.25; % how fast the adaptive gain grows when innov sum above threshold

AEKF.G_k = 1; %initial adaptive gain 1 = regular EKF

AEKF.innov_k = zeros(30,1); % length of this vector decide how far back to look at innovation

AEKF.gamma = zeros(6); % weight of different measurements in innov sum. might need to scale long lat to m/s


