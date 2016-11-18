function [AEKF] = initAEKF(Est_ICs)
%initial states vel, pos, acc bias
AEKF.X_hat.pos_hat = Est_ICs.posn;
AEKF.X_hat.vel_hat = Est_ICs.linVel;
AEKF.X_hat.bias_acc = Est_ICs.bias_acc;

% initial covariance pos, vel, acc_bias
AEKF.P_hat = Est_ICs.P_init_pos; 

%estimator constants
AEKF.accel_bound = 0.5; % accelerometer bound - dont need in EKF <---***

AEKF.innov_tresh = 0.05; % innovation sum threshold

AEKF.G_max = 4; %max adaptive gain

AEKF.G_rate = 0.25; % how fast the adaptive gain grows when innov sum above threshold

AEKF.G_k = 1; %initial adaptive gain 1 = regular EKF

AEKF.innov_k = zeros(30,1); % length of this vector decide how far back to look at innovation

AEKF.gamma = zeros(6); % weight of different measurements in innov sum. might need to scale long lat to m/s


