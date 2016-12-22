function [AHINF] = initAHINF(Est_ICs)

%initial states ang_vel, quat, gyro bias
AHINF.X_hat.q_hat = Est_ICs.q;
AHINF.X_hat.omega_hat = Est_ICs.omega;
AHINF.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

AHINF.P_hat = Est_ICs.P_init_att;

AHINF.P_hat_eps = AHINF.P_hat;


%adaptive values
AHINF.innov_tresh = 0.5; % innovation sum threshold

AHINF.delta_max = 70; % max adaptive gain

AHINF.delta_rate = 35; % how fast the adaptive gain grows when innov sum above threshold

AHINF.delta = 0; %initial adaptive gain 0 = regular EKF


AHINF.innov_k = zeros(15,1); % length of this vector decide how far back to look at innovation

AHINF.gamma = diag([ones(1,3),ones(1,3)*400]); % scales magnetometer to be same size as accelerometer readings


AHINF.G_k = [eye(4), zeros(4,3)]; % the H inf gain


AHINF.accel_bound = 1; % +/- how much larger thna gravity before not used in update

AHINF.use_acc = 1; % values for if it should use accelerometer reading
