function [HINF] = initHINF(Est_ICs)

%initial states ang_vel, quat, gyro bias
HINF.X_hat.q_hat = Est_ICs.q;
HINF.X_hat.omega_hat = Est_ICs.omega;
HINF.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

HINF.P_hat = Est_ICs.P_init_att;

HINF.P_hat_eps = HINF.P_hat;


HINF.G_k = 50*[eye(4), zeros(4,3)];


HINF.accel_bound = 1; % +/- how much larger thna gravity before not used in update
