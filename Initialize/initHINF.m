function [HINF] = initHINF(IC)

%initial states ang_vel, quat, gyro bias
HINF.X_hat.q_hat = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
HINF.X_hat.omega_hat = [0;0;0];
HINF.X_hat.bias_gyr = [0;0;0];

%estimator constants

HINF.P_hat = diag([0.01,0.01,0.01,0.01, 0.001,0.001, 0.001]);

HINF.P_hat_eps = HINF.P_hat;


HINF.G_k = 0/2*[eye(4), zeros(4,3)];


HINF.accel_bound = 1; % +/- how much larger thna gravity before not used in update
