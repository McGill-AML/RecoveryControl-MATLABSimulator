function [COMP] = initCOMP(IC)

%initial states ang_vel, quat, gyro bias
COMP.X_hat.q_hat = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
COMP.X_hat.omega_hat = [0;0;0];
COMP.X_hat.bias_gyr = [0;0;0];

%estimator constants

COMP.acc_k_i = 0.5;
COMP.mag_k_i = 0.5;

COMP.k_p = 1;

COMP.gyr_bias_k_i = 0.3; 

COMP.accel_bound = 1; % +/- how much larger thna gravity before not used in update
