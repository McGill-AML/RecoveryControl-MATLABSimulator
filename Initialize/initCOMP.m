function [COMP] = initCOMP(Est_ICs)

%initial states ang_vel, quat, gyro bias
COMP.X_hat.q_hat = Est_ICs.q;
COMP.X_hat.omega_hat = Est_ICs.omega;
COMP.X_hat.bias_gyr =  Est_ICs.bias_gyr;

COMP.w_mes = [0; 0; 0];
%estimator constants

COMP.acc_k_i = 0.5; % 0.5 works
COMP.mag_k_i = 1; % 1 works

COMP.k_p = 0.1; % 1 works

COMP.gyr_bias_k_i = .05; % .05-.5 works

COMP.accel_bound = 1; % +/- how much larger thna gravity before not used in update

