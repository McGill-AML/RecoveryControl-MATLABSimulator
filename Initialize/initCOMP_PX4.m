function [COMP] = initCOMP_PX4(Est_ICs, useExpData)

%initial states ang_vel, quat, gyro bias
COMP.X_hat.q_hat = Est_ICs.q;
COMP.X_hat.omega_hat = Est_ICs.omega;
COMP.X_hat.bias_gyr =  Est_ICs.bias_gyr;

COMP.w_mes = [0; 0; 0];
%estimator constants

% COMP.acc_k_i = 0.05; % 0.5 works
% if useExpData == 0
%     COMP.mag_k_i = 0.1; % 1 works
% else
%     COMP.mag_k_i = 0.1; % 1 works
% end
% COMP.acc_k_i = 0.25; % 0.5 works

if useExpData == 0
    COMP.acc_k_i = 0.25; % 0.5 works
else
    COMP.acc_k_i = 0.25; % 0.5 works
end

if useExpData == 0
    COMP.mag_k_i = 0.15; % 1 works
else
    COMP.mag_k_i = 0.05; % 1 works
end


COMP.k_p = 1; % 1 works


if useExpData == 0
    COMP.gyr_bias_k_i = .25; % .05-.5 works
else
    COMP.gyr_bias_k_i = .05; % .05-.5 works
end


if useExpData == 0
    COMP.accel_bound = 1; % +/- how much larger thna gravity before not used in update
else
    COMP.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end


% lowpass filter stuff (pulled exactly from px4 mathlib lp filter)
samplefrq = 250;
cutofffrq = 30;
fr = samplefrq/cutofffrq;
ohm = atan(pi/fr);
c = 1+2*cos(pi/4)*ohm + ohm^2;
COMP.lpfilt.b1 = ohm^2/c;
COMP.lpfilt.b2 = 2*COMP.lpfilt.b1;
COMP.lpfilt.b3 = COMP.lpfilt.b1;
COMP.lpfilt.a2 = 2*(ohm^2-1)/c;
COMP.lpfilt.a3 = (1-2*cos(pi/4)*ohm+ohm^2)/c;



