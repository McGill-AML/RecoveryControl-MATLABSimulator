function [Sensor, COMP] = applylpfilt(Sensor, COMP)
%apply lowpass filter as defined in PX4 firmware, mathlib
%apply mag lp filt
% COMP.lpfilt.mag.delay1 = Sensor.mag - COMP.lpfilt.mag.delay2*COMP.lpfilt.a2 -COMP.lpfilt.mag.delay3*COMP.lpfilt.a3;
% Sensor.mag = COMP.lpfilt.mag.delay1*COMP.lpfilt.b1 +COMP.lpfilt.mag.delay2*COMP.lpfilt.b2 +COMP.lpfilt.mag.delay3*COMP.lpfilt.b3;
% 
% COMP.lpfilt.mag.delay3 = COMP.lpfilt.mag.delay2;
% COMP.lpfilt.mag.delay2 = COMP.lpfilt.mag.delay1;

%apply acc lp filt
COMP.lpfilt.acc.delay1 = Sensor.acc - COMP.lpfilt.acc.delay2*COMP.lpfilt.a2 - COMP.lpfilt.acc.delay3*COMP.lpfilt.a3;
Sensor.acc = COMP.lpfilt.acc.delay1*COMP.lpfilt.b1 + COMP.lpfilt.acc.delay2*COMP.lpfilt.b2 +COMP.lpfilt.acc.delay3*COMP.lpfilt.b3;

COMP.lpfilt.acc.delay3 = COMP.lpfilt.acc.delay2;
COMP.lpfilt.acc.delay2 = COMP.lpfilt.acc.delay1;

% apply gyro lp filt
COMP.lpfilt.gyro.delay1 = Sensor.gyro - COMP.lpfilt.gyro.delay2*COMP.lpfilt.a2 -COMP.lpfilt.gyro.delay3*COMP.lpfilt.a3;
Sensor.gyro = COMP.lpfilt.gyro.delay1*COMP.lpfilt.b1 +COMP.lpfilt.gyro.delay2*COMP.lpfilt.b2 +COMP.lpfilt.gyro.delay3*COMP.lpfilt.b3;

COMP.lpfilt.gyro.delay3 = COMP.lpfilt.gyro.delay2;
COMP.lpfilt.gyro.delay2 = COMP.lpfilt.gyro.delay1;