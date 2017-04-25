function [COMP] = initlpfilt(Sensor, COMP)
%init lp filter as done in px4 firmware mathlib

dval = Sensor.mag/(COMP.lpfilt.b1 + COMP.lpfilt.b2 + COMP.lpfilt.b3);

COMP.lpfilt.mag.delay2 = dval;
COMP.lpfilt.mag.delay3 = dval;

dval = Sensor.acc/(COMP.lpfilt.b1 + COMP.lpfilt.b2 + COMP.lpfilt.b3);

COMP.lpfilt.acc.delay2 = dval;
COMP.lpfilt.acc.delay3 = dval;

dval = Sensor.gyro/(COMP.lpfilt.b1 + COMP.lpfilt.b2 + COMP.lpfilt.b3);

COMP.lpfilt.gyro.delay2 = dval;
COMP.lpfilt.gyro.delay3 = dval;