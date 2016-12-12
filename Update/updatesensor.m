function [ Sensor ] = updatesensor( state, stateDeriv )
%updatesensor.m Update simulated accelerometer and gyro using quadrotor
%states
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Simulated accelerometer data at CM is
%   "Sensor.accelerometerAtCM". Simulated accelerometer data at the real
%   IMU_POSN on Navi is "Sensor.accelerometer". All measurements are
%   noise-free and bias-free (i.e., perfect measurements). 
%-------------------------------------------------------------------------%

global IMU_POSN g

rotMat = quat2rotmat(state(10:13));

Sensor.accelerometer = [0;0;0]; %placeholder
Sensor.gyro = state(4:6);
Sensor.accelerometerAtCM = (-rotMat*[0;0;-g] + stateDeriv(1:3) + cross(state(4:6),state(1:3)))/g; %in g's

%% Calculate Accelerometer reading for IMU located not at CM

posn = state(7:9);
posn_AtIMU = posn + rotMat'*IMU_POSN;

worldAcc = rotMat'*(stateDeriv(1:3)+cross(state(4:6),state(1:3)));
worldAngVel = rotMat'*state(4:6);
worldAngAcc = rotMat'*stateDeriv(4:6);

worldAcc_AtIMU = worldAcc + cross(worldAngAcc,posn_AtIMU-posn) + cross(worldAngVel,cross(worldAngVel,posn_AtIMU-posn));
Sensor.accelerometer = (rotMat*(worldAcc_AtIMU - [0;0;-g]))/g;

end

