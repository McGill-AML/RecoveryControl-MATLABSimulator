function [ speedRad ] = rpm2rad( speedRPM )
%Converts rotation speed from rotations per minute (RPM) to rad/s
speedRad = speedRPM *(2*pi/60);

end

