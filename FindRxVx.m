function [ Rx_init, Vx_init, t0  ] = FindRxVx( Tc, Vc, wall_loc, q0, traj_head, t0, ax_given )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

global prop_loc Rbumper

R0 = quatRotMat(q0);
[roll, pitch, yaw] = quat2angle(q0,'xyz');

if traj_head >= 0 && traj_head < pi/2
    bumper_c = 4;
elseif traj_head >= pi/2 && traj_head <= pi
    bumper_c = 3;
elseif traj_head < 0 && traj_head(1) >= -pi/2
    bumper_c = 1;
elseif traj_head < -pi/2 && traj_head >= -pi
    bumper_c = 2;
else
    error('Choose another initial heading');
end

tilt = FindTilt(R0,traj_head);
xc_w = wall_loc - (R0(:,1)'*(prop_loc(:,bumper_c)) + Rbumper*cos(tilt));

ax = FindAx(roll, pitch, traj_head, tilt, ax_given); 


Vx_init = Vc - ax*Tc;
Rx_init = xc_w - (Vx_init)*Tc - 0.5*ax*Tc^2;

if Rx_init >= xc_w
    t0 = -Vx_init / ax;
    Rx_init = Rx_init + Vx_init*t0 + 0.5*ax*t0^2;
    Vx_init = 0;    
end

end

