function [Control] = controllerrecovery(tStep, Pose, Twist, Control, Hist)
% Performs collision recovery control.
%
% Inputs: 
%   dt              :   time step (default 1/200 sec)
%   Pose            :   Struct of the pose of the quadrotor
%   Twist           :   Struct of the twist of the quadrotor
%   Control         :   Struct of the control variables and output
%   Hist.control    :   History of control signals
%
% Outputs:
%   Control         :   Struct of the control variables and output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of Quaternion Formulated Non-Linear Attitude Controller 
% Author: Gareth Dicker
% Start Date: August 2015
%
% The mathematical framework for this controller was taken from:
% "Automatic Re-Initialization and Failure Recovery for Aggressive Flight
% with a Monocular Based Vision-Based Quadrotor" 
%                    by Faessler, Fontana, Forster and Scaramuzza
%   1. Compute thrust 
%   2. Compute roll/pitch error
%   3. Compute desired body rates
%   4. Compute control output thrust/torques and propellor RPMs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global m Ixx Iyy Izz u2RpmMat;

if sum(abs(Control.acc)) == 0
    error('Desired acceleration must be non-zero');
end

%% 1. Compute thrust

% compute body z-axis direction, -1 because quad z-axis points down
bodyFrameZAxis = quatrotate(Pose.attQuat', [0 0 -1]);

% if the desired acceleration and body frame z-axis point at all in the
% same direction, then set thrust equal to their dot product times mass
% Note: negative sign because z-axis points opposite of thrust

if dot(Control.acc,bodyFrameZAxis) > 0
    Control.u(1) = -m*dot(Control.acc, bodyFrameZAxis);
else
    % can't give thrust in positive z-axis
    Control.u(1) = 0;
end

% ----------> Saturate thrust here <-----------

%% 2. Compute roll/pitch error

% compute desired body frame z-axis
if sum(Control.acc ~= 0)
    bodyFrameZAxisDesired = Control.acc / norm(Control.acc);
else
    % free fall 
    bodyFrameZAxisDesired = [0; 0; 0];
end

% compute angle between actual and desired body frame z axis
theta = acos(dot(bodyFrameZAxis, bodyFrameZAxisDesired));

if theta == 0 
    % set the error quaternion to the identity in body frame
    Control.errQuat = [1 0 0 0]';
else
    % compute the world frame normal 
    n = cross(bodyFrameZAxis, bodyFrameZAxisDesired);
    % to truncate any rounding error, normalize 
    n = n/norm(n);
    % rotate into body frame
    nBody = quatrotate(quatinv(Pose.attQuat)',n);
    % compute desired quaternion
    Control.errQuat = real([cos(theta/2)         ; nBody(1)*sin(theta/2); ...
                       nBody(2)*sin(theta/2); nBody(3)*sin(theta/2)]);
end


%% 3. Compute desired body rates 

% compute first two desired body rates (p and q) by scaling error
% quaternion terms q1 and q2
ERROR_TO_DESIRED_BODYRATES = 20;    %this is p_{rp} of Faessler's control
Control.twist.angVel(1:2) = ERROR_TO_DESIRED_BODYRATES*Control.errQuat(2:3);

% if the error is negative, make the desired body rates negative
if Control.errQuat(1) < 0
    Control.twist.angVel(1:2) = -Control.twist.angVel(1:2);
end

% set thired desired body rate (r) always go to zero
Control.twist.angVel(3) = 0;

%% Perform PD control on actual and desired body rates
    
% define gains
propPQ  = 20.0; % proportional for p and q
propR   = 2.0;  % proportional only for r

% compute desired boy frame accelerations with P control on the body rates
vP = propPQ*(Control.twist.angVel(1) - Twist.angVel(1));
vQ = propPQ*(Control.twist.angVel(2) - Twist.angVel(2));
vR = propR *(Control.twist.angVel(3) - Twist.angVel(3)); 

% compute body frame torques from desired body frame accelerations
Control.u(2) = (vP - Twist.angVel(2)*Twist.angVel(3)*(Iyy-Izz)/Ixx)*Ixx;
Control.u(3) = (vQ - Twist.angVel(1)*Twist.angVel(3)*(Izz-Ixx)/Iyy)*Iyy;
Control.u(4) = (vR - Twist.angVel(1)*Twist.angVel(2)*(Ixx-Iyy)/Izz)*Izz;

% compute desired propeller speeds in RPM
rpmSquared = u2RpmMat*Control.u;
for i = 1:4
    if rpmSquared(i) < 0
        rpmSquared(i) = 0;
    end
end
Control.rpm = real(sqrt(rpmSquared));

% saturate commands
rpmSaturation = 8000;
Control.rpm (Control.rpm > rpmSaturation) = rpmSaturation;
Control.rpm (Control.rpm < 1000) = 1000;

% set negatives for CW and CCW rotations
Control.rpm(1) = -Control.rpm(1);
Control.rpm(3) = -Control.rpm(3);

end

