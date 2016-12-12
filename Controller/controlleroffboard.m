function Control = controlleroffboard(state,stateDeriv,iSim,timeInit,tStep,Control)
%controlleroffboard.m High-level offboard/position controller
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%       Based on "Minimum Snap Trajectory Generation and Control for Quadrotors"
%       by Mellinger & Kumar, to match px4 offboard control. 
%       Gains tuned to mimic Crash Set VII (unstable heading control)
%-------------------------------------------------------------------------%

global m g Ixx Iyy Izz u2RpmMat motorDir

%% Save inputs 
posnDes = Control.pose.posn;
yawDes = Control.pose.attEuler(3);

posnDerivDes = Control.twist.linVel;
worldAccDes = Control.twist.worldAcc;

angVelDes = Control.twist.angVel;

q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
R = quat2rotmat(q);

errPosnDerivPrev = Control.errPosnDeriv;
errAngVelPrev = Control.errAngVel;

%% Controller Parameters
Kposn_P = diag([0.95,0.95,0.5]); %diag([0.95,0.95,0.5]);
Kvel_P = diag([2.0,2.0,1.5]); %diag([2,2,1.5]);
Kvel_I = diag([0.02,0.02,0.02]);
Kvel_D = diag([0,0,0]);
% Kvel_D = diag([0.05,0.05,0]);

Katt_P = diag([8,8,0.05]);%diag([8,8,0.05]);
Kattrate_P = diag([2.1,2.1,0.01]);%diag([2.1,2.1,0.05/20]); %Positive definite gain matrix on p,q,r
Kattrate_I = zeros(3,3);
Kattrate_D = diag([0, 0, 0]);
% Kattrate_D = diag([0.0055, 0.0055, 0]);

% Kattrate_P = diag([11,11,11]); %Positive definite gain matrix on p,q,r
% Kattrate_I = diag([5.73, 5.73, 5.73]);
% Kattrate_D = diag([0.32, 0.32, 0]);

%% Control
% Calculate errors on position and velocity
errPosn = state(7:9) - posnDes;

posnDerivDes = -Kposn_P*errPosn;
errPosnDeriv = stateDeriv(7:9) - posnDerivDes;

if iSim == timeInit
    errPosnDerivIntegral = [0;0;0];
    errPosnDerivDeriv = [0;0;0];    
else
    errPosnDerivIntegral = errPosnDeriv + errPosnDerivPrev;
    errPosnDerivDeriv = (errPosnDeriv - errPosnDerivPrev)/tStep;
end

Control.integralErrPosnDeriv = Control.integralErrPosnDeriv + errPosnDerivIntegral*tStep*0.5;


% Compute desired force vector
zW = [0;0;1]; %unit vector pointing in world upwards direction
% Fdes = -(-Kposn_P*errPosn - Kvel_P*errPosnDeriv + m*g*zW + m*worldAccDes);
Fdes = (- Kvel_P*errPosnDeriv -Kvel_I*Control.integralErrPosnDeriv...
         -Kvel_D*errPosnDerivDeriv + m*g*zW + m*worldAccDes);
zB = R'*[0;0;-1]; %unit vector pointing in upwards direction on body
u1 = -dot(Fdes,zB);

% Find desired body axes
zB_des = Fdes/norm(Fdes);
xC_des = [cos(yawDes); sin(yawDes); 0];
yB_des = cross(zB_des,xC_des)/norm(cross(zB_des,xC_des));
xB_des = cross(yB_des,zB_des);
R_des = [xB_des, -yB_des, -zB_des];
R_des2 = invar2rotmat('x',pi)*[xB_des, yB_des, zB_des];

% Define error on orientation
eR_matrix = (R_des'*R' - R*R_des);
eR = 0.5*[eR_matrix(3,2) - eR_matrix(2,3); ...
          eR_matrix(1,3) - eR_matrix(3,1); ...
          eR_matrix(2,1) - eR_matrix(1,2)];

% Angular velocity error
angVelDes = -Katt_P*eR;
e_omega = state(4:6) - angVelDes;

% error rates and integrals
if iSim == timeInit
    errAngVelIntegral = [0;0;0];
    errAngVelDeriv = [0;0;0];    
else
    errAngVelIntegral = e_omega + errAngVelPrev;
    errAngVelDeriv = (e_omega - errAngVelPrev)/tStep;
end

Control.integralErrAngVel =  Control.integralErrAngVel + errAngVelIntegral*tStep*0.5;


moments = - Kattrate_P*e_omega - Kattrate_I*Control.integralErrAngVel - Kattrate_D*errAngVelDeriv;

u2 = moments(1);
u3 = moments(2);
u4 = moments(3);

%% Generate Control Signal
%Thrust and Moment Control Signal
u = [u1;u2;u3;u4];

%Propeller RPM Control Signal
temp = u2RpmMat*u;
rpmsquare = temp.*(temp>0);
% rpm = sqrt(rpmsquare);
rpm = max(min(sqrt(rpmsquare),8000),3000); %saturate motor speeds
% rpm = max(min(sqrt(rpmsquare),7500),3500); %saturate motor speeds
% rpm = max(min(sqrt(rpmsquare),7800),3600); %saturate motor speeds
% rpm = motorDir*[-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM
rpm = [-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM


%% Assign values to output Control
Control.rpm = rpm;
Control.u = [u1;u2;u3;u4];
Control.errEuler = eR;
Control.errAngVel = e_omega;
Control.errPosnDeriv = errPosnDeriv;

end

