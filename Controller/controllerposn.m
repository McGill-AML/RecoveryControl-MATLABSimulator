function Control = controllerposn(state,iSim,timeInit,tStep,yawDes,Control)
%controllerposn.m High-level position controller
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Based on controller from --- 
%                Zhang, Mingfeng, Adam Harmat, and Inna Sharf.
%                "Autonomous Flight of a Quadrotor Using Multi-Camera Visual SLAM."
%                Int. Conf. on Intelligent Unmanned Systems. Vol. 10. 2014.
%-------------------------------------------------------------------------%

global m g Ixx Iyy Izz u2RpmMat


%% Save inputs 
posnDes = Control.pose.posn;

errAltitudePrev = Control.errAltitude;
errAltitudeDerivPrev = Control.errAltitudeDeriv;

errPosnMagPrev = Control.errPosnMag;

errAttRollPrev = Control.errEuler(1);
errAttPitchPrev = Control.errEuler(2);
errAttYawPrev = Control.errEuler(3);
errAttYawDerivPrev = Control.errYawDeriv;


%% Determine Current Attitude
q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
R = quat2rotmat(q);
[roll, pitch, yaw] = quat2angle(q,'xyz');


%% Zhang 2014 Controller
%% Altitude Controller Parameters
Kpz = 2;%20; %Zhang x4 value = 1
Kiz = 0.05;%40;
Kdz = 0.4;

Kpvz = 1.6; %Zhang x4 value = 2.8
Kivz = 0;%0.16;%60; %Zhang x4 value = 4

% altitudeDerivSaturation = 3; %Zhang x4 value = 0.6

%% Horizontal Position Controller Parameters
Kps = 1;%0.6; %Zhang x4 value = 0.6
Kis = 0.1;
Kpvx = 2; %Zhang x4 value = 2
Kpvy = 2; %Zhang x4 value = 2

Kpyaw = 1.2;%Zhang x4 value = 0.7
Kiyaw = 0;%1;
Kdyaw = 0.2;

% posnDerivSaturation = 2.5; %Zhang x4 value = 1
% rollSaturation = 1;%0.2 %Zhang x4 value = 0.1
% pitchSaturation = 1;%0.2; %Zhang x4 value = 0.1
yawDerivSaturation = pi/4; %Zhang x4 value = 0.3

%% Attitude Controller Parameters
% Original Simulator Values:
% Kprp = 7.2; %Zhang x4 value = 7.2
% Kirp = 4; %Zhang x4 value = 4
% Kdrp = 4.2; %Zhang x4 value = 4.2
% 
% Kpvyaw = 1.8; %Zhang x4 value = 2.8
% Kivyaw = 2; %Zhang x4 value = 4;
% Kdvyaw = 0; %Zhang x4 value = 0;

% Spiri Experimental Values:
% Kprp = 60;
% Kirp = 0;
% Kdrp = 6;
% 
% Kpvyaw = 1.8;
% Kivyaw = 0.2;
% Kdvyaw = 0;

Kprp = 60;
Kirp = 0;
Kdrp = 12;

Kpvyaw = 1.8;
Kivyaw = 0.2;
Kdvyaw = 0;


    %% PID Altitude Controller
    errAltitude = posnDes(3) - state(9);

    if iSim == timeInit
        errAltitudeIntegral = 0;
        errAltitudeDeriv = 0;    
    else
        errAltitudeIntegral = errAltitude + errAltitudePrev;
        errAltitudeDeriv = errAltitude - errAltitudePrev;
    end
    
    Control.integralErrAltitude = Control.integralErrAltitude + errAltitudeIntegral*tStep*0.5;
    altitudeDerivDes = Kpz*errAltitude + Kiz*Control.integralErrAltitude + Kdz*(errAltitudeDeriv)/tStep;

    % if v_des < 0
    %     v_des = max([-altitudeDerivSaturation,v_des]);
    % else
    %     v_des = min([v_des,altitudeDerivSaturation]);
    % end

    errAltitudeDeriv = altitudeDerivDes - R(:,3)'*state(1:3);
    if iSim == timeInit
        errAltitudeDerivPrev = -errAltitudeDeriv;
    end

    Control.integralErrAltitudeDeriv = Control.integralErrAltitudeDeriv + (errAltitudeDerivPrev+errAltitudeDeriv)*tStep*0.5;
    altitudeAcc = Kpvz*errAltitudeDeriv + Kivz*Control.integralErrAltitudeDeriv;
    altitudeAcc = R(3,3)*(altitudeAcc+g);

    %% PI Horizontal Position and Heading Controller

    % Roll & Pitch
    errPosnX = posnDes(1) - state(7);
    errPosnY = posnDes(2) - state(8);
    errPosnMag = sqrt(errPosnX^2 + errPosnY^2);
    
    if iSim == timeInit
        errPosnMagIntegral = 0;
    else
        errPosnMagIntegral = errPosnMag + errPosnMagPrev;
    end
    
    Control.integralErrPosnMag = Control.integralErrPosnMag + errPosnMagIntegral*tStep*0.5;
    posnDerivDes = Kps*errPosnMag + Kis*Control.integralErrPosnMag;
    
    % if posnDerivDes < 0
    %     posnDerivDes = max([-posnDerivSaturation,posnDerivDes]);
    % else
    %     posnDerivDes = min([posnDerivDes,posnDerivSaturation]);
    % end

    posnXDerivDes = posnDerivDes*cos(atan2(errPosnY,errPosnX));
    posnYDerivDes = posnDerivDes*sin(atan2(errPosnY,errPosnX));

    errPosnDerivX = posnXDerivDes - R(:,1)'*state(1:3);
    errPosnDerivY = posnYDerivDes - R(:,2)'*state(1:3);

    posnXAcc = Kpvx*errPosnDerivX;
    posnYAcc = Kpvy*errPosnDerivY;
    posnAcc = [posnXAcc;posnYAcc;0];

    posnXAcc = R(1,:)*posnAcc;
    posnYAcc = R(2,:)*posnAcc;
    altitudeAcc2 = R(3,:)*posnAcc;
    u1 = m*(altitudeAcc+altitudeAcc2);

    % pitch_des = -(cos(yaw)*ax + sin(yaw)*ay)/g;
    % roll_des = -(sin(yaw)*ax - cos(yaw)*ay)/g;

    rollDes = posnYAcc/g;
    pitchDes = -posnXAcc/g;

    % if pitch_des < 0
    %     pitch_des = max([-pitchSaturation,pitch_des]);
    % else
    %     pitch_des = min([pitch_des,pitchSaturation]);
    % end
    % 
    % if roll_des < 0
    %     roll_des = max([-rollSaturation,roll_des]);
    % else
    %     roll_des = min([roll_des,rollSaturation]);
    % end

    % Heading
    errAttYaw = yawDes - yaw;

    if errAttYaw > pi
        errAttYaw = -(2*pi - errAttYaw);
    end

    % 
    % if abs(errAttYaw) > pi
    %     Kpyaw = -Kpyaw;
    % %     Kiyaw = -Kiyaw;
    % %     Kdyaw = -Kdyaw;
    % end

    if iSim == timeInit
        errYawIntegral = 0;
        errYawDeriv = 0;
    else
        errYawIntegral = errAttYaw + errAttYawPrev;
        errYawDeriv = errAttYaw - errAttYawPrev;
    end
    Control.integralErrEuler(3) = Control.integralErrEuler(3) + tStep*errYawIntegral*0.5;
    attYawDerivDes = Kpyaw*errAttYaw + Kiyaw*Control.integralErrEuler(3) + Kdyaw*(errYawDeriv)/tStep;
    
  
    if attYawDerivDes < 0
        attYawDerivDes = max([-yawDerivSaturation,attYawDerivDes]);
    else
        attYawDerivDes = min([attYawDerivDes,yawDerivSaturation]);
    end

% 
%% Attitude Controller

errAttRoll = rollDes - roll;
errAttPitch = pitchDes - pitch;
errAttYawDeriv = attYawDerivDes - state(6);

if iSim == timeInit
    errRollIntegral = 0;
    errPitchIntegral = 0;
    errYawDerivIntegral = 0;

    errRollDeriv = 0;
    errPitchDeriv = 0;
    errYawDerivDeriv = 0;
else
    errRollIntegral = errAttRoll + errAttRollPrev;
    errPitchIntegral = errAttPitch + errAttPitchPrev;
    errYawDerivIntegral = errAttYawDeriv + errAttYawDerivPrev;

    errRollDeriv = errAttRoll - errAttRollPrev;
    errPitchDeriv = errAttPitch - errAttPitchPrev;
    errYawDerivDeriv = errAttYawDeriv - errAttYawDerivPrev;
end

Control.integralErrEuler(1) = Control.integralErrEuler(1) + tStep*errRollIntegral*0.5;
Control.integralErrEuler(2) = Control.integralErrEuler(2) + tStep*errPitchIntegral*0.5;
Control.integralErrYawDeriv = Control.integralErrYawDeriv + tStep*errYawDerivIntegral*0.5;

rollDerivDes = Kprp*errAttRoll + Kirp*Control.integralErrEuler(1) + Kdrp*(errRollDeriv)/tStep;
pitchDerivDes = Kprp*errAttPitch + Kirp*Control.integralErrEuler(2) + Kdrp*(errPitchDeriv)/tStep;
yawDerivDerivDes =  Kpvyaw*errAttYawDeriv + Kivyaw*Control.integralErrYawDeriv + Kdvyaw*(errYawDerivDeriv)/tStep;

u2 = (rollDerivDes - state(5)*state(6)*(Iyy-Izz)/Ixx)*Ixx;
u3 = (pitchDerivDes - state(4)*state(6)*(Izz-Ixx)/Iyy)*Iyy;
u4 = (yawDerivDerivDes - state(4)*state(5)*(Ixx-Iyy)/Izz)*Izz;

%% Generate Control Signal
%Thrust and Moment Control Signal
u = [u1;u2;u3;u4];


%Propeller RPM Control Signal
temp = u2RpmMat*u;
rpmsquare = temp.*(temp>0);
rpm = sqrt(rpmsquare);
% rpm = max(min(sqrt(rpmsquare),8000),3000); %saturate motor speeds
rpm = [-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM

% % Saturate propeller acceleration
% 
% prop_accel = zeros(4,1);
% prop_accel(1) = min(abs(omegadot(1)),prop_accel_sat)*sign(omegadot(1));
% prop_accel(2) = min(abs(omegadot(2)),prop_accel_sat)*sign(omegadot(2));
% prop_accel(3) = min(abs(omegadot(3)),prop_accel_sat)*sign(omegadot(3));
% prop_accel(4) = min(abs(omegadot(4)),prop_accel_sat)*sign(omegadot(4));
% 
% % Recalculate prop speed based on saturated propeller acceleration
% omegadot = prop_accel;
% omega_rad = omegadot*tStep + omega_prev_rad;
% omega = omega_rad * (60/(2*pi));


%% Assign values to output Control
Control.rpm = rpm;
Control.errAltitude = errAltitude;
Control.errAltitudeDeriv = errAltitudeDeriv;
Control.errPosnMag = errPosnMag;
Control.errEuler = [errAttRoll; errAttPitch; errAttYaw];
Control.errYawDeriv = errAttYawDeriv;
Control.twist.angVel(3) = attYawDerivDes;
Control.u = [u1;u2;u3;u4];

Control.desEuler = [rollDes; pitchDes; yawDes];
Control.desYawDeriv = attYawDerivDes;


end

