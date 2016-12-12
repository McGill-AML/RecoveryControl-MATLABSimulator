function Control = controlleratt(state,iSim,timeInit,tStep,Control,manualCmds)
%controlleratt.m Low level attitude controller
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Based on controller from --- 
%                Zhang, Mingfeng, Adam Harmat, and Inna Sharf.
%                "Autonomous Flight of a Quadrotor Using Multi-Camera Visual SLAM."
%                Int. Conf. on Intelligent Unmanned Systems. Vol. 10. 2014.
%-------------------------------------------------------------------------%

global m g Ixx Iyy Izz u2RpmMat timeImpact

%% Save inputs 
altitudeDes = Control.pose.posn(3);
attRollDes = Control.desEuler(1);
attPitchDes = Control.desEuler(2);
attYawDes = Control.desEuler(3);

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
Kiz = 0;%0.05;%40;
Kdz = 0.4;

Kpvz = 1.6; %Zhang x4 value = 2.8
Kivz = 0;%0.16;%60; %Zhang x4 value = 4

% altitudeDerivSaturation = 3; %Zhang x4 value = 0.6

%% Heading Controller Parameters
Kpyaw = 1.2; %1.2
Kiyaw = 0; %1
Kdyaw = 0.2; %0.2

%% Attitude Controller Parameters
Kprp = 60;
Kirp = 0;
Kdrp = 12;

Kpvyaw = 1.8;
Kivyaw = 0;%0.2;
Kdvyaw = 0;

if timeImpact == 10000 || isempty(manualCmds)
    %% PID Altitude Controller
    errAltitude = altitudeDes - state(9);

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
    u1 = m*R(3,3)*(altitudeAcc+g);

    %% Heading Control

    % Heading
    errAttYaw = attYawDes - yaw;

    if errAttYaw > pi
        errAttYaw = -(2*pi - errAttYaw);
    end

    if iSim == timeInit
        errYawIntegral = 0;
        errYawDeriv = 0;
    else
        errYawIntegral = errAttYaw + errAttYawPrev;
        errYawDeriv = errAttYaw - errAttYawPrev;
    end

    Control.integralErrEuler(3) = Control.integralErrEuler(3) + tStep*errYawIntegral*0.5;
    attYawDerivDes = Kpyaw*errAttYaw + Kiyaw*Control.integralErrEuler(3) + Kdyaw*(errYawDeriv)/tStep; 
    
else% Use experiment joystick commands to match controller thrust and attitude commands
    for k = max(size(manualCmds)):-1:1
        if iSim >= timeImpact + manualCmds(k).time    
            u1 = manualCmds(k).thrust;
            attRollDes = manualCmds(k).attEuler(1);
            attPitchDes = manualCmds(k).attEuler(2);
            attYawDerivDes = manualCmds(k).angVel(3);

            errAltitude = 0;
            errAltitudeDeriv = 0;
            errAttYaw = 0;

            break    
        end
    end
end
    
%% Attitude Controller

errAttRoll = attRollDes - roll;
errAttPitch = attPitchDes - pitch;
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
rpm = max(min(sqrt(rpmsquare),8000),3000); %saturate motor speeds
rpm = [-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM

%% Assign values to output Control
Control.rpm = rpm;
Control.errAltitude = errAltitude;
Control.errAltitudeDeriv = errAltitudeDeriv;
Control.errEuler = [errAttRoll; errAttPitch; errAttYaw];
Control.errYawDeriv = errAttYawDeriv;
Control.twist.angVel(3) = attYawDerivDes;
Control.u = [u1;u2;u3;u4];

Control.desYawDeriv = attYawDerivDes;


end