function [Control] = initcontrol()
    global m g

    [Pose, Twist] = initposetwist;
    % Desired pose and twist 
    Control.pose = Pose;
    Control.twist = Twist;
    
    % Desired acceleration in body frame, used during recovery
    Control.acc = [0; 0; 0];
    
    % Errors
    Control.errQuat = [0; 0; 0; 0];
    
    % Errors for controlleratt & controllerposn
    Control.errEuler = [0;0;0];
    Control.errAltitude = 0;
    Control.errAltitudeDeriv = 0; 
    Control.errYawDeriv = 0;
    Control.errPosnMag = 0;
    
    % Cumulative errors
    Control.integralErrEuler = [0;0;0];
    Control.integralErrAltitude = 0;
    Control.integralErrAltitudeDeriv = 0;
    Control.integralErrYawDeriv = 0;
    Control.integralErrPosnMag = 0;
    
    % Desired attitude
    Control.desEuler = [0; 0; 0];
    Control.desYawDeriv = 0;

    % Control force and torque outputs in body frame
    Control.u = [-m*g; 0; 0; 0];

    % Control propellor speeds outputs (1, 2, 3, 4)
    Control.rpm = [0; 0; 0; 0];
    Control.rpmDeriv = [0; 0; 0; 0];
    
    % controllerposn: 'posn'
    % controlleratt: 'att'
    % controllerrecovery: 'recovery'
    Control.type = 'na';
    
    
    
end