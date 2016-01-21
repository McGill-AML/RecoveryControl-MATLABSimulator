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

    % Control force and torque outputs in body frame
    Control.u = [-m*g; 0; 0; 0];

    % Control propellor speeds outputs (1, 2, 3, 4)
    Control.rpm = [0; 0; 0; 0];
    Control.rpmDeriv = [0; 0; 0; 0];
    
    % Type 1: controllerposn
    % Type 2: controlleratt
    % Type 3: controllerrecovery
    Control.type = 0;
    
    
    
end