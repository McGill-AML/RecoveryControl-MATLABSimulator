function [state, stateDeriv, PropState, IC, Twist, Pose, Control, Setpoint, Hist] = initstructs()
    
    % Initializes state and control structs as well as history
    % structs used for plotting results
    % Updated January 2016
    
    global m g

    % Define structures 
    %state stateDeriv Twist Pose Control Hist 
    
    % Initialize full state of quadrotor 
    % Index map:
    % [1 2 3 4 5 6 7 8 9 10 11 12 13]
    % [u v w p q r x y z q0 q1 q2 q3]
    state = zeros(13,1);

    % The derivative of the state, used in state propagation
    stateDeriv = zeros(13,1);
    
    %%%%%%%%%%%%%%%%%%%
    % Propeller State %
    %%%%%%%%%%%%%%%%%%%%    
    PropState.rpm = zeros(4,1);
    PropState.rpmDeriv = zeros(4,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Define Twist struct %
    %%%%%%%%%%%%%%%%%%%%%%%

    % Linear velocity in body frame
    Twist.linVel = [0; 0; 0];
    
    % Linear velocity in the WORLD frame
    Twist.posnDeriv = [0; 0; 0];

    % Angular velocities in body frame
    Twist.angVel = [0; 0; 0];

    % Euler angle rates
    Twist.attEulerRate = [0; 0; 0];

    %%%%%%%%%%%%%%%%%%%%%%
    % Define Pose struct %
    %%%%%%%%%%%%%%%%%%%%%%

    % Position in world frame
    Pose.posn = [0; 0; 0];

    % Attitude in Euler angles 
    Pose.attEuler = [0; 0; 0];

    % Attitude in quaternion form
    Pose.attQuat = [0; 0; 0; 0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define Initial Condition Struct %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    IC.posn = [0;0;0];    
    IC.angVel = [0;0;0];
    IC.attEuler = [0;0;0];
    IC.linVel = [0;0;0];
    IC.rpm = [0;0;0;0];

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Define Control struct %
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Desired pose and twist 
    Control.pose = Pose;
    Control.twist = Twist;
    
    % Desired acceleration in body frame, used during recovery
    Control.acc = [0; 0; 0];
    
    % Error quaternion between roll/pitch and desired roll/pitch
    Control.errQuat = [0; 0; 0; 0];
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
    
    Setpoint.posn = [0; 0; 0];
    Setpoint.head = 0;
    Setpoint.time = 0; 

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Define History struct %
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %% to move to after ICs separate function
    
    % Initialize history of the state and its derivative
    Hist.state = state;
    Hist.stateDeriv = stateDeriv;

    % Initialize history of twist, pose and control structs
    Hist.twist = Twist;
    Hist.pose = Pose;
    Hist.control = Control;

end