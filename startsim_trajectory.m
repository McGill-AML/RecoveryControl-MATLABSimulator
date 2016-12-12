%startsim_trajectory.m Prescribe quadrotor trajectory via position
%waypoints
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Prescribe quadrotor trajectory via position waypoints
%-------------------------------------------------------------------------%

clear all;

global g
global timeImpact
global globalFlag

%% Initialize Fuzzy Logic Process
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 10;
tStep = 1/100;%1/200;

ImpactParams.wallLoc = 5;%1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5; %Desired time of impact. Does nothing
ImpactParams.frictionModel.muSliding = 0.3;
ImpactParams.frictionModel.velocitySliding = 1e-4; %m/s
timeImpact = 10000;
timeStabilized = 10000;

%% Initialize Structures
IC = initIC;
Control = initcontrol;
PropState = initpropstate;
Setpoint = initsetpoint;

[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

ImpactIdentification = initimpactidentification;

%% Set initial Conditions
IC.posn = [0;0;2];  
IC.angVel = [0;0;0];
IC.attEuler = [0;0;0];
IC.linVel = [0;0;0];
SimParams.timeInit = 0;
rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

Experiment.propCmds = [];
Experiment.manualCmds = [];

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);

[IC.rpm, Control.u] = initrpm(rotMat, [0;0;0]); %Start with hovering RPM

PropState.rpm = IC.rpm;

%% Waypoint Trajectory
% Can add/delete waypoints as desired

% Setpt 1
Setpoint.head = 0;
Setpoint.time = 0;
Setpoint.posn = [0;0;2];
Trajectory = Setpoint;

% Setpt 2
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 2;
Setpoint.posn = [1;0;2];
Trajectory = [Trajectory;Setpoint];

% Setpt 3
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 2;
Setpoint.posn = [1;1;2];
Trajectory = [Trajectory;Setpoint];

% Setpt 4
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 2;
Setpoint.posn = [0;1;2];
Trajectory = [Trajectory;Setpoint];

% Setpt 5
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 2;
Setpoint.posn = [0;0;2];
Trajectory = [Trajectory;Setpoint];

iTrajectory = 1;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, 0);
[Pose, Twist] = updatekinematics(state, stateDeriv);

%% Initialize sensors
Sensor = initsensor(state, stateDeriv);

%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);

% Initialize Continuous History
if SimParams.recordContTime == 1 
    ContHist = initconthist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, ...
                            PropState, Contact, globalFlag, Sensor);
end

%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)    
    %% Loop through waypoints
    if iSim > Trajectory(iTrajectory).time
        if iTrajectory + 1 <= numel(Trajectory)
            iTrajectory = iTrajectory + 1;            
        end
    end  
    %% Update Sensors
    rotMat = quat2rotmat(Pose.attQuat);
    Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
    Sensor.gyro = Twist.angVel;
    
    %% Impact Detection    
    [ImpactInfo, ImpactIdentification] = detectimpact(iSim, tStep, ImpactInfo, ImpactIdentification,...
                                                      Hist.sensors,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv, state);
    [FuzzyInfo] = fuzzylogicprocess(iSim, ImpactInfo, ImpactIdentification,...
                                    Sensor, Hist.poses(end), SimParams, Control, FuzzyInfo);
                                
    % Calculate accelref in world frame based on FuzzyInfo.output, estWallNormal
    if sum(FuzzyInfo.InputsCalculated) == 4 && Control.accelRefCalculated == 0;
            disp(FuzzyInfo.output);
            Control.accelRef = calculaterefacceleration(FuzzyInfo.output, ImpactIdentification.wallNormalWorld);
            disp(Control.accelRef);
            Control.accelRefCalculated = 1;
    end
    
    %% Control
    if Control.accelRefCalculated*SimParams.useRecovery == 1 %recovery control       
        if SimParams.useFaesslerRecovery == 1  
            
            Control = checkrecoverystage(Pose, Twist, Control, ImpactInfo);
            [Control] = computedesiredacceleration(Control, Twist);

            % Compute control outputs
            [Control] = controllerrecovery(tStep, Pose, Twist, Control);       
            Control.type = 'recovery';
            
        else %Setpoint recovery
            disp('Setpoint recovery');
            Control.pose.posn = [0;0;2];
            Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
            
            Control.type = 'posn';
            
%             Control = controlleratt(state,iSim,SimParams.timeInit,tStep,2,[0;deg2rad(20);0],Control,timeImpact, manualCmds)
        end
    else %normal posn control
        Control.pose.posn = Trajectory(iTrajectory).posn;
        Control.recoveryStage = 0;
        Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
        
        Control.type = 'posn';
    end
    
    
    %% Propagate Dynamics
    options = getOdeOptions();
    [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                            tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                            Experiment.propCmds),[iSim iSim+tStep],state,options);
    
    % Reset contact flags for continuous time recording        
    globalFlag.contact = localFlag.contact;
    
    
    if SimParams.recordContTime == 0
        
        [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:), ...
                                                         tStep,Control.rpm,ImpactParams, PropState.rpm, ...
                                                         Experiment.propCmds);        
        if sum(globalFlag.contact.isContact)>0
            Contact.hasOccured = 1;
            if ImpactInfo.firstImpactOccured == 0
                ImpactInfo.firstImpactOccured = 1;
            end
        end

    else      
        % Continuous time recording
        for j = 1:size(stateODE,1)
            [stateDeriv, Contact, PropState] = dynamicsystem(tODE(j),stateODE(j,:), ...
                                                             tStep,Control.rpm,ImpactParams, PropState.rpm, ...
                                                             Experiment.propCmds);            
            if sum(globalFlag.contact.isContact)>0
                Contact.hasOccured = 1;
                if ImpactInfo.firstImpactOccured == 0
                    ImpactInfo.firstImpactOccured = 1;
                end
            end     
                      
            ContHist = updateconthist(ContHist,stateDeriv, Pose, Twist, Control, PropState, Contact, globalFlag, Sensor); 
        end
        
        ContHist.times = [ContHist.times;tODE];
        ContHist.states = [ContHist.states,stateODE'];    
    end
    
    localFlag.contact = globalFlag.contact;     
    state = stateODE(end,:)';
    t = tODE(end);
    [Pose, Twist] = updatekinematics(state, stateDeriv);

    %Discrete Time recording @ 200 Hz
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);

    %% End loop conditions
    % Navi has crashed:
    if state(9) <= 0
        display('Navi has hit the floor :(');
        ImpactInfo.isStable = 0;
        break;
    end  
    
    % Navi has drifted very far away from wall:
    if state(7) <= -10
        display('Navi has left the building');
        ImpactInfo.isStable = 1;
        break;
    end
    
    % Recovery control has worked, altitude stabilized:
    if Control.recoveryStage == 4
        display('Altitude has been stabilized');
        ImpactInfo.isStable = 1;
        break;
    end
end

%% Generate plottable arrays
Plot = hist2plot(Hist);
