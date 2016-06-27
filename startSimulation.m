tic
clear all
global m g Kt timeImpact globalFlag

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 2;
SimParams.timeInit = 0;
tStep = 1/200;

ImpactParams.wallLoc = 1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5;
ImpactParams.frictionModel.muSliding = 0.1;
ImpactParams.frictionModel.velocitySliding = 1e-4; %m/s
timeImpact = 10000;

%% Initialize Structures
IC = initIC;
Control = initcontrol;
PropState = initpropstate;
Setpoint = initsetpoint;

[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

%% Set initial Conditions
IC.attEuler = [0;deg2rad(0);deg2rad(0)];
IC.posn = [0;0;5];
IC.linVel = [1;0;0];
rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

% not used --->
Experiment.propCmds = [];
Experiment.manualCmds = []; 
globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4); % <-------

%Start with hovering RPM ---- is this overkill now?
IC.rpm = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  
PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC);
[Pose, Twist] = updatekinematics(state, stateDeriv);

%% Initialize sensors
Sensor = initsensor(rotMat, stateDeriv, Twist);

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
%     display(iSim);
    %% Sensors
    
    % ! not using  ------>
    rotMat = quat2rotmat(Pose.attQuat);
    Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
    Sensor.gyro = Twist.angVel; % <------
    
    %% Impact Detection
    if ImpactInfo.firstImpactDetected == 0
        if norm(Sensor.accelerometer(1:2)) >= 0.5 %Horizontal Accel data
            ImpactInfo.firstImpactDetected = 1;
            timeImpactDetected = iSim;
        end
    end
    %% Control

    % Before impact
    if ImpactInfo.firstImpactOccured == 0
        % go into wall at some angle
        Control.accelRef = [g/4; 0; 0];
        Control = computedesiredacceleration(Control, Twist); 
        %use recovery controller to impact the wall
        Control = controllerrecovery_preimpact(tStep, Pose, Twist, Control);  
        
    % After impact
    else
        Control = checkrecoverystage(Pose, Twist, Control, ImpactInfo);
        Control = computedesiredacceleration(Control, Twist);    
        Control = controllerrecovery(tStep, Pose, Twist, Control);   
    end
    
    %% Propagate dynamics
    options = getOdeOptions();
    [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                            tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                            Experiment.propCmds),[iSim iSim+tStep],state,options);
    %% Contact recording
    
    % Reset contact flags for continuous time recording        
    globalFlag.contact = localFlag.contact;
    
    % if NOT recording contact times
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
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);
end

%% Generate plottable arrays
Plot = hist2plot(Hist);

animate(0,Hist,'ZX',ImpactParams,timeImpact,[])

% compute speed at impact 
%  Plot.posnDerivs(:,vlookup(Plot.times,timeImpact))

