tic

% clearvars accelref
clear all;

VxImpact = 2;
pitchImpact = -35; 
yawImpact = 0;

global g mag
global timeImpact
global globalFlag

%% Initialize Fuzzy Logic Process
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 2;
tStep = 1/200;%1/200;

ImpactParams.wallLoc = 0.5;%1.5;
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

sensParams = initsensor_params;

[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

ImpactIdentification = initimpactidentification;

%% Set initial Conditions

%%%%%%%%%%%% ***** SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%%%%
Control.twist.posnDeriv(1) = VxImpact; %World X Velocity at impact      %%%
IC.attEuler = [0;0;0]; %[deg2rad(0);deg2rad(pitchImpact);deg2rad(yawImpact)];     %%%
IC.posn = [ImpactParams.wallLoc-0.2;0;5];                               %%%
Setpoint.posn(3) = IC.posn(3);                                          %%%
xAcc = 0;                                                               %%%
%%%%%%%%%%% ***** END SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%

rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

% [IC.posn(1), VxImpact, SimParams.timeInit, xAcc ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);
SimParams.timeInit = 0; %% comment out if using getinitworldx()

Setpoint.head = IC.attEuler(3);
Setpoint.time = 1; %SimParams.timeInit;
Setpoint.posn(1) = IC.posn(1)+5;
Trajectory = Setpoint;

IC.linVel =  [0;0;0]; %rotMat*[VxImpact;0;0];

Experiment.propCmds = [];
Experiment.manualCmds = [];

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);

[IC.rpm, Control.u] = initrpm(rotMat, [xAcc;0;0]); %Start with hovering RPM

PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, xAcc);
[Pose, Twist] = updatekinematics(state, stateDeriv);

%% Initialize sensors
Sensor = initsensor(state, stateDeriv, sensParams); % init sensor values
sensParams = initgps_baro(Sensor, sensParams); %init initial GPS and baro in order to initialize cartesian coord at starting spot

%% Initialize state estimators
EKF = initEKF;
AEKF = initAEKF;
SPKF = initSPKF;
ASPKF = initASPKF;

%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                sensParams, EKF, AEKF, SPKF, ASPKF);

% Initialize Continuous History
if SimParams.recordContTime == 1 
    ContHist = initconthist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, ...
                            PropState, Contact, globalFlag, Sensor);
end

%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)    
    

    
    %% Impact Detection    
    [ImpactInfo, ImpactIdentification] = detectimpact(iSim, ImpactInfo, ImpactIdentification,...
                                                      Sensor,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD);
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
        Control.recoveryStage = 0;
        Control.desEuler = IC.attEuler;
        Control.pose.posn(3) = Trajectory(end).posn(3);
        Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Control,[]);
        Control.type = 'att';
    end
    
    
    %% Propagate Dynamics
    options = getOdeOptions();
    [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                            tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                            Experiment.propCmds),[iSim iSim+tStep],state,options);
    
    % Reset contact flags for continuous time recording        
    globalFlag.contact = localFlag.contact;
    
     %% Update Sensors
    [Sensor,sensParams] = measurement_model(state, stateDeriv, sensParams, tStep);
    
    
    %% State Estimation
    SPKF = SPKF_attitude(Sensor, SPKF, EKF, sensParams, tStep);
    ASPKF = ASPKF_attitude(Sensor, ASPKF, AEKF, sensParams, tStep);
    
    EKF = EKF_position(Sensor, EKF, SPKF, Hist.SPKF(end).X_hat.q_hat, sensParams, tStep, iSim);
    AEKF = AEKF_position(Sensor, AEKF, ASPKF, Hist.ASPKF(end).X_hat.q_hat, sensParams, tStep, iSim);
    
    
    
    %% Record History
    if SimParams.recordContTime == 0
        
        [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:), ...
                                                         tStep,Control.rpm,ImpactParams, PropState.rpm, ...
                                                         Experiment.propCmds);        
        if sum(globalFlag.contact.isContact)>0
            Contact.hasOccured = 1;
            if sensParams.crash.new == 1
                sensParams.crash.time_since = 0;
                sensParams.crash.new = 0;
            end
                
            sensParams.crash.occur = 1;
            
            if ImpactInfo.firstImpactOccured == 0
                ImpactInfo.firstImpactOccured = 1;
            end
        else
            sensParams.crash.new = 1;
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
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF);

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

toc

%% Generate plottable arrays
Plot = hist2plot(Hist);


font_size = 15;
line_size = 15;
line_width = 2;