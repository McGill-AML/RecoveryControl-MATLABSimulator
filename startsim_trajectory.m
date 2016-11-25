
% clearvars -except SPKF ASPKF rmse loop_no timer Setpoint Trajectory gamma

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
SimParams.timeFinal = 20;
tStep = 1/100;%1/200;

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
% Setpoint = initsetpoint;

sensParams = initsensor_params; % initialize sensor parameters for use in measurement model

move_avg_acc = zeros(3,6);


[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

ImpactIdentification = initimpactidentification;

%% Set initial Conditions
IC.posn = [ImpactParams.wallLoc-0.5;0;5];  
IC.angVel = [0;0;0];
IC.attEuler = [0;0;pi/5];
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
% Setpt 1
% Setpoint.head = pi/5;
% Setpoint.time = 4;
% Setpoint.posn = [0;0;5];
% Trajectory = Setpoint;
% 
% % Setpt 2
% Setpoint.head = pi/5;
% Setpoint.time = Setpoint.time + 10;
% Setpoint.posn = [8;0;5];
% Trajectory = [Trajectory;Setpoint];

% % Setpt 3
% Setpoint.head = 0;
% Setpoint.time = Setpoint.time + 5;
% Setpoint.posn = [1;1;2];
% Trajectory = [Trajectory;Setpoint];
% 
% % Setpt 4
% Setpoint.head = 0;
% Setpoint.time = Setpoint.time + 5;
% Setpoint.posn = [0;1;2];
% Trajectory = [Trajectory;Setpoint];
% 
% % Setpt 5
% Setpoint.head = 0;
% Setpoint.time = Setpoint.time + 5;
% Setpoint.posn = [0;0;2];
% Trajectory = [Trajectory;Setpoint];

iTrajectory = 1;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, 0);
[Pose, Twist] = updatekinematics(state, stateDeriv);

%% Initialize sensors
Sensor = initsensor(state, stateDeriv, sensParams); % init sensor values
sensParams = initgps_baro(Sensor, sensParams); %init initial GPS and baro in order to initialize cartesian coord at starting spot

Est_sensParams = initEst_sensPars(sensParams); %initialize sensor parameters for use in estimators (to add error, etc.)

%% Initialize state estimators
Est_ICs = initSE_ICs(IC, sensParams); % set initial condition estimate in order to add errors

EKF = initEKF(Est_ICs);
AEKF = initAEKF(Est_ICs);
SPKF = initSPKF(Est_ICs);
ASPKF = initASPKF(Est_ICs);
COMP = initCOMP(Est_ICs);
HINF = initHINF(Est_ICs);
SPKF_full = initSPKF_full(Est_ICs);
EKF_att = initEKF_att(Est_ICs);
SRSPKF = initSRSPKF(Est_ICs);



time_to_break = 0; %var so sim doesn't stop once it's stabilized
%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att,SRSPKF);
            
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
    
        
    [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:), ...
                                                     tStep,Control.rpm,ImpactParams, PropState.rpm, ...
                                                     Experiment.propCmds);  
    
     %% Update Sensors
    [Sensor,sensParams] = measurement_model(state, stateDeriv, sensParams, tStep);
    
%     [Sensor.acc, move_avg_acc] = moving_avg_filt(Sensor.acc, move_avg_acc);
    %% State Estimation
    tic;
    SPKF = SPKF_attitude(Sensor, SPKF, EKF, Est_sensParams, tStep);
    timer(1) = timer(1) + toc;
    
    tic;
    ASPKF = ASPKF_attitude(Sensor, ASPKF, EKF, Est_sensParams, tStep);
    timer(2) = timer(2) + toc;
    
    tic;
    EKF_att = EKF_attitude(Sensor, EKF_att, EKF, Est_sensParams, tStep);
    timer(3) = timer(3) + toc;
    
    tic;
    SPKF_full = SPKF_full_state(Sensor, SPKF_full, Est_sensParams, tStep, iSim);
    timer(4) = timer(4) + toc;
    
    tic;
    COMP = CompFilt_attitude(Sensor, COMP, EKF, Est_sensParams, tStep);
    timer(5) = timer(5) + toc;
    
    tic;
    HINF = HINF_attitude(Sensor, HINF, EKF, Est_sensParams, tStep);
    timer(6) = timer(6) + toc;
    
    tic;
    SPKF = SRSPKF_attitude(Sensor, SRSPKF, EKF, Est_sensParams, tStep);
    timer(7) = timer(7) + toc;
%     
    EKF = EKF_position(Sensor, EKF, SPKF, Hist.SPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
%     AEKF = AEKF_position(Sensor, AEKF, ASPKF, Hist.ASPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
    

    
    
    
    %% Record History
    if SimParams.recordContTime == 0
        
        %moved this up a notch. otherwise the sensors were behind a
        %timestep
%         [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:), ...
%                                                          tStep,Control.rpm,ImpactParams, PropState.rpm, ...
%                                                          Experiment.propCmds);        
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
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att, SRSPKF);
                    
    %% End loop conditions
    % Navi has crashed:
    if state(9) <= 0
        display('Navi has hit the floor :(');
        ImpactInfo.isStable = 0;
        break;
    end  
    
    % Navi has drifted very far away from wall:
    if state(7) <= -30
        display('Navi has left the building');
        ImpactInfo.isStable = 1;
        break;
    end
    
    % Recovery control has worked, altitude stabilized:
    if Control.recoveryStage == 4 && time_to_break == 0
        time_of_recovery = iSim;
        time_to_break = iSim + 10;
    elseif iSim == time_to_break && iSim ~= 0
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


