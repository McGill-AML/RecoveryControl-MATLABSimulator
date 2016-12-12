%startsim_matchexpoffboard.m Main script for running quadrotor simulation to match
%experimental Crash Set VII
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Main script for running quadrotor simulation to match
%                experimental Crash Set VII. Before the collision, attitude
%                control is used. After the collision, control switches to
%                controlleroffboard, which mimics the unstable/suboptimal
%                PX4 offboard controller used during experiments. 
%-------------------------------------------------------------------------%
clear all;

crash = 'VII-15';

global g
global timeImpact
global globalFlag

%% Initialize Fuzzy Logic Process
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 0;%Use Faessler recovery
SimParams.useRecovery = 1;
SimParams.timeFinal = 2;
tStep = 1/200;%1/200;

ImpactParams.wallLoc = 1.5;%1.5;
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

[VxImpact, IC.attEuler,~,~, ~, ~] = matchexperimentIC(crash);
[~, Setpoint] = matchoffboardsetpoints(crash);
IC.posn(3) = Setpoint.posn(3);

%%%%%%%%%%%% ***** SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%%%%
Control.twist.posnDeriv(1) = VxImpact; %World X Velocity at impact      %%%
% IC.attEuler = [deg2rad(0);deg2rad(pitchImpact);deg2rad(yawImpact)];     %%%
IC.posn(1) = ImpactParams.wallLoc-0.32;                               %%%
% Setpoint.posn(3) = IC.posn(3);                                          %%%
xAcc = 0;                                                               %%%
%%%%%%%%%%% ***** END SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%

rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

% [IC.posn(1), VxImpact, SimParams.timeInit, xAcc ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);
SimParams.timeInit = 0; %% comment out if using getinitworldx()

% Setpoint.head = deg2rad(-9.19+180);
% Setpoint.time = 2; %not used
% Setpoint.posn(1:2) = [1.5;0];
Trajectory = Setpoint;
iTrajectory = 1;

Experiment.propCmds = [];
Experiment.manualCmds = [];

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);


IC.linVel =  rotMat*[VxImpact;0;0];
[IC.rpm, Control.u] = initrpm(rotMat, [xAcc;0;0]); %Start with hovering RPM

PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, xAcc);
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
    
    
    %% Impact Detection    
    [ImpactInfo, ImpactIdentification] = detectimpact(iSim, tStep, ImpactInfo, ImpactIdentification,...
                                                      Hist.sensors,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv,state);
    [FuzzyInfo] = fuzzylogicprocess(iSim, ImpactInfo, ImpactIdentification,...
                                    Sensor, Hist.poses(end), SimParams, Control, FuzzyInfo);
                                
    % Calculate accelref in world frame based on FuzzyInfo.output, estWallNormal
    if sum(FuzzyInfo.InputsCalculated) == 4 && Control.accelRefCalculated == 0;
            Control.accelRef = calculaterefacceleration(FuzzyInfo.output, ImpactIdentification.wallNormalWorld);
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
%             disp('Offboard Control Engaged');
            Control.pose.posn = Trajectory(iTrajectory).posn;
            Control.pose.attEuler(3) = Trajectory(iTrajectory).head;
            Control.twist.linVel = [0;0;0];
            Control.twist.worldAcc = [0;0;0];
            Control.twist.angVel = [0;0;0];

            Control.recoveryStage = 0;
            Control = controlleroffboard(state,stateDeriv,iSim,SimParams.timeInit,tStep,Control);
    %         Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Control.pose.attEuler(3),Control);
            Control.type = 'offboard';
        end
    else %normal posn control
        Control.recoveryStage = 0;
        Control.desEuler = IC.attEuler;
        Control.pose.posn(3) = Trajectory(end).posn(3);
        Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Control,Experiment.manualCmds);
        Control.type = 'att';
    end
    
%     if iSim >= timeImpact +  1.085258
%         disp('Disarmed');
%         Control.rpm = [0;0;0;0];
%         Control.type = 'disarmed';
%     end
    
    %% Propagate Dynamics
    options = getOdeOptions();
    [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                            tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                            Experiment.propCmds),[iSim iSim+tStep],state,options);
    
    % Reset contact flags for continuous time recording        
    globalFlag.contact = localFlag.contact;
    
       
    %% Record History
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
    
    %% Update Sensors
    Sensor = updatesensor( state, stateDeriv );

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
