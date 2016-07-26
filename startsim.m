%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Monte Carlo Simulation of Crash Recovery using Fuzzy Logic  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set 1:
% ICs:
%   -pitch +/- 60 deg
%   -roll  +/- 60 deg
%   -0 < yaw < 45 deg
%   Notes: weird correlation to failure with positive high roll.
%   Saved file: 'pitch_60_roll_60.mat'

% Set 2:
% ICs:
%   -pitch +/- 60 deg
%   -roll  +/- 15 deg
%   -0 < yaw < 45 deg
%   Saved file: 'pitch_60_roll_15.mat'

% For sets 1 and 2:
%   1000 trials. 
%   RPM max acceleration 70,000 rpm/s. 
%   min/max RPM 1000/8000.

% Set 3:
%   -pitch +/- 60 deg
%   -roll  +/- 15 deg
%   -0 < yaw < 45 deg

%   RPM max acceleration 27,000 rpm/s, other conditions same. 

% For all trials:
%   Thrust coefficient: 8.7e-8
%   Drag coefficient:   8.7e-9
%   Friction coefficient: 0.3
%   Angle error to body rate gain: 15.0
%   Proportional gains only for body rate control (20 for {p,q}, 2 for {r})
%   Fuzzy logic output between -1 and 1 multiplied by 9.81 for Control.accelRef

% See /Controller/checkrecoverystage.m for recovery stage switch conditions
% See /Controller/controllerrecovery.m for recovery method
% See /Results/plot_monte.m for plotting results
% See /Fuzzy\Logic/initfuzzylogicprocess.m for fuzzy logic parameters

tic
clear all;

global g timeImpact globalFlag
 
ImpactParams = initparams_navi;
 
SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 2;
tStep = 1/200;
 
num_iter = 1000;
 
IC = initIC; % dummy initialization
Monte = initmontecarlo(IC);
 
for k = 1:num_iter
    display(k);
 
    ImpactParams.frictionModel.muSliding = 0.3; % 0.2 - 0.4 possible
    ImpactParams.wallLoc = 0.0; % as close as possible so that impact ICs are same as when simulation starts
    ImpactParams.wallPlane = 'YZ';
    ImpactParams.timeDes = 0.5; % irrelevant
    ImpactParams.frictionModel.velocitySliding = 1e-4; % m/s
    timeImpact = 10000; % irrelevant
 
    IC = initIC;
    Control = initcontrol;
    PropState = initpropstate;
    Setpoint = initsetpoint;
    [Contact, ImpactInfo] = initcontactstructs;
    localFlag = initflags; % for contact analysis, irrelevant
 
    % Initialize Fuzzy Logic Process
    [FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();
    

    % rotation matrix
    ImpactIdentification = initimpactidentification;
    
    % Randomized ICs    
    % World X velocity at impact
    x_velocity = rand*1.5 + 0.5;
    Control.twist.posnDeriv(1) = x_velocity;  
    % Incoming pitch +/- 50 deg, roll +/- 60 deg, incoming yaw 0 to 45 deg
    IC.attEuler = [deg2rad(120*(rand-0.5));deg2rad(100*(rand-0.5));deg2rad(45*rand)];     %%%
    % starts next to the wall 5 meter up
    IC.posn = [-0.32; 0; 5];                             
    Setpoint.posn(3) = IC.posn(3);                                        
    xAcc = 0; %don't change                                                
    
    rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');
 
    SimParams.timeInit = 0; 
    Setpoint.head = IC.attEuler(3);
    Setpoint.time = SimParams.timeInit;
    Setpoint.posn(1) = IC.posn(1);
    Trajectory = Setpoint;
 
    IC.linVel =  rotMat*[x_velocity;0;0];
 
    Experiment.propCmds = [];
    Experiment.manualCmds = [];
 
    globalFlag.experiment.rpmChkpt = zeros(4,1);
    globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);
 
    [IC.rpm, Control.u] = initrpm(rotMat, [xAcc;0;0]); %Start with hovering RPM
 
    PropState.rpm = IC.rpm;
 
    % Initialize state and kinematics structs from ICs
    [state, stateDeriv] = initstate(IC, xAcc);
    [Pose, Twist] = updatekinematics(state, stateDeriv);
 
    % Initialize sensors
    Sensor = initsensor(rotMat, stateDeriv, Twist);
 
    % Initialize history 
    Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);
 
%     % Initialize Continuous History
%     if SimParams.recordContTime == 1 
%         ContHist = initconthist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, ...
%                                 PropState, Contact, globalFlag, Sensor);
%     end
 
    %% Simulation Loop
    for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
    %     display(iSim)    
 
        %% Update Sensors
        rotMat = quat2rotmat(Pose.attQuat);
        Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
        Sensor.gyro = Twist.angVel;
 
        %% Impact Detection    
        [ImpactInfo, ImpactIdentification] = detectimpact(iSim, ImpactInfo, ImpactIdentification,...
                                                          Sensor,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD);
        [FuzzyInfo] = fuzzylogicprocess(iSim, ImpactInfo, ImpactIdentification,...
                                        Sensor, Hist.poses(end), SimParams, Control, FuzzyInfo);
 
        % Calculate accelref in world frame based on FuzzyInfo.output, estWallNormal
        if sum(FuzzyInfo.InputsCalculated) == 4 && Control.accelRefCalculated == 0;
    %             disp(FuzzyInfo.output);
                Control.accelRef = calculaterefacceleration(FuzzyInfo.output, ImpactIdentification.wallNormalWorld);
%                 disp(Control.accelRef);
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
                Control.pose.posn = [0;0;Trajectory(end).posn(3)];
                Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
                Control.type = 'posn';            
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
 
%         if Control.recoveryStage == 4
% %             display('Altitude has been stabilized');
%             ImpactInfo.isStable = 1;
%             timeStabilized = iSim;
%             break;
%         end
    end
    Monte = updatemontecarlo(k, IC, Hist, Monte);
end
toc

%% Cut off the dummy first trial
Monte.trial = Monte.trial(2:end);
Monte.IC = Monte.IC(2:end);
Monte.recovery = Monte.recovery(2:end,:);
Monte.heightLoss = Monte.heightLoss(2:end);
Monte.horizLoss = Monte.horizLoss(2:end);
%% Convert to plottable info
Plot = monte2plot(Monte);

%% Plot results
% histogram(Plot.heightLoss,num_iter/2)
% histogram(Plot.horizLoss,num_iter/2)
% histogram(Plot.timeUntilStageTwo, num_iter/2);
  
%% Generate plottable arrays
Plot = hist2plot(Hist);
animate(0,Hist,'ZX',ImpactParams,timeImpact)

