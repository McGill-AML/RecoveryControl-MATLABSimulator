% Monte Carlo simulation of a quadrotor impacting a wall

% Maintained by Gareth Dicker (dicker.gareth@gmail.com) and Fiona Chui
% July 2016
% Control method based on Faessler's 're-initialization' paper

% Notes for authors: 
%       - no fuzzy logic yet included, so reference acceleration is g away
%       - uncertainty about max RPM acceleration 
%       - uncertainty about friction coefficient
%       - uncertainty about geometry of impact (electronics cap on Navi is
%       not represented in this simulation)
%% Initialize Simulation Parameters

clear all
global m g Kt timeImpact globalFlag

ImpactParams = initparams_navi;

SimParams.recordContTime = 0; % irrelevant
SimParams.useFaesslerRecovery = 1; % irrelevant
SimParams.useRecovery = 1;  % irrelevant

% length of simulation
SimParams.timeInit = 0;
SimParams.timeFinal = 2;

% same as pixhawk control loop @ 200 Hz
tStep = 1/200;

%% Monte Carlo #1 with 1000 trials

% Random ICs:
%               -72 deg < pitch < 18 deg ---- this was initially supposed
%               to be -45 < pitch < 45
%               -15 deg < roll  < 15 deg
%               -45 deg < yaw   < 45 deg
%             1 m/s < incoming speed < 2 m/s

% Zero initial body rates, friction coefficient constant @ 0.3
% RPM acceleration saturated (liberally) at 70,000 rpm/s
% Body rate control gains P = 20 for {p,q} and P = 2 for {r}, no D terms
% Thrust coefficient = 8.7e-8
% Drag coefficient   = 8.7e-9  (one tenth of thrust, arbitrary)

% Arbitrary switch conditions specified in /Controller/checkrecoverystage

% Recorded data:
%               ICs 
%               percentage of trials to reach each stage of recovery
%               histograms of height and horizontal losses

% Analysis to include identification of correlations between specific
% ICs and failure cases

num_iter = 1000;

% Initialize Monte Carlo Struct which holds all histories of each trial
IC = initIC; % dummy initialization
Monte = initmontecarlo(IC);

% keeping friction constant for now, based on sample bumper/wall friction test
ImpactParams.frictionModel.muSliding = 0.3; % 0.2 - 0.4 possible

%% Run Monte Carlo Simulation
tic
for k = 1:num_iter
    display(k);
    
    ImpactParams.wallLoc = 0.3; % as close as possible so that impact ICs are same as when simulation starts
    ImpactParams.wallPlane = 'YZ';
    
    ImpactParams.timeDes = 0.5; % irrelevant
    ImpactParams.frictionModel.velocitySliding = 1e-4; % m/s
    timeImpact = 10000; % irrelevant

    % Initialize Structures
    IC = initIC;
    Control = initcontrol;
    PropState = initpropstate;
    Setpoint = initsetpoint;
    [Contact, ImpactInfo] = initcontactstructs;
    localFlag = initflags; % for contact analysis, irrelevant

    %% Set initial Conditions
    IC.attEuler = [deg2rad(30*(rand-0.5));deg2rad(90*(rand-0.8));deg2rad(90*(rand-0.5))];
    % picked arbitrary height of 5 meters
    IC.posn = [0;0;5];
    % 1 to 2 m/s into the wall
    IC.linVel = [rand+1;0;0];
    IC.friction = ImpactParams.frictionModel.muSliding;
    % rotation matrix
    rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

    % not used --->
    Experiment.propCmds = [];
    Experiment.manualCmds = []; 
    globalFlag.experiment.rpmChkpt = zeros(4,1);
    globalFlag.experiment.rpmChkptIsPassed = zeros(1,4); % <-------

    %Start with hovering RPM ---- is this overkill?
    IC.rpm = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  
    PropState.rpm = IC.rpm; 

    % Initialize state and kinematics structs from ICs
    [state, stateDeriv] = initstate(IC);
    [Pose, Twist] = updatekinematics(state, stateDeriv);

    % Initialize sensors
    Sensor = initsensor(rotMat, stateDeriv, Twist);

    % Initialize History Arrays
    Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);

    %% Simulation Loop
    for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep

        % ! not using  ------>
        rotMat = quat2rotmat(Pose.attQuat);
        Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
        Sensor.gyro = Twist.angVel; % <------

        %% Control
        % Before impact
        if ImpactInfo.firstImpactOccured == 0
            % give all four rpms equal divided by incoming angle cosine
            % don't know why the following three lines cause error if
            % removed ----->
            Control = checkrecoverystage(Pose, Twist, Control, ImpactInfo);
            Control = computedesiredacceleration(Control, Twist);    
            Control = controllerrecovery(tStep, Pose, Twist, Control); %<-----  
            
            % set rpms constant until impact
            Control.rpm = IC.rpm;
        else
            % after impact, perform recovery stages
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
    % add relevant info to Monte struct for plotting
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
histogram(Plot.heightLoss,num_iter/2)
% histogram(Plot.horizLoss,num_iter/2)


% plot how long each trial took to reach stage three
histogram(Plot.timeUntilStageTwo, num_iter/2);
% plot how long each trial took to reach stage three
% histogram(Plot.timeUntilStageThree, num_iter/2);

%%
% Plot XZ trajectories
% hold on;
% for k = 1:num_iter
%     plot(Plot.times,Monte(3,:,k))
% end

animate(0,Hist,'ZX',ImpactParams,timeImpact,[])

% compute speed at impact 
%  Plot.posnDerivs(:,vlookup(Plot.times,timeImpact))

% REPLAY A GIVEN TRIAL WITH SAME ICs

%% 
% Plot = hist2plot(Hist);
% plot(Plot.times, Plot.angVels);
