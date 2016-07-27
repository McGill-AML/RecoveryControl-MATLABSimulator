% Full maneuver based on Crash 10

tic

clear all

global m g Kt 
global timeImpact
global globalFlag

%% Initialize Simulation Parameters
initparams;

SimParams.recordContTime = 0;
SimParams.useRecovery = 0;
SimParams.timeFinal = 45;
tStep = 1/60;

ImpactParams.wallLoc = 1000;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 1.5;%0.5;
ImpactParams.compliantModel.e = 0.9;
ImpactParams.compliantModel.k = 40;
ImpactParams.compliantModel.n = 0.54;
timeImpact = 10000;

%% Initialize Structures
IC = initIC;
Control = initcontrol;
PropState = initpropstate;
Setpoint = initsetpoint;

[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

%% Match initial conditions to experiment
 expCrash = 10;
 [Control.twist.posnDeriv(1), IC.attEuler, IC.posn(3), Setpoint.posn(3), xAcc, Experiment] = matchexperimentIC(expCrash);
% 
% 
% rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');
% 
% [IC.posn(1), initialLinVel, SimParams.timeInit ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);
% 
% Setpoint.head = IC.attEuler(3);
% Setpoint.time = SimParams.timeInit;
% Setpoint.posn(1) = IC.posn(1);
% Trajectory = Setpoint;

% IC.linVel =  rotMat*[initialLinVel;0;0];
IC.rpm = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  %Start with hovering RPM

PropState.rpm = IC.rpm;

%only to test controller:
IC.posn = [0;0;0];%[0;0;0];    
IC.angVel = [0;0;0];
IC.attEuler = [0;0;0];
IC.linVel = [0;0;0];
SimParams.timeInit = 0;

%% Crash Maneuver
% %First setpoint
% Setpoint.head = pi;
% Setpoint.time = 0;
% Setpoint.posn = IC.posn;
% Trajectory = Setpoint;
% 
% %Second setpoint
% Setpoint.head = pi;
% Setpoint.time = 5;
% Setpoint.posn = [0;0;0.6];
% Trajectory = [Trajectory;Setpoint];

%% Waypoint Maneuver
% Setpt 1
Setpoint.head = 0;
Setpoint.time = 0;
Setpoint.posn = IC.posn;
Trajectory = Setpoint;

%% Setpt 2
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;0;0];
Trajectory = [Trajectory;Setpoint];

%% Setpt 2
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;0;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 3
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [2;0;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 4
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [2;1;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 5
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;1;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 6
Setpoint.head = 0;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;0;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 7
Setpoint.head = pi/2;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;0;1];
Trajectory = [Trajectory;Setpoint];

%% Setpt 8
Setpoint.head = pi/2;
Setpoint.time = Setpoint.time + 5;
Setpoint.posn = [0;0;0];
Trajectory = [Trajectory;Setpoint];


%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC);
[Pose, Twist] = updatekinematics(state, stateDeriv);


%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag);

% Initialize Continuous History
if SimParams.recordContTime == 1 
    ContHist = initconthist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, ...
                            PropState, Contact, globalFlag);
end

iTrajectory = 2;
t = SimParams.timeInit;
crashIntoWall = 0;

%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)       
    
    if crashIntoWall == 0
        if t > Trajectory(iTrajectory).time
            if iTrajectory + 1 <= size(Trajectory,1)
                iTrajectory = iTrajectory + 1;
            else
                crashIntoWall = 1;
            end
        end    
    end
    
    crashIntoWall = 0;
    
    if crashIntoWall == 0
        Control.pose.posn = Trajectory(iTrajectory).posn;
        Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(iTrajectory).head,Control);
        Control.type = 'posn';
    else
        altitudeDes = 0.66;
        attitudeDes = [deg2rad(-1.2);deg2rad(16);pi + deg2rad(4)];

        Control.pose.posn(3) = altitudeDes;
        Control.desEuler = attitudeDes;
        Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Control,Experiment.manualCmds);   
        Control.type = 'att';
    end
    
    %Propagate Dynamics
    options = odeset('RelTol',1e-3);
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
        end  

    else  
    
        % Continuous time recording
        for j = 1:size(stateODE,1)
            [stateDeriv, Contact, PropState] = dynamicsystem(tODE(j),stateODE(j,:), ...
                                                             tStep,Control.rpm,ImpactParams, PropState.rpm, ...
                                                             Experiment.propCmds);
            
            if sum(globalFlag.contact.isContact)>0
                Contact.hasOccured = 1;
            end     
                      
            ContHist = updateconthist(ContHist,stateDeriv, Pose, Twist, Control, PropState, Contact, globalFlag); 
        end
    ContHist.times = [ContHist.times;tODE];
    ContHist.states = [ContHist.states,stateODE'];    
    end
    
    localFlag.contact = globalFlag.contact;     
    state = stateODE(end,:)';
    t = tODE(end);
    [Pose, Twist] = updatekinematics(state, stateDeriv);

    %Discrete Time recording @ 200 Hz
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag);

%     %End loop if Spiri has crashed
%     if state(9) <= 0 && t > 0
%         display('Spiri has hit the floor :(');
%         ImpactInfo.isStable = 0;
%         break;
%     end  
    
end

toc

% Get impact info
if SimParams.recordContTime == 0
    for iBumper = 1:4
        ImpactInfo.bumperInfos(iBumper) = getsiminfo(Hist,iBumper, Trajectory(1).head);
    end
else
    for iBumper = 1:4
        ImpactInfo.bumperInfos(iBumper) = getsiminfo(ContHist,iBumper,Trajectory(1).head);
    end
end

cellBumperInfos = struct2cell(ImpactInfo.bumperInfos);

ImpactInfo.maxNormalForce = max([cellBumperInfos{find(strcmp(fieldnames(ImpactInfo.bumperInfos),'maxNormalForces')),:}]);
ImpactInfo.maxDefl = max([cellBumperInfos{find(strcmp(fieldnames(ImpactInfo.bumperInfos),'maxDefls')),:}]);
ImpactInfo.numContacts = sum([cellBumperInfos{find(strcmp(fieldnames(ImpactInfo.bumperInfos),'numContacts')),:}]);

Plot = hist2plot(Hist);
copy2ros = [[1:1:size(Plot.times)]', ones(size(Plot.times)), Plot.times, Plot.posns',Plot.quaternions'];