tic

clear all

global m g Kt 
global timeImpact
global globalFlag

%% Initialize Simulation Parameters
initparams;

SimParams.recordContTime = 0;
SimParams.useRecovery = 0;
SimParams.timeFinal = 1.2;
tStep = 1/200;

ImpactParams.wallLoc = 1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5;
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
expCrash = 11;
[Control.twist.posnDeriv(1), IC.attEuler, IC.posn(3), Setpoint.posn(3), xAcc, Experiment] = matchexperimentIC(expCrash);


rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

[IC.posn(1), initialLinVel, SimParams.timeInit ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);

Setpoint.head = IC.attEuler(3);
Setpoint.time = SimParams.timeInit;
Setpoint.posn(1) = IC.posn(1);
Trajectory = Setpoint;

IC.linVel =  rotMat*[initialLinVel;0;0];
IC.rpm = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  %Start with hovering RPM

PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC);
[Pose, Twist] = updatekinematics(state, stateDeriv);


%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag);

% Initialize Continuous History
if SimParams.recordContTime == 1 
    ContHist = initconthist(ContHist,SimParams.timeInit,state,Contact,PropState,stateDeriv,globalFlag); 
end


%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)    
   
    if Contact.hasOccured*SimParams.useRecovery == 1        
        Control.pose.posn = [0 Trajectory(end).posn(2:3)];
        Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
    else
        Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Setpoint.posn(3),IC.attEuler,Control,Contact.hasOccured,timeImpact, Experiment.manualCmds);
     
    end
    
    %Propagate Dynamics
    options = odeset('RelTol',1e-3);
    [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE,tStep,Control.rpm,ImpactParams,PropState.rpm,Experiment.propCmds),[iSim iSim+tStep],state,options);
    
    % Reset contact flags for continuous time recording        
    globalFlag.contact = localFlag.contact;
    
    
    if SimParams.recordContTime == 0
        
        [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:),tStep,Control.rpm,ImpactParams, PropState.rpm,Experiment.propCmds);
        
        if sum(globalFlag.contact.isContact)>0
            Contact.hasOccured = 1;
        end  

    else  
    
        % Continuous time recording
        for j = 1:size(stateODE,1)
            [stateDeriv, Contact, PropState] = dynamicsystem(tODE(j),stateODE(j,:),tStep,Control.rpm,ImpactParams, PropState.rpm,Experiment.propCmds);
            
            if sum(globalFlag.contact.isContact)>0
                Contact.hasOccured = 1;
            end     
                      
            ContHist = updateconthist(ContHist,Contact,PropState,stateDeriv,globalFlag); 
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

    %End loop if Spiri has crashed
    if state(9) <= 0
        display('Spiri has hit the floor :(');
        ImpactInfo.isStable = 0;
        break;
    end  
    
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

