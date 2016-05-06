tic

% clear all
clearvars -except Hist_mu0 Plot_mu0 Hist_mu1 Plot_mu1 Hist_mu2 Plot_mu2 Hist_mu3 Plot_mu3 Hist_mu4 Plot_mu4

global m g Kt 
global timeImpact
global globalFlag

%% Initialize Simulation Parameters
initparams_spiri;

SimParams.recordContTime = 1;
SimParams.useRecovery = 0;
SimParams.timeFinal = 5;
tStep = 1/200;%1/200;

ImpactParams.wallLoc = 1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5;
ImpactParams.compliantModel.e = 0.9;
ImpactParams.compliantModel.k = 40;
ImpactParams.compliantModel.n = 0.54;
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

%% Match initial conditions to experiment
expCrash = 'A10';
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

% %only to test controller:
% IC.posn = [0;0;0];    
% IC.angVel = [0;0;0];
% IC.attEuler = [0;0;0];
% IC.linVel = [0;0;0];
% SimParams.timeInit = 0;

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


%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
    display(iSim)    
   
  
    if Contact.hasOccured*SimParams.useRecovery == 1        
        Control.pose.posn = [0 Trajectory(end).posn(2:3)];
        Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
        Control.type = 'posn';
    else
        Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Setpoint.posn(3), ... 
                                IC.attEuler,Control,Contact.hasOccured,timeImpact, Experiment.manualCmds);
        Control.type = 'att';
    end
       
    %Propagate Dynamics
    options = odeset('RelTol',1e-3,'AbsTol',[1e-5 1e-5 1e-5 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3]); %Default: RelTol 1e-3, AbsTol 1e-6
%     [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
%                                                             tStep,Control.rpm,ImpactParams,PropState.rpm, ...
%                                                             Experiment.propCmds),[iSim iSim+tStep],state,options);
    [tODE,stateODE] = ode113(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
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

Plot = hist2plot(Hist);
