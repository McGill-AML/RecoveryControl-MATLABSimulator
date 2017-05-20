global g timeImpact globalFlag poleRadius numTrials

VxImpact = 2.0;
yawImpact = 0.0;
pitchImpact = -15; 
rollImpact = 0.0;
poleRadius = 0.1; 
SimParams.useRecovery = 0;
Batch = [];
record = [];
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();
numTrials = 1000;
elapsedTime = 0;
for iBatch = 1:numTrials 
    tic
    disp(iBatch)
    offset = -1+2*((iBatch-1)/numTrials);
    offset_meters = 0.35*offset;
    ImpactParams = initparams_navi;
    SimParams.recordContTime = 0;
    SimParams.timeFinal = 1.0;
    tStep = 1/200;
    ImpactParams.wallLoc = 0.0;
    ImpactParams.wallPlane = 'YZ';
    ImpactParams.timeDes = 0.5; 
    ImpactParams.frictionModel.muSliding = 0.3;%0.3;
    ImpactParams.frictionModel.velocitySliding = 1e-4; %m/s
    timeImpact = 10000;
    timeStabilized = 10000;
    IC = initIC;
    Control = initcontrol;
    PropState = initpropstate;
    Setpoint = initsetpoint;
    [Contact, ImpactInfo] = initcontactstructs;
    localFlag = initflags;
    ImpactIdentification = initimpactidentification;
    Control.twist.posnDeriv(1) = VxImpact; 
    IC.attEuler = [deg2rad(rollImpact);deg2rad(pitchImpact);deg2rad(yawImpact)];  
    IC.posn = [-0.45;offset_meters;2];  
    Setpoint.posn(3) = IC.posn(3); 
    xAcc = 0;                                                               
    rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');
    SimParams.timeInit = 0; 
    Setpoint.head = IC.attEuler(3);
    Setpoint.time = SimParams.timeInit;
    Setpoint.posn(1) = IC.posn(1);
    Trajectory = Setpoint;
    IC.linVel =  rotMat*[VxImpact;0;0];
    Experiment.propCmds = [];
    Experiment.manualCmds = [];
    globalFlag.experiment.rpmChkpt = zeros(4,1);
    globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);
    [IC.rpm, Control.u] = initrpm(rotMat, [xAcc;0;0]); 
    PropState.rpm = IC.rpm;
    [state, stateDeriv] = initstate(IC, xAcc);
    [Pose, Twist] = updatekinematics(state, stateDeriv);
    Sensor = initsensor(state, stateDeriv);
    Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);
    
    for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep   
       
        [ImpactInfo, ImpactIdentification] = detectimpact(iSim, tStep, ImpactInfo, ImpactIdentification,...
                                                          Hist.sensors,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv,state);
        [FuzzyInfo] = fuzzylogicprocess(iSim, ImpactInfo, ImpactIdentification,...
                                        Sensor, Hist.poses(end), SimParams, Control, FuzzyInfo);

        if (sum(FuzzyInfo.InputsCalculated) == 4 && Control.accelRefCalculated == 0)
                Control.accelRef = 4.8*ImpactIdentification.wallNormalWorld;
                Control.accelRefCalculated = 1;
                
        end
        
        if Control.accelRefCalculated*SimParams.useRecovery == 1       
                Control = checkrecoverystage(Pose, Twist, Control, ImpactInfo);
                [Control] = computedesiredacceleration(Control, Twist);
                [Control] = controllerrecovery(tStep, Pose, Twist, Control);       
                Control.type = 'recovery';
        else 
            Control.recoveryStage = 0;
            Control.desEuler = IC.attEuler;
            Control.pose.posn(3) = IC.posn(3);
            Control = controlleratt(state,iSim,SimParams.timeInit,tStep,Control,[]);
            Control.type = 'att';
        end
        
        options = getOdeOptions();
        [tODE,stateODE] = ode23(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                                tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                                Experiment.propCmds),[iSim iSim+tStep],state,options);

        globalFlag.coninitpropstatetact = localFlag.contact;

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
        Sensor = updatesensor(state, stateDeriv);
        Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);

        % Navi has crashed
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

    
    end

    Plot = hist2plot(Hist);
    CrashData.Plot = Plot;
%     CrashData.ImpactInfo = ImpactInfo;
    CrashData.ImpactIdentification = ImpactIdentification;
%     CrashData.Hist = Hist;
    CrashData.ImpactParams = ImpactParams;
    CrashData.timeImpact = timeImpact;
    CrashData.offset = offset;
%     CrashData.contact = Contact;
    Batch = [Batch; CrashData];
    elapsedTime = toc + elapsedTime
end

save('1000_trials_may_20.mat','Batch');

%%
%  animate(0,3,Hist,'XY',ImpactParams,timeImpact,'NA',100);
