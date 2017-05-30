global g timeImpact globalFlag poleRadius numTrials

VxImpact = 2.0;
yawImpact = 0.0;
rollImpact = 0.0;
poleRadius = 0.1; 
SimParams.useRecovery = 1;
Batch = [];
record = [];
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();
numOffset = 71;
numPitch = 46;
elapsedTime = 0;
for iPitch = 1:numPitch 
    pitchImpact = 1 - iPitch; 
    tic
    for iOffset=1:numOffset
        recoverySuccessful = 0;
        disp(numOffset*(iPitch-1)+iOffset);
        offset = -1+2*((iOffset-1)/(numOffset-1));
        offset_meters = 0.35*offset;
        ImpactParams = initparams_navi;
        SimParams.recordContTime = 0;
        SimParams.timeFinal = 3.0;
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
        IC.posn = [-0.4;offset_meters;2];  
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
                    [Control, recoverySuccessful] = checkrecoverystage(Pose, Twist, Control, ImpactInfo, recoverySuccessful);
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
        end

        Plot = hist2plot(Hist);
        Trial = {offset, pitchImpact, ...
                 recoverySuccessful, ImpactIdentification.wallNormalWorld, ...
                 Plot.times, Plot.posns, Plot.defls, ...
                 Plot.recoveryStage, Hist.states, Plot.normalForces, timeImpact}; 
             
        Batch = [Batch;Trial];
        elapsedTime = toc + elapsedTime
    end
end

save('iterate_with_recovery.mat','Batch');
% %%
% close all
% animate(0,1,Hist,'XY',ImpactParams,timeImpact,'NA',300);
% plot(Plot.times,abs(Plot.propRpms))