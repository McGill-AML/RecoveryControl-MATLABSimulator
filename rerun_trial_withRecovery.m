
clear all

global timeImpact globalFlag poleRadius numImpacts FirstNormalExpected 

VxImpact = 2.0;
yawImpact = 0.0;
rollImpact = 0.0;
poleRadius = 0.1; 
SimParams.useRecovery = 1;
Batch = [];
record = [];
NumImpactTimeline=[];
impactSwitchRecorder=[];
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();
numOffset = 71;
numPitch = 46;
elapsedTime = 0;


for iPitch = 1:numPitch
    pitchImpact = 1 - iPitch;
    yawImpact = 0.0;
    rollImpact = 0.0;
    for iOffset=1:numOffset
        before_Ref=[];
        FirstNormalExpected=zeros(4,3);
        NumImpactTimeline =[];
        impactSwitchRecorder=[];
        numImpacts=0;
        recoverySuccessful = 0;
        disp(numOffset*(iPitch-1)+iOffset);
        offset = -1+2*((iOffset-1)/(numOffset-1));
        offset_meters = 0.35*offset;
        ImpactParams = initparams_navi;
        SimParams.recordContTime = 0;
        SimParams.timeFinal = 2.0;
        tStep = 1/200;
        ImpactParams.wallLoc = 0.0;
        ImpactParams.wallPlane = 'YZ';
        ImpactParams.timeDes = 0.5; 
        ImpactParams.frictionModel.muSliding =0.3;
        ImpactParams.frictionModel.velocitySliding =1e-4; %m/s
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
        IC.posn = [-0.4; offset_meters; 1];
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

        bodyFrame_ZAxisDesired = quatrotate(Pose.attQuat', [0 0 -1]);
        tempGlobalFlag=[0;0;0;0];  % added for to check number of collisions
        for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep   
            
            [ImpactInfo, ImpactIdentification] = detectimpact(iSim, tStep, ImpactInfo, ImpactIdentification,...
                                                              Hist.sensors,Hist.poses,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv,state);
                                                          
            if (ImpactInfo.firstImpactDetected && Control.accelRefCalculated == 0)
                    
                
                ImpactIdentification.wallNormalWorldCorrected2;
                ActualAcc=[Twist.worldAcc(1);Twist.worldAcc(2);0]/norm([Twist.worldAcc(1);Twist.worldAcc(2);0]);
                line = [ImpactIdentification.wallNormalWorldCorrected2(1:2); ...
                        -sum(ImpactIdentification.wallNormalWorldCorrected2(1:2).*state(7:8))];
                before_Ref = [Hist.states(7:8,1);1]
                after_Ref  = [0;0;0];
                after_Ref(1)  = before_Ref(1) - (2*line(1)/norm(line(1:2))^2)*(dot(line,before_Ref));
                after_Ref(2)  = before_Ref(2) - (2*line(2)/norm(line(1:2))^2)*(dot(line,before_Ref));
                after_Ref = after_Ref+[state(7:8);0];
                after_Ref = after_Ref/norm(after_Ref) ...
                    +[ 2*abs(ImpactIdentification.wallNormalWorldCorrected2(2));0;0];   % provide recoil component in IC velocity Direction
                after_Ref = after_Ref/norm(after_Ref);
                Control.accelRef = 4.8*after_Ref;
                Control.accelRefCalculated = 1;                  
            end

            if Control.accelRefCalculated*SimParams.useRecovery == 1    
                if Control.recoveryStage == 0
                    Control.recoveryStage = 1;
                end
                    [Control] = computedesiredacceleration(Control, Twist);
                    [Control] = controllerrecovery(tStep, Pose, Twist, Control);   
                    [Control, recoverySuccessful] = checkrecoverystage(Pose, Twist, Control, ImpactInfo, recoverySuccessful);
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
                                                                     tStep,Control.rpm,ImpactParams, ...
                                                                     PropState.rpm, ...
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
            incrementedOnce=0;
          
          for i=1:4      % to keep track of number of impacts with the pole
              if tempGlobalFlag(i)~= globalFlag.contact.isContact(i)
                  if tempGlobalFlag(i)==0 && globalFlag.contact.isContact(i)==1 && incrementedOnce==0
                      numImpacts=numImpacts+1;
                      incrementedOnce=1;
                  end
                  tempGlobalFlag(i)= globalFlag.contact.isContact(i);
              end
          end
          impactSwitchRecorder = [impactSwitchRecorder; globalFlag.contact.isContact'];
          NumImpactTimeline = [NumImpactTimeline; numImpacts];
        end
        if SimParams.useRecovery == 0
            bodyFrame_ZAxis = quatrotate(Pose.attQuat', [0 0 -1]);
            Theta = acos(dot(bodyFrame_ZAxis, bodyFrame_ZAxisDesired));
            if Theta == 0
                errQuaternion = [1 0 0 0]';
            else
                n = cross(bodyFrame_ZAxis, bodyFrame_ZAxisDesired);
                n = n/norm(n);
                nBody = quatrotate(quatinv(Pose.attQuat)',n);
                errQuaternion = real([cos(Theta/2)         ; nBody(1)*sin(Theta/2); ...
                                      nBody(2)*sin(Theta/2); nBody(3)*sin(Theta/2)]);
            end
            worldVelocity=rotMat*(Twist.linVel);
            SWITCH = abs(sin(Theta/2))<= 0.25 ... % 30 degree bound wrt the IC
                           && abs(Twist.attEulerRate(1)) < 2.0  ...
                           && abs(Twist.attEulerRate(2)) < 2.0  ...
                           && abs(worldVelocity(3)) < 0.2             ... 
            if SWITCH 
                recoverySuccessful = 1;
            end
        end
        Plot = hist2plot(Hist);
        Trial = {offset, pitchImpact, ...
                 recoverySuccessful, ImpactIdentification.wallNormalWorldCorrected2, ...
                 sum(FirstNormalExpected)/norm(sum(FirstNormalExpected)),ActualAcc, Plot.times, Plot.posns, Plot.defls, ...
                 Plot.recoveryStage, Hist.states, Plot.normalForces, ...
                 timeImpact,impactSwitchRecorder,numImpacts}; 
        
        elapsedTime = toc + elapsedTime
        Batch = [Batch; Trial];
    end
end


save('June 29 _With Recovery Controller','Batch');
%%
% close all
% % % for iter=1
%         animate(1,1,Hist,'XY',ImpactParams,timeImpact,'without',200, NumImpactTimeline);
% end
% plot(Plot.times,abs(Plot.propRpms))
