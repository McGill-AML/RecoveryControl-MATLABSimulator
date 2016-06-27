tic

% clearvars -except Hist_mu0 Plot_mu0 Hist_mu1 Plot_mu1 Hist_mu2 Plot_mu2 Hist_mu3 Plot_mu3 Hist_mu4 Plot_mu4
clearvars accelref

global m g
global timeImpact
global globalFlag

%% Initialize Fuzzy Logic Process
intensityFuzzyProcess = initFuzzyLogicProcess();

%% Set up Calculation Delay Times
% PreImpact Attitude
PREIMPACT_ATT_CALCSTEPFWD = 2;

% FLP input 1: Accelerometer Horizontal Magnitude
fuzzyInput.ID = 1;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 0.010; %10 to 15 ms
fuzzyInput.value = 0;
fuzzyInputArray = fuzzyInput;

%FLP input 2: Inclination
fuzzyInput.ID = 2;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 0; %available at t_detection
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

%FLP input 3: Flipping Direction Angle
fuzzyInput.ID = 3;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 0.010; %5 to 10 ms
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

%FLP input 4: Gyro Horizontal Magnitude
fuzzyInput.ID = 4;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 0.010; %10 to 15 ms
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

fuzzyInputsCalculated = [0;0;0;0];


%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 2;
tStep = 1/200;%1/200;

ImpactParams.wallLoc = 1.5;%1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5;
ImpactParams.frictionModel.muSliding = 0;
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

%% Set initial Conditions

Control.twist.posnDeriv(1) = 1.5; %World X Velocity at impact
IC.attEuler = [deg2rad(0);deg2rad(-15);deg2rad(0)];
IC.posn = [0;0;4];
Setpoint.posn(3) = IC.posn(3);
xAcc = 0;

rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

[IC.posn(1), initialLinVel, SimParams.timeInit, xAcc ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);

Setpoint.head = IC.attEuler(3);
Setpoint.time = SimParams.timeInit;
Setpoint.posn(1) = IC.posn(1);
Trajectory = Setpoint;

IC.linVel =  rotMat*[initialLinVel;0;0];

Experiment.propCmds = [];
Experiment.manualCmds = [];

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);

[IC.rpm, Control.u] = initrpm(rotMat, [xAcc;0;0]); %Start with hovering RPM


PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, xAcc);
[Pose, Twist] = updatekinematics(state, stateDeriv);

%% Initialize sensors
Sensor = initsensor(rotMat, stateDeriv, Twist);

%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);

% Initialize Continuous History
if SimParams.recordContTime == 1 
    ContHist = initconthist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, ...
                            PropState, Contact, globalFlag, Sensor);
end

estForceExternalBodyArray = [0;0;0];
estForceExternalWorldArray = [0;0;0];

%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)    
    
    %% Sensors
    rotMat = quat2rotmat(Pose.attQuat);
    Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
    Sensor.gyro = Twist.angVel;
    
    %% Impact Detection
    
    % External Force Estimation
    estForceExternalBody = m*Sensor.accelerometer*g - [0;0;Control.u(1)];
    estForceExternalWorld = rotMat'*estForceExternalBody;
    estForceExternalBodyArray = [estForceExternalBodyArray,estForceExternalBody];
    estForceExternalWorldArray = [estForceExternalWorldArray,estForceExternalWorld];
    
    if ImpactInfo.firstImpactDetected == 0
        if norm(estForceExternalBody) >= 1 %External Force estimation
            ImpactInfo.firstImpactDetected = 1;
            timeImpactDetected = iSim;
            
            %pre-impact attitude
            rotMatPreImpact = quat2rotmat(Hist.poses(end-PREIMPACT_ATT_CALCSTEPFWD).attQuat);      
                        
            %wall location, i.e. e_n, e_N
            
            estWallNormalWorld = estForceExternalWorld/norm(estForceExternalWorld);
            estWallNormalBody = estForceExternalBody/norm(estForceExternalBody);         
         
        end
    end
    
    if SimParams.useRecovery == 1
    if ImpactInfo.firstImpactDetected == 1 && ImpactInfo.accelRefCalculated == 0%Calculate fuzzy inputs
        for iInput = 1:4
            if fuzzyInputsCalculated(iInput) == 0
                if fuzzyInputArray(iInput).calcStepDelay <= iSim - timeImpactDetected
                %waited enuff time to calculate
                    switch iInput 
                        case 1 %FLP input 1: Accelerometer Horizontal Magnitude
%                             fuzzyInputArray(iInput).value = norm(a_acc(1:2));
                            fuzzyInputArray(iInput).value = norm(estForceExternalBody);
                            disp('input 1 calc');
                        case 2 %FLP input 2: Inclination
                            estWallTangentWorld = cross([0;0;1],estWallNormalWorld);
                            negBodyZ = [0;0;-1];
                            bodyZProjection = rotMatPreImpact'*negBodyZ - dot((rotMatPreImpact'*negBodyZ),estWallTangentWorld)*estWallTangentWorld;
                            dotProductWithWorldZ = dot(bodyZProjection,[0;0;1]);
                            inclinationAngle = acos(dotProductWithWorldZ/norm(bodyZProjection));
                            
                            dotProductWithWorldNormal = dot(bodyZProjection,estWallNormalWorld);
                            angleWithWorldNormal = acos(dotProductWithWorldNormal/(norm(bodyZProjection)*norm(estWallNormalWorld)));                            
                            inclinationSign = sign(angleWithWorldNormal - pi/2);                            
                            
                            fuzzyInputArray(iInput).value = inclinationSign*rad2deg(inclinationAngle); 
                            disp('input 2 calc');
                        case 3 %FLP input 3: Flipping Direction Angle
                            %Try calculating according to world frame:
                            angVelWorld = rotMat'*Sensor.gyro;
                            angVelWorldPerp = cross(angVelWorld,[0;0;1]);
                            angVelWorldPerpHoriz = angVelWorldPerp(1:2);
                            wallNormalWorldHoriz = estWallNormalWorld(1:2);
                            fuzzyInputArray(iInput).value = rad2deg(acos(dot(angVelWorldPerpHoriz,wallNormalWorldHoriz)/...
                                                            (norm(angVelWorldPerpHoriz)*norm(wallNormalWorldHoriz))));
                            disp('input 3 calc');
                        case 4 %FLP input 4: Gyro Horizontal Magnitude
                            fuzzyInputArray(iInput).value = norm(Sensor.gyro(1:2));
                        disp('input 4 calc');
                    end
                    fuzzyInputsCalculated(iInput) = 1;
                end
            end
            
        end
        
        if sum(fuzzyInputsCalculated) == 4 %Inputs complete, now do the fuzzy logic
            temp = struct2cell(fuzzyInputArray);            
            inputFLP = [temp{3,:}];
            outputFLP = evalfis(inputFLP, intensityFuzzyProcess);
            if abs(outputFLP) <= eps
                outputFLP = 0;
            end
%             fuzzyOutput_array(iBatch) = outputFLP;

            % Calculate accelref in world frame based on outputFLP, estWallNormal
            accelref = calculaterefacceleration(outputFLP, estWallNormalWorld)
            ImpactInfo.accelRefCalculated = 1;
            recoveryStage = 1;
        end
    end
    end
    
    %% Control
    if ImpactInfo.accelRefCalculated*SimParams.useRecovery == 1 %recovery control       
        if SimParams.useFaesslerRecovery == 1  
%             disp('in Faessler control');
            
            [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage);

            [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage, accelref);

            % Compute control outputs
            [Control] = controllerrecovery(tStep, Pose, Twist, Control, Hist);            
            
            Control.type = 'recovery';
            
        else %Setpoint recovery
            disp('Setpoint recovery');
            Control.pose.posn = [0;0;2];
            Control = controllerposn(state,iSim,SimParams.timeInit,tStep,Trajectory(end).head,Control);
            
            Control.type = 'posn';
            
%             Control = controlleratt(state,iSim,SimParams.timeInit,tStep,2,[0;deg2rad(20);0],Control,timeImpact, manualCmds)
        end
    else %normal posn control
        recoveryStage = 0;
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
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, recoveryStage, Sensor);

    %% End loop if Spiri has crashed
    if state(9) <= 0
        display('Navi has hit the floor :(');
        ImpactInfo.isStable = 0;
        break;
    end  
    
    if state(7) <= -10
        display('Navi has left the building');
        ImpactInfo.isStable = 1;
        break;
    end
end

toc

%% Generate plottable arrays
Plot = hist2plot(Hist);
