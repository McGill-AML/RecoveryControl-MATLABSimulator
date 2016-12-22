
% clearvars -except SPKF ASPKF rmse loop_no timer Setpoint Trajectory gamma

global g mag
global timeImpact
global globalFlag

%% Initialize Fuzzy Logic Process
[FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput();

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1; 
SimParams.timeFinal = 20;
tStep = 1/100;%1/200;

ImpactParams.wallLoc = 0.5;%1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 0.5; %Desired time of impact. Does nothing
ImpactParams.frictionModel.muSliding = 0.3;
ImpactParams.frictionModel.velocitySliding = 1e-4; %m/s


%% Initialize Structures
IC = initIC;



sensParams = initsensor_params; % initialize sensor parameters for use in measurement model

% move_avg_acc = zeros(3,6);


localFlag = initflags;


%% Set initial Conditions
IC.posn = [ImpactParams.wallLoc-0.5;0;5];  
IC.angVel = [0;0;0];
IC.attEuler = [0;0;pi/5];
IC.linVel = [0;0;0];
SimParams.timeInit = 0;
rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');





%% Initialize sensors

sensParams = initgps_baro(Sensor, sensParams); %init initial GPS and baro in order to initialize cartesian coord at starting spot

Est_sensParams = initEst_sensPars(sensParams); %initialize sensor parameters for use in estimators (to add error, etc.)

%% Initialize state estimators
Est_ICs = initSE_ICs(IC, sensParams); % set initial condition estimate in order to add errors

EKF = initEKF(Est_ICs);
AEKF = initAEKF(Est_ICs);
SPKF = initSPKF(Est_ICs);
ASPKF = initASPKF(Est_ICs);
COMP = initCOMP(Est_ICs);
HINF = initHINF(Est_ICs);
SPKF_full = initSPKF_full(Est_ICs);
EKF_att = initEKF_att(Est_ICs);
SRSPKF = initSRSPKF(Est_ICs);
SRSPKF_full = initSRSPKF_full(Est_ICs);
ASPKF_opt = initASPKF_opt(Est_ICs);
AHINF = initAHINF(Est_ICs);



time_to_break = 0; %var so sim doesn't stop once it's stabilized
%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att,SRSPKF, SRSPKF_full, ASPKF_opt, AHINF);
            


%% Simulation Loop
for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep
%     display(iSim)    
    

    
   
    
%     [Sensor.acc, move_avg_acc] = moving_avg_filt(Sensor.acc, move_avg_acc);
    %% State Estimation
    tic;
    SPKF = SPKF_attitude(Sensor, SPKF, EKF, Est_sensParams, tStep);
    timer(1) = timer(1) + toc;
%     
    tic;
    ASPKF = ASPKF_attitude(Sensor, ASPKF, EKF, Est_sensParams, tStep);
    timer(2) = timer(2) + toc;
%     
    tic;
    EKF_att = EKF_attitude(Sensor, EKF_att, EKF, Est_sensParams, tStep);
    timer(3) = timer(3) + toc;
%     
    tic;
    SPKF_full = SPKF_full_state(Sensor, SPKF_full, Est_sensParams, tStep, iSim);
    timer(4) = timer(4) + toc;
%     
    tic;
    COMP = CompFilt_attitude(Sensor, COMP, EKF, Est_sensParams, tStep);
    timer(5) = timer(5) + toc;
%     
    tic;
    HINF = HINF_attitude(Sensor, HINF, EKF, Est_sensParams, tStep);
    timer(6) = timer(6) + toc;
    
%     tic;
%     SRSPKF = SRSPKF_attitude(Sensor, SRSPKF, EKF, Est_sensParams, tStep);
%     timer(7) = timer(7) + toc;
    
%     tic;
%     SRSPKF_full = SRSPKF_full_state(Sensor, SRSPKF_full, Est_sensParams, tStep, iSim);
%     timer(8) = timer(8) + toc;
    
    tic;
    ASPKF_opt = ASPKF_opt_attitude(Sensor, ASPKF_opt, AEKF, sensParams, tStep);
    timer(9) = timer(9) + toc;
    
    tic;
    AHINF = AHINF_attitude(Sensor, AHINF, EKF, Est_sensParams, tStep);
    timer(10) = timer(10) + toc;
%     
    EKF = EKF_position(Sensor, EKF, SPKF, Hist.SPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
%     AEKF = AEKF_position(Sensor, AEKF, ASPKF, Hist.ASPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
    

    
    
    [Pose, Twist] = updatekinematics(state, stateDeriv);

    %Discrete Time recording @ 200 Hz
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att, SRSPKF, SRSPKF_full, ASPKF_opt, AHINF);
                    
                    
                    
                    
end

toc

%% Generate plottable arrays
Plot = hist2plot(Hist);


font_size = 15;
line_size = 15;
line_width = 2;


