
% clearvars -except SPKF ASPKF rmse loop_no SE_timer Setpoint Trajectory gamma
clear;

global g mag
global timeImpact
global globalFlag

loop_no = 1; %need to set for init functions - has no effect when useExpData = 1
useExpData = 1;

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



% move_avg_acc = zeros(3,6);


localFlag = initflags;


%% Set initial Conditions
IC.posn = [0;0;0];  
IC.angVel = [0;0;0];
IC.attEuler = [0;0;0];
IC.linVel = [0;0;0];
SimParams.timeInit = 0;
rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');

%% init stuff for using experimental data

initExtData; % script to import data from PX4log
mag = zeros(3,1);
useDataPts4Mag = 5;
mag(1) = mean(IMU_MagX(1:useDataPts4Mag));
mag(2) = mean(IMU_MagY(1:useDataPts4Mag));
mag(3) = mean(IMU_MagZ(1:useDataPts4Mag));

%switch to NED frame
mag(1) = norm([mag(1), mag(2)]);
mag(2) = 0;

%nomralize
mag = mag/norm(mag);


% init variables for sim to NaN so they throw errors if used. req'd for
% logging data
SimParams.timeInit= NaN;
state= NaN; 
stateDeriv = NaN; 
Pose= NaN; 
Twist= NaN; 
Control= NaN;
PropState= NaN; 
Contact= NaN; 
t = NaN;


%% Initialize sensors

GPS = zeros(5,1); % use because experiments didn't actually have gps data. in future could simulate gps using VISN x y z. 
BAR = 0;

Sensor = package_sens_data(IMU_AccX(1), IMU_AccY(1), IMU_AccZ(1), IMU_GyroX(1), IMU_GyroY(1), IMU_GyroZ(1), IMU_MagX(1), IMU_MagY(1), IMU_MagZ(1), GPS, BAR, 1); %init sensor

sensParams = initsensor_params(useExpData); % initialize sensor parameters for use in measurement model

% sensParams = initgps_baro(Sensor, sensParams); %init initial GPS and baro in order to initialize cartesian coord at starting spot

Est_sensParams = initEst_sensPars(sensParams); %initialize sensor parameters for use in estimators (to add error, etc.)

%% Initialize state estimators
Est_ICs = initSE_ICs(IC, sensParams, loop_no, useExpData); % set initial condition estimate in order to add errors

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
SPKF_norm = initSPKF_norm(Est_ICs, loop_no);

SE_timer = zeros(11,1);



time_to_break = 0; %var so sim doesn't stop once it's stabilized
%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att,SRSPKF, SRSPKF_full, ASPKF_opt, AHINF, SPKF_norm, useExpData);
            


%% Simulation Loop
for iSim = 2:length(TIME)
%     display(iSim)    
    tStep = TIME(iSim)-TIME(iSim-1);

    Sensor = package_sens_data(IMU_AccX(iSim), IMU_AccY(iSim), IMU_AccZ(iSim), IMU_GyroX(iSim), IMU_GyroY(iSim), IMU_GyroZ(iSim), IMU_MagX(iSim), IMU_MagY(iSim), IMU_MagZ(iSim), GPS, BAR);
   
    
%     [Sensor.acc, move_avg_acc] = moving_avg_filt(Sensor.acc, move_avg_acc);
    %% State Estimation
    tic;
    SPKF = SPKF_attitude(Sensor, SPKF, EKF, Est_sensParams, tStep);
    SE_timer(1) = SE_timer(1) + toc;
%     
    tic;
    ASPKF = ASPKF_attitude(Sensor, ASPKF, EKF, Est_sensParams, tStep);
    SE_timer(2) = SE_timer(2) + toc;
%     
    tic;
    EKF_att = EKF_attitude(Sensor, EKF_att, EKF, Est_sensParams, tStep);
    SE_timer(3) = SE_timer(3) + toc;
%     
%     tic;
%     SPKF_full = SPKF_full_state(Sensor, SPKF_full, Est_sensParams, tStep, iSim);
%     SE_timer(4) = SE_timer(4) + toc;
%     
    tic;
    COMP = CompFilt_attitude(Sensor, COMP, EKF, Est_sensParams, tStep);
    SE_timer(5) = SE_timer(5) + toc;
%     
%     tic;
%     HINF = HINF_attitude(Sensor, HINF, EKF, Est_sensParams, tStep);
%     SE_timer(6) = SE_timer(6) + toc;
    
%     tic;
%     SRSPKF = SRSPKF_attitude(Sensor, SRSPKF, EKF, Est_sensParams, tStep);
%     SE_timer(7) = SE_timer(7) + toc;
    
%     tic;
%     SRSPKF_full = SRSPKF_full_state(Sensor, SRSPKF_full, Est_sensParams, tStep, iSim);
%     SE_timer(8) = SE_timer(8) + toc;
    
%     tic;
%     ASPKF_opt = ASPKF_opt_attitude(Sensor, ASPKF_opt, AEKF, sensParams, tStep);
%     SE_timer(9) = SE_timer(9) + toc;
    
%     tic;
%     AHINF = AHINF_attitude(Sensor, AHINF, EKF, Est_sensParams, tStep);
%     SE_timer(10) = SE_timer(10) + toc;

%     tic;
%     SPKF_norm = SPKF_norm_const(Sensor, SPKF_norm, EKF, Est_sensParams, tStep);
%     SE_timer(11) = SE_timer(11) + toc;
%     
%     EKF = EKF_position(Sensor, EKF, SPKF, Hist.SPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
%     AEKF = AEKF_position(Sensor, AEKF, ASPKF, Hist.ASPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
    

    
    

    %Discrete Time recording @ 200 Hz
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att, SRSPKF, SRSPKF_full, ASPKF_opt, AHINF,SPKF_norm, useExpData);
                    
                    
                    
                    
end

toc

%% Generate plottable arrays
Plot = hist2plot(Hist, useExpData);

Plot.times = TIME;
Plot.quaternions = [VISN_QuatW'; VISN_QuatX'; VISN_QuatY'; VISN_QuatZ'];

Plot.quaternions = [ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz'];

Plot.posns = [VISN_X'; VISN_Y'; VISN_Z'];

%dont know if i need these or not
% TIME(end+1) = TIME(end)*2 -TIME(end-1)
% VISN_QuatW(end+1) = VISN_QuatW(end);
% VISN_QuatX(end+1) = VISN_QuatX(end);
% VISN_QuatY(end+1) = VISN_QuatY(end);
% VISN_QuatZ(end+1) = VISN_QuatZ(end);
ATT_qw(end+1) = ATT_qw(end);
ATT_qx(end+1) = ATT_qx(end);
ATT_qy(end+1) = ATT_qy(end);
ATT_qz(end+1) = ATT_qz(end);


font_size = 15;
line_size = 15;
line_width = 2;


