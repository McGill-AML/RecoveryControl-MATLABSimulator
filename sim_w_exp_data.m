% clearvars -except SPKF ASPKF rmse loop_no SE_timer Setpoint Trajectory gamma
 clear;
 fileNo = 17
 tic;
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

%initialize the mag field data
useDataPts4Mag = 20;
% mag(1) = mean(IMU_MagX(1:useDataPts4Mag));
% mag(2) = mean(IMU_MagY(1:useDataPts4Mag));
% mag(3) = mean(IMU_MagZ(1:useDataPts4Mag));
% 
% %switch to NED frame
% mag(1) = norm([mag(1), mag(2)]);
% mag(2) = 0;

%set mag decl
mag_decl = 0; %14.321;

%to initialize inertial mag field, find rotation matrix into inertial
[Est_ICs.q, rotMat] = initOrientExpData([IMU_AccX(1:useDataPts4Mag), IMU_AccY(1:useDataPts4Mag), IMU_AccZ(1:useDataPts4Mag)]', [IMU_MagX(1:useDataPts4Mag), IMU_MagY(1:useDataPts4Mag), IMU_MagZ(1:useDataPts4Mag)]', mag_decl);

%then rotate mag into inertial
mag_b = zeros(3,1);
mag_b(1) = mean(IMU_MagX(1:useDataPts4Mag));
mag_b(2) = mean(IMU_MagY(1:useDataPts4Mag));
mag_b(3) = mean(IMU_MagZ(1:useDataPts4Mag));
mag = rotMat*mag_b;

%nomralize
% mag = mag/norm(mag);



corMag = 1;


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

Sensor = package_sens_data(IMU_AccX(1), IMU_AccY(1), IMU_AccZ(1), IMU_GyroX(1), IMU_GyroY(1), IMU_GyroZ(1), IMU_MagX(1), IMU_MagY(1), IMU_MagZ(1), GPS, BAR); %init sensor

sensParams = initsensor_params(useExpData); % initialize sensor parameters for use in measurement model

% sensParams = initgps_baro(Sensor, sensParams); %init initial GPS and baro in order to initialize cartesian coord at starting spot

Est_sensParams = initEst_sensPars(sensParams,useExpData); %initialize sensor parameters for use in estimators (to add error, etc.)

%% Initialize state estimators
Est_ICs = initSE_ICs(IC, sensParams, loop_no, useExpData); % set initial condition estimate in order to add errors

if useExpData == 1
    init_with_pts = 20;
    [Est_ICs.q, rotMat]  = initOrientExpData([IMU_AccX(1:init_with_pts)'; IMU_AccY(1:init_with_pts)'; IMU_AccZ(1:init_with_pts)'], [IMU_MagX(1:init_with_pts)'; IMU_MagY(1:init_with_pts)';IMU_MagZ(1:init_with_pts)'], mag_decl);
end

%just initialize to whatever their estimator initialized to - not sure why
%my initialization is wrong
% clean up att q estimate so that it doesn't have NaNs
for ii = 1:length(ATT_qw)
    if isnan(ATT_qw(ii))
        jj = ii;
        while isnan(ATT_qw(jj))
            jj = jj + 1;
        end
        ATT_qw(ii:jj-1) = ATT_qw(jj);
        ATT_qx(ii:jj-1) = ATT_qx(jj);
        ATT_qy(ii:jj-1) = ATT_qy(jj);
        ATT_qz(ii:jj-1) = ATT_qz(jj);
    end
end

for ii = 1:length(BATT_Curr)
    if isnan(BATT_Curr(ii))
        jj = ii;
        while isnan(BATT_Curr(jj))
            jj = jj + 1;
        end
        BATT_Curr(ii:jj-1) = BATT_Curr(jj);
    end
end
% 
% Est_ICs.q = [ATT_qw(1); ATT_qx(1); ATT_qy(1); ATT_qz(1)];

EKF = initEKF(Est_ICs);
AEKF = initAEKF(Est_ICs);
SPKF = initSPKF(Est_ICs,useExpData);
ASPKF = initASPKF(Est_ICs,useExpData);
COMP = initCOMP(Est_ICs,useExpData);
HINF = initHINF(Est_ICs,useExpData);
SPKF_full = initSPKF_full(Est_ICs);
EKF_att = initEKF_att(Est_ICs,useExpData);
SRSPKF = initSRSPKF(Est_ICs,useExpData);
SRSPKF_full = initSRSPKF_full(Est_ICs);
ASPKF_opt = initASPKF_opt(Est_ICs,useExpData);
AHINF = initAHINF(Est_ICs,useExpData);

% SPKF_norm = initSPKF_norm(Est_ICs, useExpData);
SPKF_norm = initCOMP_PX4(Est_ICs,useExpData);
% SPKF_norm.gyr_bias_k_i =.05;


% Est_sensParams.EKF.var_bias_gyr = Est_sensParams.EKF.var_bias_gyr*0;

SE_timer = zeros(11,1);

%init lp filter for COMP filter
[COMP] = initlpfilt(Sensor, COMP);

time_to_break = 0; %var so sim doesn't stop once it's stabilized

%% section for auto tuning params by varying with the outer loops in run_loop_exp
if exist('running_a_loop','var')
    
%     comp_vary = 0.05:0.05:0.25;
%     
%     COMP.mag_k_i = comp_vary(change_two);
% %     COMP.acc_k_i = comp_vary(change_two); %yo theses are the same now
%     SPKF_norm.mag_k_i = comp_vary(change_two);
%     SPKF_norm.acc_k_i = comp_vary(change_two);
%     SPKF_norm.gyr_bias_k_i = comp_vary(change_one);
%     COMP.gyr_bias_k_i = comp_vary(change_one);
%     COMP.accel_bound = (change_one*2)-1;
%     
%     ekf_mag_acc_vary = [0.1, 1, 10, 100, 300];
% %     hinf_val = [0.1, 0.1, 0.1, 0.2, 0.2];
% % % 
% % HINF.G_k = hinf_val(change_one)*[eye(4), zeros(4,3)];
% % % 
% % AHINF.delta_max = hinf_val(change_two);
% % AHINF.delta_rate = hinf_val(change_two)/10;
% % 
% % %     
%     Est_sensParams.EKF.var_bias_gyr = Est_sensParams.EKF.var_bias_gyr*ekf_mag_acc_vary(change_two);
%     Est_sensParams.var_mag = Est_sensParams.var_mag*ekf_mag_acc_vary(change_two);
% gmax_val =  [.1, .1, .2, 0.2, 0.3];
% grate_val = [0.005, 0.01, 0.01, 0.05, .01];
% 
% AHINF.delta_max = gmax_val(change_one);
% AHINF.delta_rate = grate_val(change_one);

% acc_b_val = [4, 5, 6, 7, 8];
% 
% EKF_att.accel_bound =  acc_b_val(change_one);

%     if change_one == 2
% %           SPKF.accel_bound = 3;
% %           EKF_att.accel_bound = 3;
% %             Est_sensParams.var_mag(3) = Est_sensParams.var_mag(3)/100;
% %         
% %         HINF.accel_bound = 2;
%           
% %         Est_sensParams.var_mag = Est_sensParams.var_mag *10;
%     elseif change_one == 3 
% %           SPKF.accel_bound = 1;
% %           EKF_att.accel_bound = 1;
% %         
% %         HINF.accel_bound = 4;
%              
% %           ASPKF.G_max = 3;
% %           ASPKF.G_rate = .1;
% %         Est_sensParams.var_acc = Est_sensParams.var_acc*10;
%     elseif change_one == 4
% %             Est_sensParams.var_acc = Est_sensParams.var_acc/10;
% %           SPKF.accel_bound = 0.5;
% %           EKF_att.accel_bound = 0.5;
% %           SPKF.kappa = -3;
%           
% %         Est_sensParams.var_bias_gyr = Est_sensParams.var_bias_gyr*.01;
%     elseif change_one == 5
% %           SPKF.accel_bound = 4;
% %           EKF_att.accel_bound = 4;
% %           SPKF.kappa = -6;
% %           
% %           HINF.accel_bound = 4;
%           
% %           ASPKF.G_max = 3;
% %           ASPKF.G_rate = .1;
% %         Est_sensParams.var_gyr = Est_sensParams.var_gyr*100;
% %         hinf_val = [0.05, 0.1, 0.7, 10, 50];
% %         AHINF.delta_max = hinf_val(change_two);
% %         AHINF.delta_rate = hinf_val(change_two)/10;
% %         Est_sensParams.var_bias_gyr = Est_sensParams.var_bias_gyr*.01;
%     end

end
%% Initialize History Arrays

% Initialize history 
Hist = inithist(SimParams, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att,SRSPKF, SRSPKF_full, ASPKF_opt, AHINF, SPKF_norm, useExpData, tStep, length(TIME));
            
magfilt = zeros(3,length(TIME));
gyrfilt = zeros(3,length(TIME));
accfilt = zeros(3,length(TIME));



%% Simulation Loop
for iSim = 2:length(TIME)
%     display(iSim)    
    tStep = TIME(iSim)-TIME(iSim-1);

    Sensor = package_sens_data(IMU_AccX(iSim), IMU_AccY(iSim), IMU_AccZ(iSim), IMU_GyroX(iSim), IMU_GyroY(iSim), IMU_GyroZ(iSim), IMU_MagX(iSim), IMU_MagY(iSim), IMU_MagZ(iSim), GPS, BAR);
   
    Sensor = correct_mag(Sensor, BATT_Curr(iSim), corMag, mot_comp);
    
   [Sensor_filt,COMP] = applylpfilt(Sensor, COMP);
    magfilt(:,iSim) = Sensor_filt.mag;
    gyrfilt(:,iSim) = Sensor_filt.gyro;
    accfilt(:,iSim) = Sensor_filt.acc;
    
    Sensor_SPKF = Sensor;
    Sensor_SPKF.gyro = Sensor_filt.gyro;
    
%     Sensor_filt = Sensor;
%     [Sensor.acc, move_avg_acc] = moving_avg_filt(Sensor.acc, move_avg_acc);
    %% State Estimation
%     tic;
%     SPKF = SPKF_attitude(Sensor_SPKF, SPKF, EKF, Est_sensParams, tStep, useExpData);
%     SE_timer(1) = SE_timer(1) + toc;
%     
%     tic;
%     ASPKF = ASPKF_attitude(Sensor_SPKF, ASPKF, EKF, Est_sensParams, tStep, useExpData,0);
%     SE_timer(2) = SE_timer(2) + toc;
%     
%     tic;
    EKF_att = EKF_attitude(Sensor_SPKF, EKF_att, EKF, Est_sensParams.EKF, tStep,useExpData);
%     SE_timer(3) = SE_timer(3) + toc;
%     
%     tic;
%     SPKF_full = SPKF_full_state(Sensor, SPKF_full, Est_sensParams, tStep, iSim);
%     SE_timer(4) = SE_timer(4) + toc;

%     
%     tic;
%     COMP = CompFilt_attitude(Sensor_filt, COMP, EKF, Est_sensParams, tStep,useExpData);
%     SE_timer(5) = SE_timer(5) + toc;
%     
%     tic;
    HINF = HINF_attitude(Sensor_SPKF, HINF, EKF, Est_sensParams.EKF, tStep, useExpData);
%     SE_timer(6) = SE_timer(6) + toc;
    
%     tic;
%     SRSPKF = SRSPKF_attitude(Sensor, SRSPKF, EKF, Est_sensParams, tStep, useExpData);
%     SE_timer(7) = SE_timer(7) + toc;
    
%     tic;
%     SRSPKF_full = SRSPKF_full_state(Sensor, SRSPKF_full, Est_sensParams, tStep, iSim);
%     SE_timer(8) = SE_timer(8) + toc;
    
%     tic;
%     ASPKF_opt = ASPKF_opt_attitude(Sensor_SPKF, ASPKF_opt, AEKF, Est_sensParams, tStep, useExpData);
%     SE_timer(9) = SE_timer(9) + toc;
%     
%     tic;
%     ASPKF_opt = SPKF_attitude_SVD(Sensor, ASPKF_opt, AEKF, sensParams, tStep);
%     SE_timer(9) = SE_timer(9) + toc;
    
%     tic;
    AHINF = AHINF_attitude(Sensor_SPKF, AHINF, EKF, Est_sensParams.EKF, tStep, useExpData);
%     SE_timer(10) = SE_timer(10) + toc;

%     tic;
%     SPKF_norm = CompFilt_PX4(Sensor_filt, SPKF_norm, EKF, Est_sensParams, tStep, useExpData);
%     SE_timer(4) = SE_timer(4) + toc;
    
%     tic;
%     SPKF_norm = SPKF_norm_const(Sensor, SPKF_norm, EKF, Est_sensParams, tStep);
%     SE_timer(11) = SE_timer(11) + toc;
%     
%     EKF = EKF_position(Sensor, EKF, SPKF, Hist.SPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
%     AEKF = AEKF_position(Sensor, AEKF, ASPKF, Hist.ASPKF(end).X_hat.q_hat, Est_sensParams, tStep, iSim);
    

    
    

    %Discrete Time recording @ 200 Hz
    Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att, SRSPKF, SRSPKF_full, ASPKF_opt, AHINF,SPKF_norm, useExpData, iSim, tStep);
                    
                    
                    
                    
end

% toc

%% Generate plottable arrays
Plot = hist2plot(Hist, useExpData);

Plot.times = TIME;

Plot.quaternions = [ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz'];

toc;

Plot.posns = [VISN_X'; VISN_Y'; VISN_Z'];


%% rotate vicon into inertial frame
vicDelay = 17; %vicon delay in number of timesteps - need to shift to use as truth. 

Vicon_Vframe = cleanVicon([VISN_QuatW'; VISN_QuatX'; VISN_QuatY'; VISN_QuatZ']);
Vicon_Vframe = shiftVicon(Vicon_Vframe, vicDelay);
[vic2NED, q_v2NED] = findVic2NED(Vicon_Vframe,[IMU_MagX, IMU_MagY, IMU_MagZ]', [IMU_AccX, IMU_AccY, IMU_AccZ]', mag_decl, 80);

Vicon_NED = zeros(size(Vicon_Vframe));
for ii = 1:length(Vicon_Vframe)
%     temp = quatmultiply( Vicon_Vframe(:,ii),[0;1;0;0]);
    RViVb = quat2rotmat(Vicon_Vframe(:,ii));
    RVbQb = quat2rotmat([0;1;0;0]);
    temp = rotmat2quat((RVbQb'*RViVb')');
    rotMatV = quat2rotmat(temp);
    Vicon_NED(:,ii) = rotmat2quat(vic2NED*rotMatV);
%     Vicon_NED(:,ii) = quatmultiply(temp, quatinv(q_v2NED));
%     temp = quatmultiply([0.9455;  0;  0;  -0.3256],[0; 1; 0; 0]);
%      Vicon_NED(:,ii) = quatmultiply(temp, Vicon_Vframe(:,ii));
%     Vicon_NED(:,ii) = quatmultiply([0.9455;  0;  0;  -0.3256], temp);
end


Vicon_NED = cleanVicon(Vicon_NED);

Plot.quaternions = Vicon_NED;
% Plot.quaternions = [ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz'];

%rotate Px4 to match up with vicon init - why doesn't PX4 init to same? 
quat_PX42NED = quatmultiply([ATT_qw(1); ATT_qx(1); ATT_qy(1); ATT_qz(1)], quatinv(Vicon_NED(:,1)));
%just doing this to init it.
Plot.SRSPKF_quat = [ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz'];

for ii = 1:length([ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz'])

Plot.SRSPKF_quat(:,ii) = quatmultiply([ATT_qw(ii); ATT_qx(ii); ATT_qy(ii); ATT_qz(ii)],quatinv(quat_PX42NED));
end

Plot.eulerAngles = zeros(3, length(Plot.quaternions));
for ii = 1:length(Plot.quaternions)
    [a, b, c] = quat2angle(Plot.quaternions(:,ii), 'zyx');
   Plot.eulerAngles(:,ii) =  [a, b, c]';
end


%% compute total errors

if ~exist('running_a_loop','var')
    
    rmseEUL = [];
    rmse = [];
    rmse = rmse_att(Plot,sensParams, rmse, 0, useExpData, 1, fileNo, crashIndex, viconDropIndex, vicDelay);
    
%     rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);

    
    rmseEUL = rmse_att_euler(Plot, rmseEUL, 0, useExpData, 1, fileNo, crashIndex, viconDropIndex, vicDelay);
end

font_size = 15;
line_size = 15;
line_width = 2;


