%% plot accelerometer and vicon position
close all;

titlemod = '';

% sensors
figure;plot(TIME, [IMU_MagX, IMU_MagY, IMU_MagZ]); grid on;
title(['Magnetometer ',titlemod]); legend('X', 'Y', 'Z');


figure;plot(TIME, [IMU_GyroX, IMU_GyroY, IMU_GyroZ]); grid on;
title(['Gyroscope ',titlemod]); legend('X', 'Y', 'Z');

figure; plot(TIME, [IMU_AccX, IMU_AccY, IMU_AccZ]); grid on;
title(['Accelerometer ',titlemod]); legend('X','Y','Z');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);


figure; plot(TIME, [IMUAccX, IMUAccY, IMUAccZ]); grid on;
title(['Accelerometer ',titlemod]); legend('X','Y','Z');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);


figure;plot(TIME, [IMUGyroX, IMUGyroY, IMUGyroZ]); grid on;
title(['Gyroscope ',titlemod]); legend('X', 'Y', 'Z');

figure;plot(TIME, [IMUMagX, IMUMagY, IMUMagZ]); grid on;
title(['Magnetometer ',titlemod]); legend('X', 'Y', 'Z');



figure;plot(TIME, SENS_DiffPresFilt); grid on;
title('Barometer Pressure'); 

figure;plot(TIME, SENS_BaroAlt); grid on;
title('Barometer Altitudes'); 


%% crash vicon stuff
figure;plot(v_vicon__pose___time, [v_vicon__pose_pose_position_x,v_vicon__pose_pose_position_y,v_vicon__pose_pose_position_z]);
grid on; title('Vicon Position'); 
hold on;
plot(v_mavros_setpoint__position_local___time,[v_mavros_setpoint__position_local_pose_position_x, v_mavros_setpoint__position_local_pose_position_y, v_mavros_setpoint__position_local_pose_position_z]);
legend('Vic X','Vic Y','Vic Z', 'Set X', 'Set Y', 'Set Z', 'Location', 'southwest');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);




%% plot EKF orientation
figure;plot(TIME, [ATTqw, ATTqx, ATTqy, ATTqz]); grid on;
title('Pixhakwk EKF Orientation');



%convert both quaternions to eul angles for slightly easier interpretation
eul_EKF = zeros(3,length(ATTqw));
eul_vic = zeros(3,length(v_vicon__pose_pose_orientation_w));
for ii = 1:length(ATTqw)
    [eul_EKF(1,ii) ,eul_EKF(2,ii), eul_EKF(3,ii)] = quat2angle([ATTqw(ii); ATTqx(ii); ATTqy(ii); ATTqz(ii)]);
end

for ii = 1:length(v_vicon__pose_pose_orientation_w)
    [eul_vic(1,ii), eul_vic(2,ii), eul_vic(3,ii)] = quat2angle([v_vicon__pose_pose_orientation_w(ii); v_vicon__pose_pose_orientation_x(ii); v_vicon__pose_pose_orientation_y(ii); v_vicon__pose_pose_orientation_z(ii)]);
end

figure;plot(TIME, eul_EKF); grid on;
title('Euler anlges EKF');

figure;plot(v_vicon__pose___time, eul_vic);grid on;
title('Euler angles vicon');

%% vic quaternions

figure;plot(EKF_time_anim, [vic_att_w, vic_att_x, vic_att_y, vic_att_z]);grid on; title('vicon');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);

figure;plot(EKF_time_anim, [ATTqw(init_EKF_ind:end), ATTqx(init_EKF_ind:end), ATTqy(init_EKF_ind:end), ATTqz(init_EKF_ind:end)]);grid on;
title('EKF');hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);


%% animate shit
Hist = [EKF_time_anim,vel, ang_vel, vic_x, vic_y, vic_z, vic_att_w, vic_att_x, vic_att_y, vic_att_z];
Hist = Hist(acc_crsh_ind(run_no)-init_EKF_ind-300:vic_cutout,:);
animate(1,Hist,'ZX',['EKF_Vic_videos\g0',num2str(run_no),'_vicon']);

Hist = [EKF_time_anim,vel, ang_vel, vic_x, vic_y, vic_z, ATTqw(init_EKF_ind:end), ATTqx(init_EKF_ind:end), ATTqy(init_EKF_ind:end), ATTqz(init_EKF_ind:end)];
Hist = Hist(acc_crsh_ind(run_no)-init_EKF_ind-300:end,:);
animate(1,Hist,'ZX',['EKF_Vic_videos\g0',num2str(run_no),'_EKF']);


%% plot highpass crash stuff

figure;plot( stats_gyr_y); grid on
hold on;
plot(stats_gyr_y_noF);















