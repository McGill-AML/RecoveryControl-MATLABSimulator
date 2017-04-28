
titlemod = fileNo;

% sensors
figure;plot(TIME, [IMU_MagX, IMU_MagY, IMU_MagZ]); grid on;
title(['Magnetometer ']); legend('X', 'Y', 'Z');
xlabel('Time'); ylabel('Mag Field [Gauss]');


figure;plot(TIME, [IMU_GyroX, IMU_GyroY, IMU_GyroZ]); grid on;
title(['Gyroscope ']); legend('X', 'Y', 'Z');

figure; plot(TIME, [IMU_AccX, IMU_AccY, IMU_AccZ]); grid on;
title(['Accelerometer ']); legend('X','Y','Z');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);


figure;plot(TIME, BATT_Curr); grid on;
title(['Battery Current ']); xlabel('Time'); ylabel('Current [Amps]');


%% plot vicon
figure;plot(TIME, [VISN_QuatW, VISN_QuatX, VISN_QuatY,VISN_QuatZ]); grid on;
title(['Vision quat ']); legend('W', 'X', 'Y', 'Z');

vicDelay = 17; %vicon delay in number of timesteps - need to shift to use as truth. 

Vicon_Vframe = cleanVicon([VISN_QuatW'; VISN_QuatX'; VISN_QuatY'; VISN_QuatZ']);
% Vicon_Vframe = shiftVicon(Vicon_Vframe, vicDelay);


figure;plot(TIME, Vicon_Vframe); grid on;
title(['Vision quat cleaned']); legend('W', 'X', 'Y', 'Z');

%%
figure;plot(TIME, [ATT_qw'; ATT_qx'; ATT_qy'; ATT_qz']); grid on;
title(['PX4 quat ']); legend('W', 'X', 'Y', 'Z');
%highpass filter data to see bias info


gyro_filt = filter(0.01, [1, -.99], [IMU_GyroX, IMU_GyroY, IMU_GyroZ], [IMU_GyroX(1), IMU_GyroY(1), IMU_GyroZ(1)],1);

figure;plot(TIME, gyro_filt); grid on
title('Filtered gyro data');
