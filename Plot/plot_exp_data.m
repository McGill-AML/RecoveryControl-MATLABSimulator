
titlemod = fileNo;

% sensors
figure;plot(TIME, [IMU_MagX, IMU_MagY, IMU_MagZ]); grid on;
title(['Magnetometer ',titlemod]); legend('X', 'Y', 'Z');


figure;plot(TIME, [IMU_GyroX, IMU_GyroY, IMU_GyroZ]); grid on;
title(['Gyroscope ',titlemod]); legend('X', 'Y', 'Z');

figure; plot(TIME, [IMU_AccX, IMU_AccY, IMU_AccZ]); grid on;
title(['Accelerometer ',titlemod]); legend('X','Y','Z');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);


figure;plot(TIME, [IMU_GyroX, IMU_GyroY, IMU_GyroZ]); grid on;
title(['Gyroscope ',titlemod]); legend('X', 'Y', 'Z');


figure;plot(TIME, [MOCP_QuatW, MOCP_QuatX, MOCP_QuatY,MOCP_QuatZ]); grid on;
title(['MoCap Quat ',titlemod]); legend('W', 'X', 'Y', 'Z');


figure;plot(TIME, [VISN_QuatW, VISN_QuatX, VISN_QuatY,VISN_QuatZ]); grid on;
title(['Vision quat ',titlemod]); legend('W', 'X', 'Y', 'Z');