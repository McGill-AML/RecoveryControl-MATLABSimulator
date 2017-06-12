%pass data through highpass filter to compare results to mean subtraction
%method

% hpFilt = designfilt('lowpassiir', ...       % Response type
%        'FilterOrder', 80, ...
%        'HalfPowerFrequency', 0.6, ...
%        'DesignMethod', 	'butter');
%    
% fvtool(hpFilt)
   
hpFilt = designfilt('highpassfir','StopbandFrequency',0.1, ...
         'PassbandFrequency',0.15,'PassbandRipple',0.1, ...
         'StopbandAttenuation',100,'DesignMethod','kaiserwin');   

fvtool(hpFilt)

index_end = length(IMU_GyroX);
out_gyr_x = filter(hpFilt,IMU_GyroX(1:index_end));
out_gyr_y = filter(hpFilt,IMU_GyroY(1:index_end));
out_gyr_z = filter(hpFilt,IMU_GyroZ(1:index_end));

out_acc_x = filter(hpFilt,IMU_AccX(1:index_end));
out_acc_y = filter(hpFilt,IMU_AccY(1:index_end));
out_acc_z = filter(hpFilt,IMU_AccZ(1:index_end));

out_mag_x = filter(hpFilt,IMU_MagX(1:index_end));
out_mag_y = filter(hpFilt,IMU_MagY(1:index_end));
out_mag_z = filter(hpFilt,IMU_MagZ(1:index_end));



figure;plot(TIME, [IMU_AccX, IMU_AccY, IMU_AccZ]); grid on;
title('Accelerometer no foil 1'); legend('X', 'Y', 'Z');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);

figure;plot(TIME, [IMU_GyroX, IMU_GyroY, IMU_GyroZ]); grid on;
title('Gyroscope readings'); legend('X', 'Y', 'Z');

figure;plot(TIME, [IMU_MagX, IMU_MagY, IMU_MagZ]); grid on;
title('Magnetometer foil everything 2'); legend('X', 'Y', 'Z');



figure;plot(TIME(1:index_end),[out_gyr_x, out_gyr_y, out_gyr_z],'Linewidth',line_width); grid on;
xlabel('Time [s]','fontsize',font_size); ylabel('Angular Velocity [rad/s]','fontsize',font_size);
title('HP filtered Gyroscope'); 

figure;plot(TIME(1:index_end),[out_acc_x, out_acc_y, out_acc_z],'Linewidth',line_width); grid on;
xlabel('Time [s]','fontsize',font_size); ylabel('Acceleration [m/s^2]','fontsize',font_size);
title('HP filtered accelerometer'); 

figure;plot(TIME(1:index_end),[out_mag_x, out_mag_y, out_mag_z],'Linewidth',line_width); grid on;
xlabel('Time [s]','fontsize',font_size); ylabel('Magnetic Field Strength [Gauss]','fontsize',font_size);
title('HP filtered magnetometer'); 

title('');

index_end = length(Plot.mag);
out_mag_filt_x = filter(hpFilt,Plot.mag(1,1:index_end));
out_mag_filt_y = filter(hpFilt,Plot.mag(2,1:index_end));
out_mag_filt_z = filter(hpFilt,Plot.mag(3,1:index_end));

figure;plot(Plot.times,Plot.mag); grid on;
title('Corrected magnetometer');

figure;plot(Plot.times(1:index_end),[out_mag_filt_x; out_mag_filt_y; out_mag_filt_z]); grid on;
title('HP filtered magnetometer'); 

% buffer so that you ignore start up data
start_buff = 200;

% skewness([out_gyr_x(start_buff:end), out_gyr_y(start_buff:end),out_gyr_z(start_buff:end)])
% kurtosis([out_gyr_x(start_buff:end), out_gyr_y(start_buff:end),out_gyr_z(start_buff:end)])
% skewness([IMU_GyroX(1:index_end), IMU_GyroY(1:index_end), IMU_GyroZ(1:index_end)])
% kurtosis([IMU_GyroX(1:index_end), IMU_GyroY(1:index_end), IMU_GyroZ(1:index_end)])

cov([out_gyr_x(start_buff:end), out_gyr_y(start_buff:end),out_gyr_z(start_buff:end)])

% skewness([out_acc_x(start_buff:end), out_acc_y(start_buff:end),out_acc_z(start_buff:end)])
% kurtosis([out_acc_x(start_buff:end), out_acc_y(start_buff:end),out_acc_z(start_buff:end)])
% skewness([IMU_AccX(1:index_end), IMU_AccY(1:index_end), IMU_AccZ(1:index_end)])
% kurtosis([IMU_AccX(1:index_end), IMU_AccY(1:index_end), IMU_AccZ(1:index_end)])

cov([out_acc_x(start_buff:end), out_acc_y(start_buff:end),out_acc_z(start_buff:end)])

% skewness([out_mag_x(start_buff:end), out_mag_y(start_buff:end),out_mag_z(start_buff:end)])
% kurtosis([out_mag_x(start_buff:end), out_mag_y(start_buff:end),out_mag_z(start_buff:end)])
% skewness([IMU__MagX(1:index_end), IMU__MagY(1:index_end), IMU__MagZ(1:index_end)])
% kurtosis([IMU__MagX(1:index_end), IMU__MagY(1:index_end), IMU__MagZ(1:index_end)])

cov([out_mag_x(start_buff:end), out_mag_y(start_buff:end),out_mag_z(start_buff:end)])


%% looking at freq data 
% 
% N = 2^nextpow2(length(IMU_AccX));
% Fs = 200;
% 
% AccX_freq = fft(IMU_MagZ);
% L = length(IMU_AccX);
% P2 = abs(AccX_freq/L);
% P1 = P2(1:floor(L/2)+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:floor(L/2))/L;
% figure;plot(f,P1);grid on;

