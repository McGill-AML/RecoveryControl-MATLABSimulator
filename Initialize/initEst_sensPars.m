function [Est_sensParams] = initEst_sensPars(sensParams, useExpData)

Est_sensParams = sensParams;

if useExpData == 1 % changed this shit too.
 Est_sensParams.var_acc = [1; 1; 1]*300; % 300 %abs(sensParams.var_acc)*1;% + randn(3,1)*0.1);

 Est_sensParams.var_gyr =  abs(sensParams.var_gyr)*1;% + randn(3,1)*0.0003);

 Est_sensParams.var_mag = [7*1e-3;  7*1e-3;  7*1e-3]*3000;% abs(sensParams.var_mag)*1;% + randn(3,1)*0.00003);

 Est_sensParams.var_gps =  sensParams.var_gps;% + [0.00000001; -0.00000001 ; 0.00003; 0.00001; 0.00001]; %x, y, height, x-dot y-dot

 Est_sensParams.var_baro =  sensParams.var_baro;

    
Est_sensParams.var_bias_acc =   sensParams.var_bias_acc;

Est_sensParams.var_bias_gyr =  sensParams.var_bias_gyr*1;

Est_sensParams.var_bias_mag =  sensParams.var_bias_mag;
%% EKF
Est_sensParams.EKF = sensParams;

 Est_sensParams.EKF.var_acc = [1; 1; 1]*300; %abs(sensParams.var_acc)*1;% + randn(3,1)*0.1);

 Est_sensParams.EKF.var_gyr =  abs(sensParams.var_gyr)*1;% + randn(3,1)*0.0003);

 Est_sensParams.EKF.var_mag = [7*1e-3;  7*1e-3;  7*1e-3]*3000;% abs(sensParams.var_mag)*1;% + randn(3,1)*0.00003);

 Est_sensParams.EKF.var_gps =  sensParams.var_gps;% + [0.00000001; -0.00000001 ; 0.00003; 0.00001; 0.00001]; %x, y, height, x-dot y-dot

 Est_sensParams.EKF.var_baro =  sensParams.var_baro;

Est_sensParams.EKF.var_bias_acc =   sensParams.var_bias_acc;

Est_sensParams.EKF.var_bias_gyr =  sensParams.var_bias_gyr*.1;

Est_sensParams.EKF.var_bias_mag =  sensParams.var_bias_mag;
else
     Est_sensParams.var_acc = [1; 1; 1]*10; %abs(sensParams.var_acc)*1;% + randn(3,1)*0.1); This was *100

 Est_sensParams.var_gyr =  abs(sensParams.var_gyr)*1;% + randn(3,1)*0.0003);

 Est_sensParams.var_mag = [7*1e-3;  7*1e-3;  7*1e-3]*1;% abs(sensParams.var_mag)*1;% + randn(3,1)*0.00003); This was *100

 Est_sensParams.var_gps =  sensParams.var_gps;% + [0.00000001; -0.00000001 ; 0.00003; 0.00001; 0.00001]; %x, y, height, x-dot y-dot

 Est_sensParams.var_baro =  sensParams.var_baro;

    
Est_sensParams.var_bias_acc =   sensParams.var_bias_acc;

Est_sensParams.var_bias_gyr =  sensParams.var_bias_gyr*1;

Est_sensParams.var_bias_mag =  sensParams.var_bias_mag;
%% EKF
Est_sensParams.EKF = sensParams;

 Est_sensParams.EKF.var_acc = [1; 1; 1]*100; %abs(sensParams.var_acc)*1;% + randn(3,1)*0.1);  This was *100

 Est_sensParams.EKF.var_gyr =  abs(sensParams.var_gyr)*1;% + randn(3,1)*0.0003);

 Est_sensParams.EKF.var_mag = [7*1e-3;  7*1e-3;  7*1e-3]*1;% abs(sensParams.var_mag)*1;% + randn(3,1)*0.00003);  This was *100

 Est_sensParams.EKF.var_gps =  sensParams.var_gps;% + [0.00000001; -0.00000001 ; 0.00003; 0.00001; 0.00001]; %x, y, height, x-dot y-dot

 Est_sensParams.EKF.var_baro =  sensParams.var_baro;

Est_sensParams.EKF.var_bias_acc =   sensParams.var_bias_acc;

Est_sensParams.EKF.var_bias_gyr =  sensParams.var_bias_gyr*1;

Est_sensParams.EKF.var_bias_mag =  sensParams.var_bias_mag;

end

% Est_sensParams.var_acc = [20; 20; 20];
%         
% Est_sensParams.var_gyr = [1.8*1e-3; 1.8*1e-3; 1.8*1e-3];
% 
% Est_sensParams.var_mag = [4*1e-1;  4*1e-1;  4*1e-1]; % was 6 6 12
% 
% Est_sensParams.var_gps = [0.06*1e-7; 0.06*1e-7; 0.6*1e-2; 0.1*1e-2; 0.1*1e-2]; %x, y, height, x-dot y-dot
% 
% Est_sensParams.var_baro = 0.9670*1e-0;
% 
% %% sensor bias variance - not used as static biases are used instead consider
% Est_sensParams.var_bias_acc = 0.001*ones(3,1);
% 
% Est_sensParams.var_bias_gyr = 0.0001*ones(3,1);
% 
% Est_sensParams.var_bias_mag = 0.00001*ones(3,1);