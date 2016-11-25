function [sensParams] = initsensor_params()
    global mag
    
    % actual montreal mag field
    mag = [0.3;-.07;0.94];
%     mag = [0.9; .2; 0];
    
    % sendor variances
    sensParams.var_acc = [1; 1;  1];

    sensParams.var_gyr = [1.8*1e-3; 1.8*1e-3; 1.8*1e-3];

    sensParams.var_mag = [7*1e-4;  7*1e-4;  7*1e-4];

    sensParams.var_gps = [0.6*1e-7; 0.6*1e-7; 0.6*1e-3; 0.1*1e-2; 0.1*1e-2]; %x, y, height, x-dot y-dot

    sensParams.var_baro = 0.9670*1e-0;

    %% sensor bias variance - not used as static biases are used instead consider
    sensParams.var_bias_acc = 0.001*ones(3,1);

    sensParams.var_bias_gyr  = 0.001*ones(3,1);

    sensParams.var_bias_mag  = 0.00001*ones(3,1);

    %% walking gps model - not used during crash
    sensParams.var_bias_gps  = .00001;
    sensParams.gps_bias_tau = 2*10^2;

    sensParams.var_bias_gps_guess  = .0000001;

    sensParams.var_bias_baro = 0.00001;

    sensParams.GPS_rate = 1/10;

    %% set constant sensor biases

    sensParams.bias.acc = randn(3,1)*0.1;

    sensParams.bias.gyr = [ -0.0173; -0.0056; 0.0218];%randn(3,1)*0.01;

    sensParams.bias.mag = randn(3,1)*0.001;

    sensParams.bias.gps = randn(3,1)*0.000003;

    sensParams.bias.baro = randn(1,1)*10;

    % initialize GPS coords to near McConnell Eng buidling, height as well
    sensParams.gps_init = [45.5046581, -73.5758246, 23.142];
    
    %% initialize sensor crash variances - only for accel and gyro, mag is
    % fine
    
    sensParams.crash.var_acc = 14;
    
    sensParams.crash.var_gyr = 0.1;
    
    sensParams.crash.time_const = 0.4;
    
    sensParams.crash.occur = 0;
    
    sensParams.crash.time_since = 0;
    
    sensParams.crash.new = 1;


end






