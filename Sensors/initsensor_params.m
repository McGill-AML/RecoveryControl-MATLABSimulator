function [sensParams] = initsensor_params(useExpData, loop_no)
    global mag
    
    % actual montreal mag field
    if useExpData == 0
        mag = [0.3; 0 ;0.9];
        %     mag = [0.9; .2; 0];
        mag = mag/norm(mag);
    end
    
    % sensor variances
    if useExpData < 2
        % set the values for simulation - basically just using these values
        % for the estimator as well
        sensParams.var_acc = [1; 1; 1];
        
        sensParams.var_gyr = [1.8*1e-3; 1.8*1e-3; 1.8*1e-3];
        
        sensParams.var_mag = [7*1e-3;  7*1e-3;  7*1e-3];
        
        sensParams.var_gps = [0.06*1e-7; 0.06*1e-7; 0.6*1e-2; 0.1*1e-2; 0.1*1e-2]; %x, y, height, x-dot y-dot
        
        sensParams.var_baro = 0.9670*1e-0;
        
        %% sensor bias variance - not used as static biases are used instead consider
        sensParams.var_bias_acc = zeros(3,1); %0.001*ones(3,1);
        
        sensParams.var_bias_gyr = 0.0001*ones(3,1);
        
        sensParams.var_bias_mag = zeros(3,1); % 0.00001*ones(3,1);
        
    elseif useExpData == 99
        sensParams.var_acc = [200; 200; 200];
        
        sensParams.var_gyr = [1.8*1e-3; 1.8*1e-3; 1.8*1e-3];
        
        sensParams.var_mag = [484*1e-0;  484*1e-0;  484*1e-0]; % was 6 6 12
        
        sensParams.var_gps = [0.06*1e-7; 0.06*1e-7; 0.6*1e-2; 0.1*1e-2; 0.1*1e-2]; %x, y, height, x-dot y-dot
        
        sensParams.var_baro = 0.9670*1e-0;
        
        %% sensor bias variance - not used as static biases are used instead consider
        sensParams.var_bias_acc = 0.001*ones(3,1);
        
        sensParams.var_bias_gyr = 0.0001*ones(3,1);
        
        sensParams.var_bias_mag = 0.00001*ones(3,1);
        
        
    end
    %% walking gps model - not used during crash
    sensParams.var_bias_gps  = .00001;
    sensParams.gps_bias_tau = 2*10^2;

    sensParams.var_bias_gps_guess  = .0000001;

    sensParams.var_bias_baro = 0.00001;

    sensParams.GPS_rate = 1/10;

    %% set constant sensor biases
    if useExpData == 0
        
        sensParams.bias.acc = randn(3,1)*0.05;
%         sensParams.bias.acc = [-0.0024   -0.0182    0.0679    0.0394    0.0916;
%                                 0.0035   -0.0901    0.0668    0.0050    0.0263;
%                                 0.1250    0.0163   -0.0003    0.0040   -0.0267];
%         sensParams.bias.acc = sensParams.bias.acc(:,mod(loop_no-1,5)+1);
                            
        sensParams.bias.gyr = randn(3,1)*0.01;% [ -0.0173; -0.0056; 0.0218]; %
%         sensParams.bias.gyr = [0.0049   -0.0076   -0.0129    0.0152    0.0073;
%                                 0.0027    0.0029    0.0048    0.0152   -0.0189;
%                                 0.0013   -0.0067   -0.0078    0.0038    0.0059];
%         sensParams.bias.gyr = sensParams.bias.gyr(:,mod(loop_no-1,5)+1);
        
        sensParams.bias.mag = [0;0;0];%randn(3,1)*0.001;
        
        sensParams.bias.gps = [0;0;0]; %randn(3,1)*0.000003;
        
        sensParams.bias.baro = 0; %randn(1,1)*10;
        
        % initialize GPS coords to near McConnell Eng buidling, height as well
        sensParams.gps_init = [45.5046581, -73.5758246, 23.142];
        
    else
        sensParams.bias.acc = randn(3,1)*0.05;
        
        sensParams.bias.gyr = randn(3,1)*0.01;% [ -0.0173; -0.0056; 0.0218]; %
        
        sensParams.bias.mag = [0;0;0];%randn(3,1)*0.001;
        
        sensParams.bias.gps = [0;0;0]; %randn(3,1)*0.000003;
        
        sensParams.bias.baro = 0; %randn(1,1)*10;
        
        % initialize GPS coords to near McConnell Eng buidling, height as well
        sensParams.gps_init = [45.5046581, -73.5758246, 23.142];
    end
    %% initialize sensor crash variances - only for accel and gyro, mag is
    % fine
    
    sensParams.crash.var_acc = 14;
    
    sensParams.crash.var_gyr = 0.1;
    
    sensParams.crash.time_const = 0.4;
    
    sensParams.crash.occur = 0;
    
    sensParams.crash.time_since = 0;
    
    sensParams.crash.new = 1;


end






