function [AHINF] = initAHINF(Est_ICs, useExpData)

%initial states ang_vel, quat, gyro bias
AHINF.X_hat.q_hat = Est_ICs.q;
AHINF.X_hat.omega_hat = Est_ICs.omega;
AHINF.X_hat.bias_gyr = Est_ICs.bias_gyr;

%estimator constants

AHINF.P_hat = Est_ICs.P_init_att;

AHINF.P_hat_eps = AHINF.P_hat;
    


if useExpData== 0  
    AHINF.P_hat(5:7,5:7) = AHINF.P_hat(5:7,5:7)*10;
else
    AHINF.P_hat(1:4,1:4) = AHINF.P_hat(1:4,1:4)*0.001;
end


%adaptive values
AHINF.innov_tresh = 0.3;% 0.6; % innovation sum threshold

AHINF.delta_max = 0.2; % 0.2 max adaptive gain

AHINF.delta_rate = 0.05; % 0.05 how fast the adaptive gain grows when innov sum above threshold

AHINF.delta = 0.0; %initial adaptive gain 0 = regular EKF


AHINF.innov_k = zeros(5,1); % length of this vector decide how far back to look at innovation

AHINF.gamma = diag([ones(1,3)*30,ones(1,3)*1]); % 30 * mag scales magnetometer to be same size as accelerometer readings


AHINF.G_k = [eye(4), zeros(4,3)]; % the H inf gain

if useExpData== 0
    AHINF.accel_bound = 0.5; % +/- how much larger thna gravity before not used in update
else
    AHINF.accel_bound = 5; % +/- how much larger thna gravity before not used in update
end

AHINF.use_acc = 1; % values for if it should use accelerometer reading
