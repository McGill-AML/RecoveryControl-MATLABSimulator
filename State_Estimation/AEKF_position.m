function [AEKF] = AEKF_position(Sensor, AEKF, ASPKF, q_k_1, ...
                                     sensParams,  tStep, current_time)
% AEKF - kinematic predict - position, velocity and accel bias    
% Adaptive process and measurement noise covariance matrices
% simulates only recieving GPS at given GPS rate 

                                       
global g

%unpackage
%states
%attitude states are for current time step - updated in SPKF ahead of time

pos_k_1 = AEKF.X_hat.pos_hat;

q_k = ASPKF.X_hat.q_hat;

omega_k = ASPKF.X_hat.omega_hat;

rotMat_k = quat2rotmat(q_k);

rotMat_k_1 = quat2rotmat(q_k_1); %just used to rotate velocity to inertial frame

vel_k_1 = rotMat_k_1'*AEKF.X_hat.vel_hat;

%bias terms
bias_acc_k_1 = AEKF.X_hat.bias_acc;
bias_gyr_k = ASPKF.X_hat.bias_gyr;

%measurements
u_b_acc = Sensor.acc;
% u_b_gyr = Sensor.gyro; %not used in EKF
% u_b_mag = Sensor.mag; %not used in EKF

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

lat_0 = sensParams.gps_init(1);
long_0 = sensParams.gps_init(2);
height_0 = sensParams.gps_init(3);
Me  = sensParams.gps_init(4);
Ne  = sensParams.gps_init(5);

baro_0 = sensParams.baro_init;

P_k_1_pos = AEKF.P_hat;

         
%% predict
%init covariance for sig pt calcs
F_k_1 = [eye(3), tStep*eye(3), zeros(3);
         zeros(3), eye(3), -tStep*rotMat_k';
         zeros(3), zeros(3), eye(3)];

L_k_1 = [0.5*tStep^2*rotMat_k', -0.5*tStep^2*rotMat_k';
         tStep*rotMat_k', -tStep*rotMat_k';
         zeros(3), tStep*eye(3)];
     
Q_k_1 = diag([sensParams.var_acc;
       sensParams.var_bias_acc*ones(3,1)]);

        
% R_k = diag([sensParams.var_gps;
%             sensParams.var_baro]);
        
Delta_G = diag([AEKF.G_k*ones(1,3), AEKF.G_k*ones(1,3)]);


% lil_del_G = diag([ones(1,3), ones(1,3)]);

% R_k_G = 1/AEKF.G_k*lil_del_G*R_k*lil_del_G;

Q_k_G = AEKF.G_k*Delta_G*Q_k_1*Delta_G;
        

vel_k_m = vel_k_1 + tStep*rotMat_k'*(u_b_acc - bias_acc_k_1) + tStep*[0;0;g];

pos_k_m = pos_k_1 + tStep*vel_k_1;

b_b_acc_k_m = bias_acc_k_1;

P_k_m = F_k_1*P_k_1_pos*F_k_1' + L_k_1*Q_k_G*L_k_1';


%%
%measurement sigma and correct

% cycle through prev innov
AEKF.innov_k(2:end) = AEKF.innov_k(1:end-1);

if mod(current_time,sensParams.GPS_rate) == 0
    H_k = [blkdiag(1/(Me+height_0-pos_k_m(3))*180/pi, 1/((Ne+height_0-pos_k_m(3))*cos(lat_0*pi/180.0))*180/pi,  -1, eye(2), 101325* -2.25577*10^ -5*5.25588*(1 - 2.25577*10^(-5)*(-pos_k_m(3)+baro_0))^4.25588), zeros(6,3)];
    
    R_k = diag([sensParams.var_gps/1000;
        sensParams.var_baro]);
    
    lil_del_G = diag([ones(1,5), 1]);
    
    R_k_G = 1/AEKF.G_k*lil_del_G*R_k*lil_del_G;
    
    % H_k = [blkdiag(1/(Me+height_0-pos_k_m(3))*180/pi, 1/((Ne+height_0-pos_k_m(3))*cos(lat_0*pi/180.0))*180/pi,  -1, eye(2), 101325* -2.25577*10^ -5*5.25588*(1 - 2.25577*10^(-5)*(-pos_k_m(3)+baro_0))^4.25588)];
    %
    % R_k = diag([sensParams.var_gps;
    %             sensParams.var_baro]);
    
    y_k = [u_b_gps; u_b_baro];
    
    y_k_hat(1:5,1) = H_k(1:5,:)*[pos_k_m; vel_k_m; b_b_acc_k_m]+ [lat_0; long_0; height_0; 0; 0];
    
    % y_k_hat(1:5,1) = H_k(1:5,:)*[pos_k_m; vel_k_m]+ [lat_0; long_0; height_0; 0; 0];
    
    y_k_hat(6,1) = 101325*(1-2.25577*10^-5*(-pos_k_m(3)+baro_0))^5.25588;
    innov = (y_k - y_k_hat);
    AEKF.innov_k(1) = norm(tStep*AEKF.gamma*innov,2); %update prev innovation norms
    
else
    H_k = [zeros(1,5), 101325* -2.25577*10^ -5*5.25588*(1 - 2.25577*10^(-5)*(-pos_k_m(3)+baro_0))^4.25588, zeros(1,3)];
    
    R_k = diag([sensParams.var_baro]);
    
    lil_del_G = diag(1);
    
    R_k_G = 1/AEKF.G_k*lil_del_G*R_k*lil_del_G;
    
    y_k = u_b_baro;
    
    y_k_hat = 101325*(1-2.25577*10^-5*(-pos_k_m(3)+baro_0))^5.25588;
    innov = (y_k - y_k_hat);
    AEKF.innov_k(1) = norm(tStep*AEKF.gamma(6,6)*innov,2); %update prev innovation norms
end


K_k = P_k_m*H_k'/(H_k*P_k_m*H_k' + R_k_G);

P_hat_pos = (eye(9)-K_k*H_k)*P_k_m*(eye(9)-K_k*H_k)' + K_k*R_k_G*K_k';
% P_hat_pos = (eye(6)-K_k*H_k)*P_k_m*(eye(6)-K_k*H_k)' + K_k*R_k*K_k';

P_hat_pos = 0.5*(P_hat_pos + P_hat_pos');

x_k_hat_pos = [pos_k_m; vel_k_m; b_b_acc_k_m] + K_k*innov;
% x_k_hat_pos = [pos_k_m; vel_k_m] + K_k*(y_k - y_k_hat);

% update adaptive gain
if sum(AEKF.innov_k) > AEKF.innov_tresh
    if AEKF.G_k < AEKF.G_max
        AEKF.G_k = AEKF.G_k + AEKF.G_rate;
    else
        AEKF.G_k = AEKF.G_k;
    end
else
    if AEKF.G_k > 1 && AEKF.G_k - AEKF.G_rate >= 1
        AEKF.G_k = AEKF.G_k - AEKF.G_rate;
    elseif AEKF.G_k > 1
        AEKF.G_k = 1;
    else
        AEKF.G_k = AEKF.G_k;
    end
end


%% repackage

AEKF.X_hat.pos_hat = x_k_hat_pos(1:3);

AEKF.X_hat.vel_hat = x_k_hat_pos(4:6);

AEKF.X_hat.bias_acc = x_k_hat_pos(7:9);

AEKF.P_hat = P_hat_pos;
     
     