function [EKF] = EKF_position(Sensor, EKF, SPKF, q_k_1, ...
                                sensParams, tStep, iSim)
% EKF - kinematic predict - position, velocity and accel bias  \
% simulates only recieving GPS at given GPS rate 

% predicts POSITION and VELOCITY in the INERTIAL frame

global g

%unpackage
%states
pos_k_1 = EKF.X_hat.pos_hat;

q_k = SPKF.X_hat.q_hat;

omega_k = SPKF.X_hat.omega_hat;

rotMat_k = quat2rotmat([q_k(1);q_k(2:4)]);

rotMat_k_1 = quat2rotmat(q_k_1); %just used to rotate velocity to inertial frame

vel_k_1 = rotMat_k_1'*EKF.X_hat.vel_hat; %inertial frame velocity

%bias terms
bias_acc_k_1 = EKF.X_hat.bias_acc;
bias_gyr_k = SPKF.X_hat.bias_gyr;

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

P_k_1_pos = EKF.P_hat;
         
         


%% predict
F_k_1 = [eye(3), tStep*eye(3), zeros(3);
         zeros(3), eye(3), -tStep*rotMat_k';
         zeros(3), zeros(3), eye(3)];

L_k_1 = [tStep^2*rotMat_k', tStep^3*rotMat_k';
         tStep*rotMat_k', -tStep^2*rotMat_k';
         zeros(3), tStep*eye(3)];
     
Q_k = diag([sensParams.var_acc;
       sensParams.var_bias_acc]);
     
% F_k_1 = [eye(3), tStep*eye(3);
%          zeros(3), eye(3)];
%          
% 
% L_k_1 = [0.5*tStep^2*rotMat_k';
%          tStep*rotMat_k'];
%      
% Q_k = diag([sensParams.var_acc]);


vel_k_m = vel_k_1 + tStep*rotMat_k'*(u_b_acc - bias_acc_k_1) + tStep*[0;0;-g];

pos_k_m = pos_k_1 + tStep*vel_k_1;

bais_acc_k_m = bias_acc_k_1;

P_k_m = F_k_1*P_k_1_pos*F_k_1' + L_k_1*Q_k*L_k_1';


%% correct

if mod(iSim,sensParams.GPS_rate) == 0
    H_k = [blkdiag(1/(Me+height_0-pos_k_m(3))*180/pi, 1/((Ne+height_0-pos_k_m(3))*cos(lat_0*pi/180.0))*180/pi,  -1, eye(2), 0), zeros(6,3)];
    
    H_k(6,3) = 101325* 2.25577*10^ -5*5.25588*(1 - 2.25577*10^(-5)*(-pos_k_m(3)+height_0))^4.25588;
    
    R_k = diag([sensParams.var_gps;
        sensParams.var_baro]);
    
    
    y_k = [u_b_gps; u_b_baro];
    
    y_k_hat(1:5,1) = H_k(1:5,:)*[pos_k_m; vel_k_m; bais_acc_k_m]+ [lat_0; long_0; height_0; 0; 0];
    

    
    y_k_hat(6,1) = 101325*(1-2.25577*10^-5*(-pos_k_m(3)+height_0))^5.25588;
    
else
    H_k = [zeros(1,2), 101325* 2.25577*10^ -5*5.25588*(1 - 2.25577*10^(-5)*(-pos_k_m(3)+height_0))^4.25588, zeros(1,6)];
    
    R_k = diag([sensParams.var_baro]);
    
    y_k = u_b_baro;
    
    y_k_hat = 101325*(1-2.25577*10^-5*(-pos_k_m(3)+height_0))^5.25588;
end


K_k = P_k_m*H_k'/(H_k*P_k_m*H_k' + R_k);

P_hat_pos = (eye(9)-K_k*H_k)*P_k_m*(eye(9)-K_k*H_k)' + K_k*R_k*K_k';
% P_hat_pos = (eye(6)-K_k*H_k)*P_k_m*(eye(6)-K_k*H_k)' + K_k*R_k*K_k';

P_hat_pos = 0.5*(P_hat_pos + P_hat_pos');

x_k_hat_pos = [pos_k_m; vel_k_m; bais_acc_k_m] + K_k*(y_k - y_k_hat);
% x_k_hat_pos = [pos_k_m; vel_k_m] + K_k*(y_k - y_k_hat);


innov = (y_k - y_k_hat);

obsvervmat = obsv(F_k_1, H_k);
%% repackage

EKF.X_hat.pos_hat = x_k_hat_pos(1:3);

EKF.X_hat.vel_hat = rotMat_k*x_k_hat_pos(4:6); % put velocity back into body frame - for comparison with state

EKF.X_hat.bias_acc = x_k_hat_pos(7:9);
    
EKF.P_hat = P_hat_pos;
     
     



