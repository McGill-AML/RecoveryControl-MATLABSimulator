function [EKF_att] = EKF_attitude_3(Sensor, EKF_att, EKF_pos, sensParams, tStep)
% EKF_att - kinematic prediction for orientation
% unit norm constrained H infinity (min max) filter
% using the predict correct structure

%notation - _k_1 is previous timestep, _k_m is predicted, _k is corrected
%estimate

global g mag
%unpackage
%states
pos_k_1 = EKF_pos.X_hat.pos_hat;

vel_k_1 = EKF_pos.X_hat.vel_hat;

q_k_1 = EKF_att.X_hat.q_hat;

omega_k_1 = EKF_att.X_hat.omega_hat;

rotMat = quat2rotmat(q_k_1);

%bias terms
% bias_acc = EKF_pos.X_hat.bias_acc;
bias_gyr = EKF_att.X_hat.bias_gyr;
bias_acc = sensParams.bias.acc; %no accel bias so comparison is better

%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;



%% predict
%quaternion

% % Euler discretization:
% omega_hat_m = u_b_gyr - bias_gyr;
% 
% q_k_m = q_k_1 - tStep*0.5*quatmultiply([0;omega_hat_m],q_k_1);

% % power series discretization
omega_hat_m = u_b_gyr - bias_gyr; %ang vel estimate

psi_norm = norm(omega_hat_m,2);

% psi_k_p = sin(0.5*psi_norm*tStep)*omega_hat_m/psi_norm;
% 
% %ang vel matrix used in discretization
% Omega_k_m = [cos(0.5*psi_norm*tStep), -psi_k_p';
%             psi_k_p, cos(0.5*psi_norm*tStep)*eye(3)-cross_mat(psi_k_p)];
        
psi_k_p = sin(-0.5*psi_norm*tStep)*omega_hat_m/psi_norm;

%ang vel matrix used in discretization
Omega_k_m = [cos(-0.5*psi_norm*tStep), -psi_k_p';
            psi_k_p, cos(-0.5*psi_norm*tStep)*eye(3)+cross_mat(psi_k_p)];

q_k_m = Omega_k_m*q_k_1;

%renormalize quaternion (just in case)
q_k_m = q_k_m/norm(q_k_m,2);

bias_gyr_k_m = bias_gyr;

%discretized (euler discretization) then linearized state transition matrix
% A_k_1_1 = eye(4)+[0, -omega_hat_m'; omega_hat_m, cross_mat(omega_hat_m)];
% 
% A_k_1_2 = [q_k_m(2), q_k_m(3), q_k_m(4);
%            -q_k_m(1), -q_k_m(4), q_k_m(3);
%            q_k_m(4), -q_k_m(1), -q_k_m(2);
%            -q_k_m(3), q_k_m(2), -q_k_m(1)];
%        
% A_k_1 = [ -0.5*tStep*[A_k_1_1, A_k_1_2];
%           zeros(3,4), eye(3)];

% try 1 - went to [0 0 0 1]??
% A_k_1_1 = eye(3)-cross_mat(omega_hat_m)*sin(psi_norm*tStep)/psi_norm + cross_mat(omega_hat_m)^2*(1-cos(psi_norm*tStep))/psi_norm^2;
% 
% A_k_1_2 = -cross_mat(omega_hat_m)*(1-cos(psi_norm*tStep))/psi_norm^2 + eye(3)*tStep + cross_mat(omega_hat_m)^2*(psi_norm*tStep-sin(psi_norm*tStep))/psi_norm^3;
% 
% A_k_1 = [ 1, zeros(1,6);
%         zeros(3,1), A_k_1_1,  A_k_1_2;
%         zeros(3,4), eye(3)];

A_k_1_1 = eye(4)+tStep*[zeros(1,4); zeros(3,1), -cross_mat(omega_hat_m)];

A_k_1_2 = 0.5*tStep*[zeros(1,3); eye(3)];
       
A_k_1 = [ A_k_1_1, A_k_1_2;
          zeros(3,4), eye(3)];


% process noise 
Q_k = tStep*diag([sensParams.var_gyr; sensParams.var_gyr(1); sensParams.var_bias_gyr*.001]); 

% Q_k = [(sensParams.var_gyr(1)*tStep+1/3*sensParams.var_bias_gyr(1)*tStep^3)*eye(4), -(0.5*sensParams.var_bias_gyr(1)*tStep^2)*[zeros(1,3); eye(3)];
%         -(0.5*sensParams.var_bias_gyr(1)*tStep^2)*[zeros(3,1), eye(3)], sensParams.var_bias_gyr(1)*tStep*eye(3)];

%predict covariance matrix
EKF_att.P_hat = A_k_1*EKF_att.P_hat*A_k_1' + Q_k;

%% update 

% dR_dq_0 = [q_k_m(1), -q_k_m(4), q_k_m(3);
%              q_k_m(4), q_k_m(1), -q_k_m(2);
%              -q_k_m(3), q_k_m(2), q_k_m(1)];
% 
% dR_dq_1 = [q_k_m(2), q_k_m(3), q_k_m(4);
%              q_k_m(3), -q_k_m(2), -q_k_m(1);
%              q_k_m(4), q_k_m(1), -q_k_m(2)];
%          
% dR_dq_2 = [-q_k_m(3), q_k_m(2), q_k_m(1);
%              q_k_m(2), q_k_m(3), q_k_m(4);
%              -q_k_m(1), q_k_m(4), -q_k_m(3)];
%          
%          
% dR_dq_3 = [-q_k_m(4), -q_k_m(1), q_k_m(2);
%              q_k_m(1), -q_k_m(4), q_k_m(3);
%              q_k_m(2), q_k_m(3), q_k_m(4)];
         
rotMat = quat2rotmat( q_k_m );

if norm(u_b_acc,2) > norm(g,2) + EKF_att.accel_bound || norm(u_b_acc,2) < norm(g,2) - EKF_att.accel_bound
    
    %normalize mag and accel measurements now
%     u_b_acc = u_b_acc/norm(u_b_acc);
%     u_b_mag = u_b_mag/norm(u_b_mag);

    
    %magnetometer only
%     C_k = [dR_dq_0*mag, dR_dq_1*mag, dR_dq_2*mag, dR_dq_3*mag, zeros(3)];

    C_k = [zeros(3,1), -cross_mat(rotMat*mag), zeros(3,3)];
    
    R_k = diag(sensParams.var_mag);
    
    y_k = u_b_mag;
    
%     y_k_hat = C_k*[q_k_m; bias_gyr_k_m];
    y_k_hat = rotMat*mag;
    
    meas_size = 3;

else
    
    %magnetometer and accelerometer
    
    %normalize mag and accel measurements now
%     u_b_acc = u_b_acc/norm(u_b_acc);
%     u_b_mag = u_b_mag/norm(u_b_mag);

    
    
%     C_k = [dR_dq_0*[0;0;g], dR_dq_1*[0;0;g], dR_dq_2*[0;0;g], dR_dq_3*[0;0;g], zeros(3);
%            dR_dq_0*mag, dR_dq_1*mag, dR_dq_2*mag, dR_dq_3*mag, zeros(3)];

    C_k = [zeros(3,1), -cross_mat(rotMat*[0; 0; g]), zeros(3,3);
            zeros(3,1), -cross_mat(rotMat*mag), zeros(3,3)];
       
    R_k = diag([sensParams.var_acc; sensParams.var_mag]);
    
    y_k = [u_b_acc; u_b_mag];
    
%     y_k_hat = C_k*[q_k_m; bias_gyr_k_m];
    y_k_hat = [rotMat*[0; 0; g]; rotMat*mag];
    
    
    meas_size = 6;
       
end

%Here the process is broken down based on matrix inversions - in order to
%more easily facilitate coding in c++ later on.

% find kalman gain for the quaternion:

D_1 = EKF_att.P_hat(1:4,:)*C_k';

D_2 = C_k*EKF_att.P_hat*C_k' + R_k;

K_k1_p1 = D_1/D_2;

r_k = y_k - y_k_hat;

r_k_D_2 = r_k'/D_2;

r_k_tilde = r_k_D_2*r_k;

q_upd_m = K_k1_p1*r_k;
q_upd_m = [1; 0.5*q_upd_m(2:4)];

x_k_tilde = q_upd_m;
% x_k_tilde = quatmultiply(K_k1_p1*r_k,q_k_m);

K_k1 = K_k1_p1 + (1/norm(x_k_tilde,2) - 1)*x_k_tilde*r_k_D_2/r_k_tilde;


%find kalman gain for the bias term - since G_k = 0 for bias just use basic
%kalman equations

K_k2 = EKF_att.P_hat(5:7,:)*C_k'/(R_k + C_k*EKF_att.P_hat*C_k')';

K_k = [K_k1; K_k2];


%update covariance
EKF_att.P_hat = (eye(7) - K_k*C_k)*EKF_att.P_hat*(eye(7) - K_k*C_k)' + K_k*R_k*K_k';

%update states
q_upd =  K_k1*r_k;
q_upd = [1; 0.5*q_upd(2:4)];

EKF_att.X_hat.q_hat = quatmultiply(q_upd, q_k_m);

EKF_att.X_hat.q_hat = EKF_att.X_hat.q_hat/norm(EKF_att.X_hat.q_hat,2); % renormalize

EKF_att.X_hat.bias_gyr = bias_gyr_k_m + K_k2*r_k;

EKF_att.X_hat.omega_hat = u_b_gyr - EKF_att.X_hat.bias_gyr;

EKF_att.P_hat = 0.5*(EKF_att.P_hat + EKF_att.P_hat'); % make sure symetric






