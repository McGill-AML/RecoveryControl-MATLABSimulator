function [AHINF] = AHINF_attitude(Sensor, AHINF, EKF, sensParams, tStep)
% AHINF - kinematic prediction for orientation
% unit norm constrained H infinity (min max) filter
% using the predict correct structure

%notation - _k_1 is previous timestep, _k_m is predicted, _k is corrected
%estimate

global g mag
%unpackage
%states
pos_k_1 = EKF.X_hat.pos_hat;

vel_k_1 = EKF.X_hat.vel_hat;

q_k_1 = AHINF.X_hat.q_hat;

omega_k_1 = AHINF.X_hat.omega_hat;

rotMat = quat2rotmat(q_k_1);

%bias terms
% bias_acc = EKF.X_hat.bias_acc;
bias_gyr = AHINF.X_hat.bias_gyr;
bias_acc = [0;0;0]; %sensParams.bias.acc; %no accel bias so comparison is better

%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

G_k = AHINF.delta*AHINF.G_k;



%% predict
%quaternion

% % Euler discretization:
% omega_hat_m = u_b_gyr - bias_gyr;
% 
% q_k_m = q_k_1 - tStep*0.5*quatmultiply([0;omega_hat_m],q_k_1);

% % power series discretization
omega_hat_m = u_b_gyr - bias_gyr; %ang vel estimate

psi_norm = norm(omega_hat_m,2);
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

A_k_1_1 = eye(4)+tStep*[zeros(1,4); zeros(3,1), -cross_mat(omega_hat_m)];

A_k_1_2 = 0.5*tStep*[zeros(1,3); eye(3)];
       
A_k_1 = [ A_k_1_1, A_k_1_2;
          zeros(3,4), eye(3)];

% process noise weighting matrix, could change if we know gyro noise is more?
% B_k_1 = tStep*blkdiag(eye(4), eye(3)*0.1); 
B_k_1 = tStep*diag([sensParams.var_gyr; sensParams.var_gyr(1); sensParams.var_bias_gyr*.001]); 

%predict covariance matrix
AHINF.P_hat = A_k_1*AHINF.P_hat_eps*A_k_1' + B_k_1; %used to be B_k_1*B_k_1';

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

if norm(u_b_acc,2) > norm(g,2) + AHINF.accel_bound || norm(u_b_acc,2) < norm(g,2) - AHINF.accel_bound
    

    %magnetometer only
%     C_k = [dR_dq_0*mag, dR_dq_1*mag, dR_dq_2*mag, dR_dq_3*mag, zeros(3)];
    C_k = [zeros(3,1), -cross_mat(rotMat*mag), zeros(3,3)];
    
    y_k = u_b_mag;
    
%     y_k_hat = C_k*[q_k_m; bias_gyr_k_m];
    y_k_hat = rotMat*mag;
    
    R_k = diag(sensParams.var_mag);
    
    meas_size = 3;
    AHINF.use_acc = 0;

else
    

    %magnetometer and accelerometer
%     C_k = [dR_dq_0*[0;0;g], dR_dq_1*[0;0;g], dR_dq_2*[0;0;g], dR_dq_3*[0;0;g], zeros(3);
%            dR_dq_0*mag, dR_dq_1*mag, dR_dq_2*mag, dR_dq_3*mag, zeros(3)];
    C_k = [zeros(3,1), -cross_mat(rotMat*[0; 0; g]), zeros(3,3);
            zeros(3,1), -cross_mat(rotMat*mag), zeros(3,3)];
    
    y_k = [u_b_acc - bias_acc; u_b_mag];
    
%     y_k_hat = C_k*[q_k_m; bias_gyr_k_m];
    y_k_hat = [rotMat*[0; 0; g]; rotMat*mag];
    
    R_k = diag([sensParams.var_acc; sensParams.var_mag]);
    
    meas_size = 6;
    AHINF.use_acc = 1;
       
end

% adapt delta_k based on hard values of innovation
% cycle through prev innov
AHINF.innov_k(2:end) = AHINF.innov_k(1:end-1);

%innovation
r_k = y_k - y_k_hat;

%update innovation sum depending whether or not to use accelerometer data
if AHINF.use_acc
   
    AHINF.innov_k(1) = norm(tStep*AHINF.gamma(1:6,1:6)*r_k); %update prev innovation norms
else   

    AHINF.innov_k(1) = norm(tStep*AHINF.gamma(4:6,4:6)*r_k); %update prev innovation norms
end


% update the adaptive gain
if sum(AHINF.innov_k) > AHINF.innov_tresh
    if AHINF.delta < AHINF.delta_max
        AHINF.delta = AHINF.delta + AHINF.delta_rate;
    else
        AHINF.delta = AHINF.delta;
    end
else
    if AHINF.delta > 0 && AHINF.delta - AHINF.delta_rate >= 0
        AHINF.delta = AHINF.delta - AHINF.delta_rate;
    elseif AHINF.delta > 0
        AHINF.delta = 0;
    else
        AHINF.delta = AHINF.delta;
    end
end


% now do 
%Here the process is broken down based on matrix inversions - in order to
%more easily facilitate coding in c++ later on.

% find kalman gain for the quaternion:
D_3 = AHINF.P_hat*G_k'/(eye(4)-G_k*AHINF.P_hat*G_k')*G_k;

D_1 = (D_3(1:4,1:4) + eye(4))*AHINF.P_hat(1:4,1:7)*C_k';

D_2 = C_k*D_3*AHINF.P_hat*C_k' + C_k*AHINF.P_hat*C_k' + R_k;

K_k1_p1 = D_1/D_2;


r_k_D_2 = r_k'/D_2;

r_k_tilde = r_k_D_2*r_k;

q_upd_m = K_k1_p1*r_k;
q_upd_m = [1; 0.5*q_upd_m(2:4)];

x_k_tilde = q_upd_m;
% x_k_tilde = quatmultiply(q_upd_m,q_k_m);

K_k1 = K_k1_p1 + (1/norm(x_k_tilde,2) - 1)*x_k_tilde*r_k_D_2/r_k_tilde;


%find kalman gain for the bias term - since G_k = 0 for bias just use basic
%kalman equations

K_k2 = AHINF.P_hat(5:7,:)*C_k'/(R_k + C_k*AHINF.P_hat*C_k')';

K_k = [K_k1; K_k2];

L_k = (eye(7) - K_k*C_k)*AHINF.P_hat*G_k'/(eye(4) - G_k*AHINF.P_hat*G_k');

%update covariance
AHINF.P_hat_eps = (eye(7) - K_k*C_k + L_k*G_k)*AHINF.P_hat*(eye(7) - K_k*C_k + L_k*G_k)' + K_k*R_k*K_k' - L_k*L_k';

%update states
q_upd =  K_k1*r_k;
q_upd = [1; 0.5*q_upd(2:4)];

AHINF.X_hat.q_hat = quatmultiply(q_upd, q_k_m);

AHINF.X_hat.q_hat = AHINF.X_hat.q_hat/norm(AHINF.X_hat.q_hat,2); % renormalize

AHINF.X_hat.bias_gyr = bias_gyr_k_m + K_k2*r_k;

AHINF.X_hat.omega_hat = u_b_gyr - AHINF.X_hat.bias_gyr;

AHINF.P_hat_eps = 0.5*(AHINF.P_hat_eps + AHINF.P_hat_eps'); % make sure symetric





