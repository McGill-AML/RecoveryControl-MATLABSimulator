function [COMP] = CompFilt_attitude(Sensor, COMP, EKF, sensParams, tStep,useExpData)
% Explicit Complementary filter - notation and design as used in paper
% Mahony et al - Nonlinear Complementary Filter on SO(3)


global g mag

if useExpData
    g_mod = 1;
else
    g_mod = -1;
end

mag_norm = mag/norm(mag);
%unpackage
%states
pos_k_1 = EKF.X_hat.pos_hat;

vel_k_1 = EKF.X_hat.vel_hat;

q_k_1 = COMP.X_hat.q_hat;

omega_k_1 = COMP.X_hat.omega_hat;

rotMat_k_1 = quat2rotmat(q_k_1);

%bias terms
% bias_acc = EKF.X_hat.bias_acc;
bias_gyr = COMP.X_hat.bias_gyr;
bias_acc = [0;0;0]; %sensParams.bias.acc; % for the sake of comparing attitude estimators ignore accel bias


%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

%%

if norm(u_b_acc,2) > norm(g,2) + COMP.accel_bound || norm(u_b_acc,2) < norm(g,2) - COMP.accel_bound

    
    %normalize mag and accel measurements now
    u_b_mag = u_b_mag/norm(u_b_mag);

    v_b_mag = rotMat_k_1'*mag_norm;
    
    w_mes = -vex(COMP.mag_k_i/2*(u_b_mag*v_b_mag' - v_b_mag*u_b_mag'));
    

else
    
    %normalize mag and accel measurements now
    u_b_acc = u_b_acc/norm(u_b_acc);
    u_b_mag = u_b_mag/norm(u_b_mag);

    v_b_mag = rotMat_k_1'*mag_norm;
    v_b_acc = rotMat_k_1'*[0;0;-g_mod];
    
    w_mes = -vex(COMP.mag_k_i/2*(u_b_mag*v_b_mag' - v_b_mag*u_b_mag')+COMP.acc_k_i/2*((u_b_acc-bias_acc)*v_b_acc' - v_b_acc*(u_b_acc-bias_acc)'));  
    
end

    %debug term
    COMP.w_mes = w_mes;
%use Euler integration to compute bias update and quaternion update (with
%brute force normalization)
%gyro bias
bias_gyr = bias_gyr - tStep*COMP.gyr_bias_k_i*w_mes;

%quaternion
p = u_b_gyr - bias_gyr + COMP.k_p*w_mes;

% fionas old quat convention
% psi_norm = norm(p,2);
% psi_k_p = sin(-0.5*psi_norm*tStep)*p/psi_norm;
% 
% Omega_k_p = [cos(-0.5*psi_norm*tStep), -psi_k_p';
%             psi_k_p, cos(-0.5*psi_norm*tStep)*eye(3)+cross_mat(psi_k_p)];

psi_norm = norm(p,2);

psi_k_p = sin(0.5*psi_norm*tStep)*p/psi_norm;

%ang vel matrix used in discretization
Omega_k_p = [cos(0.5*psi_norm*tStep), -psi_k_p';
            psi_k_p, cos(0.5*psi_norm*tStep)*eye(3)-cross_mat(psi_k_p)];


q_k = Omega_k_p*q_k_1;

q_k = q_k/norm(q_k,2); %normalize quaternion

COMP.X_hat.q_hat = q_k;
COMP.X_hat.omega_hat = u_b_gyr - bias_gyr;
COMP.X_hat.bias_gyr = bias_gyr;






