function [COMP] = CompFilt_attitude(Sensor, COMP, EKF, tStep)
% Explicit Complementary filter - notation and design as used in paper
% Mahony et al - Nonlinear Complementary Filter on SO(3)


global g mag
%unpackage
%states
pos_k_1 = EKF.X_hat.pos_hat;

vel_k_1 = EKF.X_hat.vel_hat;

q_k_1 = COMP.X_hat.q_hat;

omega_k_1 = COMP.X_hat.omega_hat;

rotMat_k_1 = quat2rotmat(q_k_1);

%bias terms
bias_acc = EKF.X_hat.bias_acc;
bias_gyr = COMP.X_hat.bias_gyr;


%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

%%

if norm(u_b_acc,2) > norm(g,2) + COMP.accel_bound || norm(u_b_acc,2) < norm(g,2) - COMP.accel_bound

    v_b_mag = rotMat_k_1*mag;
    
    w_mes = -vex(COMP.mag_k_i/2*(u_b_mag*v_b_mag' - v_b_mag*u_b_mag'));
    

else
    
    v_b_mag = rotMat_k_1*mag;
    v_b_acc = rotMat_k_1*[0;0;g];
    
    w_mes = -vex(COMP.mag_k_i/2*(u_b_mag*v_b_mag' - v_b_mag*u_b_mag')+COMP.acc_k_i/2*((u_b_acc-bias_acc)*v_b_acc' - v_b_acc*(u_b_acc-bias_acc)'));  
    
end

%use Euler integration to compute bias update and quaternion update (with
%brute force normalization)
%gyro bias
bias_gyr = bias_gyr - tStep*COMP.gyr_bias_k_i*w_mes;

%quaternion
p = u_b_gyr - bias_gyr + COMP.k_p*w_mes;

q_dot_hat = 1/2*quatmultiply( q_k_1, [0;p]);

q_k = q_k_1 + tStep*q_dot_hat;

q_k = q_k/norm(q_k,2); %normalize quaternion

COMP.X_hat.q_hat = q_k;
COMP.X_hat.omega_hat = omega_k_1 - bias_gyr;
COMP.X_hat.bias_gyr = bias_gyr;






