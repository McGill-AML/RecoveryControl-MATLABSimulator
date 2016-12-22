function [SPKF_norm] = SPKF_norm_const(Sensor, SPKF_norm, EKF, sensParams, tStep)
% SPKF_norm - kinematic prediction for orientation
% this estimator uses an MRP formulation of the quaternion in a sigma point
% kalman filter to propagate the state estimate. Only estimates gryo bias
% and quaternion - angular velocity is assumed to be gyro value minus bias

%notation - _k_1 is previous timestep, _k_m is predicted, _k is corrected
%estimate

global g mag
%unpackage
%states
pos_k_1 = EKF.X_hat.pos_hat;

vel_k_1 = EKF.X_hat.vel_hat;

q_k_1 = SPKF_norm.X_hat.q_hat;

omega_k_1 = SPKF_norm.X_hat.omega_hat;

rotMat = quat2rotmat(q_k_1);

%bias terms
% bias_acc = EKF.X_hat.bias_acc;
bias_gyr = SPKF_norm.X_hat.bias_gyr;
bias_acc = sensParams.bias.acc;

%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

%%
%init covariance for sig pt calcs
%sig for prediction (process)
Q_k_1 = tStep*diag([sensParams.var_gyr;
                  sensParams.var_bias_gyr]); 

%sig for correct (update)  only use acc if within bounds
if norm(u_b_acc,2) > norm(g,2) + SPKF_norm.accel_bound || norm(u_b_acc,2) < norm(g,2) - SPKF_norm.accel_bound
    R_k = diag([sensParams.var_mag]);
    SPKF_norm.use_acc = 0;
else
    R_k = diag([sensParams.var_acc; 
                sensParams.var_mag]);
    SPKF_norm.use_acc = 1;
end
        
P_k_1_att = SPKF_norm.P_hat;

%construct matrix to find sigma point modifiers
Y = blkdiag(P_k_1_att,Q_k_1,R_k); 

S = chol(Y, 'lower');

L = length(S);

Sigma_pts_k_1 = zeros(L,2*L+1);

% sigma points are generated for MRP error, gyro bias and noise terms
if SPKF_norm.use_acc
    Sigma_pts_k_1(:,1) = [q_k_1; bias_gyr; zeros(12,1)];
else
    Sigma_pts_k_1(:,1) = [q_k_1; bias_gyr; zeros(9,1)];
end


%can probably speed most of this up by using lower triang' property of
%cholesky decomposition.
% this loop generates sigma points for estimated MRP and bias and generates
% the associated quaternion SPs as well
for ii = 1:L
    
    %make sigma points
    Sigma_pts_k_1(:,ii+1) = Sigma_pts_k_1(:,1) + sqrt(L+SPKF_norm.kappa)*S(:,ii);
    Sigma_pts_k_1(:,L+ii+1) = Sigma_pts_k_1(:,1) - sqrt(L+SPKF_norm.kappa)*S(:,ii);
    
    Sigma_pts_k_1(1:4,ii+1) = Sigma_pts_k_1(1:4,ii+1)/norm(Sigma_pts_k_1(1:4,ii+1));
    Sigma_pts_k_1(1:4,L+ii+1) = Sigma_pts_k_1(1:4,L+ii+1)/norm(Sigma_pts_k_1(1:4,L+ii+1));
    
end 


Sigma_pts_k_m = zeros(L,2*L+1);
omega_sig = zeros(3,2*L+1); %angular velocity sigma points are separate from rest as omega isnt estimated.

%this loop propagates (predict) the states using sigma points
for ii = 1:2*L+1
    %use discretized quat equation to calculate initial quat estimates
    %based on error
    %first calc estimated angular vel with sigma pt modified bias terms
    omega_sig(:,ii) = u_b_gyr - Sigma_pts_k_1(5:7,ii) - Sigma_pts_k_1(8:10,ii); %angular velocity

    %estimate orientation using estimated ang vel
    psi_norm = norm(omega_sig(:,ii),2);
    psi_k_p = sin(-0.5*psi_norm*tStep)*omega_sig(:,ii)/psi_norm;
    
    Omega_k_p = [cos(-0.5*psi_norm*tStep), -psi_k_p';
                  psi_k_p, cos(-0.5*psi_norm*tStep)*eye(3)+cross_mat(psi_k_p)];
    
    Sigma_pts_k_m(1:4,ii) = Omega_k_p*Sigma_pts_k_1(1:4,ii); %predicted quaternion sigma points
    
    
    Sigma_pts_k_m(5:7,ii) = Sigma_pts_k_1(5:7,ii) + tStep*Sigma_pts_k_1(11:13,ii) ;
    Sigma_pts_k_m(8:end,ii) = Sigma_pts_k_1(8:end,ii);
end
%find state and covariance predictions
X_k_m = 1/(L+SPKF_norm.kappa)*(SPKF_norm.kappa*Sigma_pts_k_m(1:7,1)+0.5*sum(Sigma_pts_k_m(1:7,2:2*L+1),2));

P_k_m = 1/(L+SPKF_norm.kappa)*(SPKF_norm.kappa*(Sigma_pts_k_m(1:7,1)-X_k_m(1:7))*(Sigma_pts_k_m(1:7,1)-X_k_m(1:7))');
for ii = 2:2*L+1
    P_k_m = P_k_m+1/(L+SPKF_norm.kappa)*(0.5*(Sigma_pts_k_m(1:7,ii)-X_k_m(1:7))*(Sigma_pts_k_m(1:7,ii)-X_k_m(1:7))');
end 

%%
%measurement sigma and correct
% bound so if there's large acceleration (besides gravity) dont use
if ~SPKF_norm.use_acc
    %only use magnetometer
    
    Sigma_Y = zeros(3,2*L+1);
    
    for ii = 1:2*L+1
        rotMat = quat2rotmat(Sigma_pts_k_m(1:4,ii));       
        Sigma_Y(1:3,ii) = rotMat*(mag) + Sigma_pts_k_m(14:16,ii); %magnetometer        
    end
    
else
    %use both mag and accel
    
    Sigma_Y = zeros(6,2*L+1);
    
    for ii = 1:2*L+1
        
        rotMat = quat2rotmat(Sigma_pts_k_m(1:4,ii));
        
        Sigma_Y(1:3,ii) = rotMat*([0;0;g]) + bias_acc + Sigma_pts_k_m(14:16,ii); %accel
        Sigma_Y(4:6,ii) = rotMat*(mag) + Sigma_pts_k_m(17:19,ii); %magnetometer
        
    end
    
end


y_k_hat = 1/(L+SPKF_norm.kappa)*(SPKF_norm.kappa*Sigma_Y(:,1)+0.5*sum(Sigma_Y(:,2:2*L+1),2));

%now find matrices needed for kalman gain

V_k = SPKF_norm.kappa/(L+SPKF_norm.kappa)*(Sigma_Y(:,1)-y_k_hat)*(Sigma_Y(:,1)-y_k_hat)';
for ii = 2:2*L+1
    V_k = V_k+.5/(L+SPKF_norm.kappa)*(Sigma_Y(:,ii)-y_k_hat)*(Sigma_Y(:,ii)-y_k_hat)';
end
% V_k = V_k;

U_k = SPKF_norm.kappa/(L+SPKF_norm.kappa)*(Sigma_pts_k_m(1:7,1)-X_k_m(1:7))*(Sigma_Y(:,1)-y_k_hat)';

for ii =2:2*L+1
    U_k = U_k + .5/(L+SPKF_norm.kappa)*(Sigma_pts_k_m(1:7,ii)-X_k_m(1:7))*(Sigma_Y(:,ii)-y_k_hat)';
end

%kalman gain
K_k_tilde = U_k/V_k; % classic SPKF gain

if SPKF_norm.use_acc
    r_k = ([u_b_acc; u_b_mag] - y_k_hat); %innovation term
else
    r_k = u_b_mag - y_k_hat;
end

x_k_tilde = X_k_m(1:4) + K_k_tilde(1:4,:)*r_k; %updated quat term for norm constraint

r_k_Pyy = r_k'/V_k;

K_k_1 = K_k_tilde(1:4,:) + 1/(r_k_Pyy*r_k)*(1/norm(x_k_tilde(1:4)) -1)*x_k_tilde(1:4)*r_k_Pyy;

K_k = [K_k_1; K_k_tilde(5:7,:)];



DX_k = K_k*r_k;



SPKF_norm.X_hat.bias_gyr = X_k_m(5:7) + DX_k(5:7);

SPKF_norm.X_hat.omega_hat = u_b_gyr - SPKF_norm.X_hat.bias_gyr; %ang vel is just gyro minus bias

SPKF_norm.X_hat.q_hat = X_k_m(1:4) + DX_k(1:4);

SPKF_norm.X_hat.q_hat = SPKF_norm.X_hat.q_hat/norm(SPKF_norm.X_hat.q_hat);


P_hat = P_k_m - K_k*U_k' - U_k*K_k' + K_k*V_k*K_k';


SPKF_norm.P_hat = 0.5*(P_hat + P_hat');