function [SRSPKF] = SRSPKF_attitude(Sensor, SRSPKF, EKF, sensParams, tStep)
% SPKF - kinematic prediction for orientation
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

q_k_1 = SRSPKF.X_hat.q_hat;

omega_k_1 = SRSPKF.X_hat.omega_hat;

rotMat = quat2rotmat(q_k_1);


%bias terms
% bias_acc = EKF.X_hat.bias_acc;
bias_gyr = SRSPKF.X_hat.bias_gyr;
bias_acc = sensParams.bias.acc;

%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

MRP_0 = zeros(3,1); %q_k_1(2:4)/(1+q_k_1(1));

%%

Q_k_1 = tStep*diag([sensParams.var_gyr;  
              sensParams.var_bias_gyr].^0.5); 
          
          

if norm(u_b_acc,2) > norm(g,2) + SRSPKF.accel_bound || norm(u_b_acc,2) < norm(g,2) - SRSPKF.accel_bound
    
    R_k =  diag(sensParams.var_mag.^0.5);    
    
    L = length(SRSPKF.S_hat)+length(Q_k_1)+length(R_k);
    SRSPKF.use_acc = 0;
    
else
    R_k = diag([sensParams.var_acc;
                sensParams.var_mag].^0.5);
    
    L = length(SRSPKF.S_hat)+length(Q_k_1)+length(R_k);
    SRSPKF.use_acc = 1;
    
end


%normalize mag and accel measurements now
% u_b_acc = u_b_acc/norm(u_b_acc);
% u_b_mag = u_b_mag/norm(u_b_mag);


S = blkdiag(SRSPKF.S_hat,Q_k_1, R_k);

Sigma_pts_k_1 = zeros(L,2*L+1);


% sigma points are generated for MRP error, gyro bias and noise terms
if SRSPKF.use_acc
    Sigma_pts_k_1(:,1) = [MRP_0; bias_gyr; zeros(12,1)];
else
    Sigma_pts_k_1(:,1) = [MRP_0; bias_gyr; zeros(9,1)];
end

q_k_1_err = zeros(4,2*L+1);
q_k_1_sig = zeros(4,2*L+1);

q_k_1_sig(:,1) = q_k_1;
%can probably speed most of this up by using lower triang' property of
%cholesky decomposition.
% this loop generates sigma points for estimated MRP and bias and generates
% the associated quaternion SPs as well


lambda = SRSPKF.alpha^2*(L-SRSPKF.kappa)-L;

for ii = 1:L
    
    %make sigma points
    Sigma_pts_k_1(:,ii+1) = Sigma_pts_k_1(:,1) + sqrt(L+lambda)*S(:,ii);
    Sigma_pts_k_1(:,L+ii+1) = Sigma_pts_k_1(:,1) - sqrt(L+lambda)*S(:,ii);
    
    %convert MRP sigma points into quaternions by crassidis' chose a = f = 1
    eta_k_d_pos = (1-Sigma_pts_k_1(1:3,ii+1)'*Sigma_pts_k_1(1:3,ii+1))/(1+Sigma_pts_k_1(1:3,ii+1)'*Sigma_pts_k_1(1:3,ii+1));
    eta_k_d_neg = (1-Sigma_pts_k_1(1:3,L+ii+1)'*Sigma_pts_k_1(1:3,L+ii+1))/(1+Sigma_pts_k_1(1:3,L+ii+1)'*Sigma_pts_k_1(1:3,L+ii+1));
    
    q_k_1_err(:,ii+1) = [eta_k_d_pos; (1+eta_k_d_pos)*Sigma_pts_k_1(1:3,ii+1)]; %positive error quaternion
    q_k_1_err(:,L+ii+1) = [eta_k_d_neg; (1+eta_k_d_neg)*Sigma_pts_k_1(1:3,L+ii+1)]; %neg error quaternion
    
    q_k_1_sig(:,ii+1) = quatmultiply(q_k_1_err(:,ii+1),q_k_1_sig(:,1)); %compute quaternion sigma pts
    q_k_1_sig(:,L+ii+1) = quatmultiply(q_k_1_err(:,L+ii+1),q_k_1_sig(:,1));
end 


q_k_m_sig = zeros(4,2*L+1);
q_k_err = zeros(4,2*L+1);
Sigma_pts_k_m = zeros(L,2*L+1);
omega_sig = zeros(3,2*L+1); %angular velocity sigma points are separate from rest as omega isnt estimated.

%this loop propagates (predict) the states using sigma points
for ii = 1:2*L+1
    %use discretized quat equation to calculate initial quat estimates
    %based on error
    %first calc estimated angular vel with sigma pt modified bias terms
    omega_sig(:,ii) = u_b_gyr - Sigma_pts_k_1(4:6,ii) - Sigma_pts_k_1(7:9,ii); %angular velocity

    %estimate orientation using estimated ang vel
    psi_norm = norm(omega_sig(:,ii),2);
    psi_k_p = sin(-0.5*psi_norm*tStep)*omega_sig(:,ii)/psi_norm;
    
    Omega_k_p = [cos(-0.5*psi_norm*tStep), -psi_k_p';
                  psi_k_p, cos(-0.5*psi_norm*tStep)*eye(3)+cross_mat(psi_k_p)];
    
    q_k_m_sig(:,ii) = Omega_k_p*q_k_1_sig(:,ii); %predicted quaternion sigma points
    
    q_k_err(:,ii) = quatmultiply(q_k_m_sig(:,ii),[q_k_m_sig(1,1);-q_k_m_sig(2:4,1)]); %convert back to error
    %orientation propagated values
    Sigma_pts_k_m(1:3,ii) = q_k_err(2:4,ii)/(1+q_k_err(1,ii)); %convert quat error to MRP
    
    
    Sigma_pts_k_m(4:6,ii) = Sigma_pts_k_1(4:6,ii) + tStep*Sigma_pts_k_1(10:12,ii) ;
    Sigma_pts_k_m(7:end,ii) = Sigma_pts_k_1(7:end,ii);
end

lambda = SRSPKF.alpha^2*(L-SRSPKF.kappa)-L;

%find state and covariance predictions
X_k_m = 1/(L+lambda)*(lambda*Sigma_pts_k_m(1:6,1)+0.5*sum(Sigma_pts_k_m(1:6,2:2*L+1),2));


[~, S_k_m] = qr([sqrt(0.5/(L+lambda))*(bsxfun(@minus, Sigma_pts_k_m(1:6,2:2*L+1), X_k_m(1:6)))]');

S_k_m = S_k_m(1:6,1:6);

if sqrt((lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta))) < 0
    S_k_m = cholupdate(S_k_m, sqrt(-(lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta)))*(Sigma_pts_k_m(1:6,1)-X_k_m(1:6)), '-');
else
    S_k_m = cholupdate(S_k_m, sqrt((lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta)))*(Sigma_pts_k_m(1:6,1)-X_k_m(1:6)), '+');
end


%%
%measurement sigma and correct
% bound so if there's large acceleration (besides gravity) dont use
if ~SRSPKF.use_acc
    %only use magnetometer
    
    Sigma_Y = zeros(3,2*L+1);
    
    for ii = 1:2*L+1
        rotMat = quat2rotmat(q_k_m_sig(:,ii));       
        Sigma_Y(1:3,ii) = rotMat*(mag) + Sigma_pts_k_m(13:15,ii); %magnetometer        
    end
    
    
    meas_size = 3;
    
else
    %use both mag and accel
    
    Sigma_Y = zeros(6,2*L+1);
    
    for ii = 1:2*L+1
        
        rotMat = quat2rotmat(q_k_m_sig(:,ii));
        
        Sigma_Y(1:3,ii) = rotMat*([0;0;g]) + bias_acc + Sigma_pts_k_m(13:15,ii); %accel
        Sigma_Y(4:6,ii) = rotMat*(mag) + Sigma_pts_k_m(16:18,ii); %magnetometer
        
    end
    
    
            
    meas_size = 6;
end


y_k_hat = 1/(L+lambda)*(lambda*Sigma_Y(:,1)+0.5*sum(Sigma_Y(:,2:2*L+1),2));

%now find matrices needed for kalman gain

[~, S_y_k] = qr([sqrt(0.5/(L+lambda))*(bsxfun(@minus, Sigma_Y(:, 2:2*L+1), y_k_hat))]');

S_y_k = S_y_k(1:meas_size,1:meas_size);

if (lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta)) < 0
    S_y_k = cholupdate(S_y_k, sqrt(abs(lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta)))*(Sigma_Y(:,1)-y_k_hat), '-')';
else
    S_y_k = cholupdate(S_y_k, sqrt((lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta)))*(Sigma_Y(:,1)-y_k_hat), '+')';
end



U_k = (lambda/(L+lambda)+(1-SRSPKF.alpha^2+SRSPKF.beta))*(Sigma_pts_k_m(1:6,1)-X_k_m(1:6))*(Sigma_Y(:,1)-y_k_hat)';

for ii =2:2*L+1
    U_k = U_k + .5/(L+lambda)*(Sigma_pts_k_m(1:6,ii)-X_k_m(1:6))*(Sigma_Y(:,ii)-y_k_hat)';
end

%kalman gain
K_k = (U_k/S_y_k')/S_y_k;

if SRSPKF.use_acc
    DX_k = K_k*([u_b_acc; u_b_mag] - y_k_hat);
else   
    DX_k = K_k*(u_b_mag - y_k_hat);
end


SRSPKF.X_hat.bias_gyr = X_k_m(4:6) + DX_k(4:6);

SRSPKF.X_hat.omega_hat = u_b_gyr - SRSPKF.X_hat.bias_gyr; %ang vel is just gyro minus bias

%need to convert MRP to quaternions

q_k_upd(1) = (1-DX_k(1:3)'*DX_k(1:3))/(1+DX_k(1:3)'*DX_k(1:3));

q_k_upd(2:4,1) = (1+q_k_upd(1))*DX_k(1:3);

SRSPKF.X_hat.q_hat = quatmultiply(q_k_upd,q_k_m_sig(:,1));


upd =  K_k*S_y_k;
for ii = 1:meas_size
    S_k_m = cholupdate(S_k_m, upd(:,ii), '-');
end


SRSPKF.S_hat = S_k_m';




