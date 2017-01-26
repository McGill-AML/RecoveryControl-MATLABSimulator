function [SPKF_full] = SPKF_full_state(Sensor, SPKF_full, sensParams, tStep, iSim)
% SPKF_full - kinematic prediction for orientation
% this estimator uses an MRP formulation of the quaternion in a sigma point
% kalman filter to propagate the state estimate. Only estimates gryo bias
% and quaternion - angular velocity is assumed to be gyro value minus bias

%notation - _k_1 is previous timestep, _k_m is predicted, _k is corrected
%estimate

%YO YOUR VELOCITY IS ESTIMATED IN INERTIAL FRAME HERE - SO IT'S GONNA LOOK
%DIFFERENT THAN YOUR TRUE STATE

global g mag
%unpackage
%states
pos_k_1 = SPKF_full.X_hat.pos_hat;

vel_k_1 = SPKF_full.X_hat.vel_hat;

q_k_1 = SPKF_full.X_hat.q_hat;

omega_k_1 = SPKF_full.X_hat.omega_hat;

rotMat = quat2rotmat(q_k_1);

%bias terms
bias_acc = SPKF_full.X_hat.bias_acc;
bias_gyr = SPKF_full.X_hat.bias_gyr;
% bias_acc = sensParams.bias.acc;

%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

lat_0 = sensParams.gps_init(1);
long_0 = sensParams.gps_init(2);
height_0 = sensParams.gps_init(3);
Me  = sensParams.gps_init(4);
Ne  = sensParams.gps_init(5);

% baro_0 = sensParams.baro_init;

MRP_0 = zeros(3,1); %q_k_1(2:4)/(1+q_k_1(1));

%%
%init covariance for sig pt calcs
%sig for prediction (process)
Q_k_1 = tStep*diag([sensParams.var_acc;
              sensParams.var_gyr;
              sensParams.var_bias_acc;
              sensParams.var_bias_gyr]); 

%sig for correct (update)       
if mod(iSim,sensParams.GPS_rate) == 0
    R_k = diag([sensParams.var_mag;
                sensParams.var_gps*10;
                sensParams.var_baro]);
        
else
    R_k = diag([sensParams.var_mag
                sensParams.var_baro]);
end
        
P_k_1 = SPKF_full.P_hat;

%construct matrix to find sigma point modifiers
Y = blkdiag(P_k_1,Q_k_1,R_k); 

S = chol(Y, 'lower');

L = length(S);

Sigma_pts_k_1 = zeros(L,2*L+1);

% sigma points are generated for MRP error, gyro bias and noise terms
if mod(iSim,sensParams.GPS_rate) == 0
    Sigma_pts_k_1(:,1) = [pos_k_1; vel_k_1; MRP_0; bias_acc; bias_gyr; zeros(21,1)];
else
    Sigma_pts_k_1(:,1) = [pos_k_1; vel_k_1; MRP_0; bias_acc; bias_gyr; zeros(16,1)];
end

q_k_1_err = zeros(4,2*L+1);
q_k_1_sig = zeros(4,2*L+1);

q_k_1_sig(:,1) = q_k_1;
%can probably speed most of this up by using lower triang' property of
%cholesky decomposition.
% this loop generates sigma points for estimated MRP and bias and generates
% the associated quaternion SPs as well

lambda = SPKF_full.alpha^2*(L-SPKF_full.kappa)-L;

a = 1;
f = 1.5;

for ii = 1:L
    
    %make sigma points
    Sigma_pts_k_1(:,ii+1) = Sigma_pts_k_1(:,1) + sqrt(L+lambda)*S(:,ii);
    Sigma_pts_k_1(:,L+ii+1) = Sigma_pts_k_1(:,1) - sqrt(L+lambda)*S(:,ii);
    
    %convert MRP sigma points into quaternions by crassidis'
    % old version where a = f = 1
%     eta_k_d_pos = (1-Sigma_pts_k_1(7:9,ii+1)'*Sigma_pts_k_1(7:9,ii+1))/(1+Sigma_pts_k_1(7:9,ii+1)'*Sigma_pts_k_1(7:9,ii+1));
%     eta_k_d_neg = (1-Sigma_pts_k_1(7:9,L+ii+1)'*Sigma_pts_k_1(7:9,L+ii+1))/(1+Sigma_pts_k_1(7:9,L+ii+1)'*Sigma_pts_k_1(7:9,L+ii+1));
%     
%     q_k_1_err(:,ii+1) = [eta_k_d_pos; (1+eta_k_d_pos)*Sigma_pts_k_1(7:9,ii+1)]; %positive error quaternion
%     q_k_1_err(:,L+ii+1) = [eta_k_d_neg; (1+eta_k_d_neg)*Sigma_pts_k_1(7:9,L+ii+1)]; %neg error quaternion
%     
% new version with a = f as given by crassidis'
    sigNorm_pos= norm(Sigma_pts_k_1(7:9,ii+1),2);
    sigNorm_neg = norm(Sigma_pts_k_1(7:9,L+ii+1),2);
    eta_k_d_pos = (-a*sigNorm_pos^2 + f*sqrt(f^2 + (1-a^2)*sigNorm_pos^2))/(f^2+sigNorm_pos^2);
    eta_k_d_neg = (-a*sigNorm_neg^2 + f*sqrt(f^2 + (1-a^2)*sigNorm_neg^2))/(f^2+sigNorm_neg^2);
    
    q_k_1_err(:,ii+1) = [eta_k_d_pos; (a+eta_k_d_pos)/f*Sigma_pts_k_1(7:9,ii+1)]; %positive error quaternion
    q_k_1_err(:,L+ii+1) = [eta_k_d_neg; (a+eta_k_d_neg)/f*Sigma_pts_k_1(7:9,L+ii+1)]; %neg error quaternion
    
    
    
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
    omega_sig(:,ii) = u_b_gyr - Sigma_pts_k_1(13:15,ii) - Sigma_pts_k_1(19:21,ii); %angular velocity

    %estimate orientation using estimated ang vel
    psi_norm = norm(omega_sig(:,ii),2);
    psi_k_p = sin(-0.5*psi_norm*tStep)*omega_sig(:,ii)/psi_norm;
    
    Omega_k_p = [cos(-0.5*psi_norm*tStep), -psi_k_p';
                  psi_k_p, cos(-0.5*psi_norm*tStep)*eye(3)+cross_mat(psi_k_p)];
    
    q_k_m_sig(:,ii) = Omega_k_p*q_k_1_sig(:,ii); %predicted quaternion sigma points
    
    q_k_err(:,ii) = quatmultiply(q_k_m_sig(:,ii),[q_k_m_sig(1,1);-q_k_m_sig(2:4,1)]); %convert back to error
    %orientation propagated values
    Sigma_pts_k_m(7:9,ii) = q_k_err(2:4,ii)/(1+q_k_err(1,ii)); %convert quat error to MRP
    
    rotMat = quat2rotmat(q_k_m_sig(:,ii));
    
    %position propagated values
    Sigma_pts_k_m(1:3,ii) = Sigma_pts_k_1(1:3,ii) + tStep*Sigma_pts_k_1(4:6,ii);
    %velocity propagated values
    Sigma_pts_k_m(4:6,ii) = Sigma_pts_k_1(4:6,ii) + tStep*rotMat'*(u_b_acc - Sigma_pts_k_1(10:12,ii) - Sigma_pts_k_1(16:18,ii)) + tStep*[0;0;-g];
    %bias propagated values
    Sigma_pts_k_m(10:15,ii) = Sigma_pts_k_1(10:15,ii) + tStep*Sigma_pts_k_1(22:27,ii) ;
    %noise propagated values - might not actually use these anymore
    Sigma_pts_k_m(16:end,ii) = Sigma_pts_k_1(16:end,ii);
end
%find state and covariance predictions


X_k_m = 1/(L+lambda)*(lambda*Sigma_pts_k_m(1:15,1)+0.5*sum(Sigma_pts_k_m(1:15,2:2*L+1),2));

P_k_m = (lambda/(L+lambda)+(1-SPKF_full.alpha^2+SPKF_full.beta))*((Sigma_pts_k_m(1:15,1)-X_k_m(1:15))*(Sigma_pts_k_m(1:15,1)-X_k_m(1:15))');
for ii = 2:2*L+1
    P_k_m = P_k_m+0.5/(L+lambda)*((Sigma_pts_k_m(1:15,ii)-X_k_m(1:15))*(Sigma_pts_k_m(1:15,ii)-X_k_m(1:15))');
end 

%%
%measurement sigma and correct
% only use gps at the GPS udate rate, must be multiple of timestep -
% otherwise wont use properly

%normalize mag and accel measurements now
u_b_acc = u_b_acc/norm(u_b_acc); %here we dont end up using the acceleration again, but incase we end up at some point
u_b_mag = u_b_mag/norm(u_b_mag);


if mod(iSim,sensParams.GPS_rate) == 0
    R_k = diag([sensParams.var_mag
                sensParams.var_gps
                sensParams.var_baro]);
    
    Sigma_Y = zeros(9,2*L+1);
    for ii = 1:2*L+1
        rotMat = quat2rotmat(q_k_m_sig(:,ii));       
        Sigma_Y(1:3,ii) = rotMat*(mag) + Sigma_pts_k_m(28:30,ii); %magnetometer    
        
        Sigma_Y(4,ii) = Sigma_pts_k_m(1,ii)/(Me+height_0+Sigma_pts_k_m(3,ii))*180/pi + lat_0 + Sigma_pts_k_m(31,ii);
        Sigma_Y(5,ii) = -Sigma_pts_k_m(2,ii)/((Ne+height_0+Sigma_pts_k_m(3,ii))*cos(lat_0*pi/180.0))*180/pi + long_0 + Sigma_pts_k_m(32,ii);
        Sigma_Y(6,ii) = Sigma_pts_k_m(3,ii) + height_0 + Sigma_pts_k_m(33,ii);
        Sigma_Y(7,ii) = Sigma_pts_k_m(4,ii) + Sigma_pts_k_m(34,ii);
        Sigma_Y(8,ii) = Sigma_pts_k_m(5,ii) + Sigma_pts_k_m(35,ii);
        Sigma_Y(9,ii) = 101325*(1-2.25577*10^-5*(Sigma_pts_k_m(3,ii)+height_0))^5.25588 + Sigma_pts_k_m(36,ii);
    end
    

else
    R_k = diag([sensParams.var_mag
                sensParams.var_baro]);
    
    Sigma_Y = zeros(4,2*L+1);
    
    for ii = 1:2*L+1
        rotMat = quat2rotmat(q_k_m_sig(:,ii));       
        Sigma_Y(1:3,ii) = rotMat*(mag) + Sigma_pts_k_m(28:30,ii); %magnetometer       
        Sigma_Y(4,ii) = 101325*(1-2.25577*10^-5*(Sigma_pts_k_m(3,ii)+height_0))^5.25588 + Sigma_pts_k_m(31,ii);
    end
    
end


y_k_hat = 1/(L+lambda)*(lambda*Sigma_Y(:,1)+0.5*sum(Sigma_Y(:,2:2*L+1),2));

%now find matrices needed for kalman gain

V_k = (lambda/(L+lambda)+(1-SPKF_full.alpha^2+SPKF_full.beta))*(Sigma_Y(:,1)-y_k_hat)*(Sigma_Y(:,1)-y_k_hat)';
for ii = 2:2*L+1
    V_k = V_k+0.5/(L+lambda)*(Sigma_Y(:,ii)-y_k_hat)*(Sigma_Y(:,ii)-y_k_hat)';
end
% V_k = V_k + R_k;

U_k = (lambda/(L+lambda)+(1-SPKF_full.alpha^2+SPKF_full.beta))*(Sigma_pts_k_m(1:15,1)-X_k_m(1:15))*(Sigma_Y(:,1)-y_k_hat)';

for ii =2:2*L+1
    U_k = U_k + 0.5/(L+lambda)*(Sigma_pts_k_m(1:15,ii)-X_k_m(1:15))*(Sigma_Y(:,ii)-y_k_hat)';
end

%kalman gain
K_k = U_k/V_k;

if mod(iSim,sensParams.GPS_rate) == 0
    DX_k = K_k*([ u_b_mag; u_b_gps; u_b_baro] - y_k_hat);
else   
    DX_k = K_k*([ u_b_mag; u_b_baro] - y_k_hat);
end

SPKF_full.X_hat.pos_hat =  X_k_m(1:3) + DX_k(1:3);

SPKF_full.X_hat.vel_hat =  X_k_m(4:6) + DX_k(4:6);

SPKF_full.X_hat.bias_acc = X_k_m(10:12) + DX_k(10:12);

SPKF_full.X_hat.bias_gyr = X_k_m(13:15) + DX_k(13:15);

SPKF_full.X_hat.omega_hat = u_b_gyr - SPKF_full.X_hat.bias_gyr; %ang vel is just gyro minus bias

%need to convert MRP to quaternions
pNorm = norm(DX_k(7:9),2);
q_k_upd(1) = (-a*pNorm^2 + f*sqrt(f^2+(1-a^2)*pNorm^2))/(f^2+pNorm^2);

q_k_upd(2:4,1) = (a+q_k_upd(1))/f*DX_k(7:9);

q_k_upd = q_k_upd/norm(q_k_upd);

SPKF_full.X_hat.q_hat = quatmultiply(q_k_upd,q_k_m_sig(:,1));

%added this so velocity estimates could be compared to truth (which is in
%body frame)
rotMat = quat2rotmat(SPKF_full.X_hat.q_hat); 
SPKF_full.X_hat.vel_hat_body = rotMat*SPKF_full.X_hat.vel_hat;


upd =  K_k*V_k*K_k';
P_hat = P_k_m - upd;


SPKF_full.P_hat = 0.5*(P_hat + P_hat');




