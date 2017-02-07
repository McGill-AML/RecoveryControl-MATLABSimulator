function [ASPKF_opt] = ASPKF_opt_attitude(Sensor, ASPKF_opt, AEKF, sensParams, tStep)
% ASPKF_opt - kinematic prediction for orientation
% this estimator uses an MRP formulation of the quaternion in a sigma point
% kalman filter to propagate the state estimate. Only estimates gryo bias
% and quaternion - angular velocity is assumed to be gyro value minus bias
% Adaptive modification upon crashing modifies the covariance matrices.

%notation - _k_1 is previous timestep, _k_m is predicted, _k is corrected
%estimate
global g mag

%unpackage
%states
pos_k_1 = AEKF.X_hat.pos_hat;

vel_k_1 = AEKF.X_hat.vel_hat;

q_k_1 = ASPKF_opt.X_hat.q_hat;

omega_k_1 = ASPKF_opt.X_hat.omega_hat;

rotMat_k_1 = quat2rotmat(q_k_1);

%bias terms
% bias_acc = AEKF.X_hat.bias_acc;
bias_gyr = ASPKF_opt.X_hat.bias_gyr;
bias_acc = [0;0;0]; %sensParams.bias.acc; % for the sake of comparing attitude estimators assume no accelerometer bias.


%measurements
u_b_acc = Sensor.acc;
u_b_gyr = Sensor.gyro;
u_b_mag = Sensor.mag;

u_b_gps = Sensor.gps;

u_b_baro = Sensor.baro;

MRP_0 = zeros(3,1); %q_k_1(2:4)/(1+q_k_1(1));

%%
%init covariance for sig pt calcs
%sig for prediction (process)
Q_k_1 = tStep*diag([sensParams.var_gyr;  
              sensParams.var_bias_gyr]); 

       
%adaptive terms applied to noise matrices


Q_k_1_G = ASPKF_opt.Lam_k + Q_k_1;


P_k_1_att =  ASPKF_opt.P_hat;

%measurement noise adaptation
ASPKF_opt.use_acc(2:end) = ASPKF_opt.use_acc(1:end-1); % shift accelerometer use vector

if norm(u_b_acc,2) > norm(g,2) + ASPKF_opt.accel_bound || norm(u_b_acc,2) < norm(g,2) - ASPKF_opt.accel_bound
    %sig for correct (measurements)        
    R_k = diag([sensParams.var_mag]);
    
    R_k_G = ASPKF_opt.S_k(4:6,4:6) + R_k;
    
    ASPKF_opt.use_acc(1) = 0;
else
    R_k = diag([sensParams.var_acc;
                sensParams.var_mag]);
    
    R_k_G = ASPKF_opt.S_k + R_k;
    
    ASPKF_opt.use_acc(1) = 1;
end





% construct matrix to find sigma points
Y = blkdiag(P_k_1_att,Q_k_1_G,R_k_G); 


S = chol(Y, 'lower');

L = length(S);

Sigma_pts_k_1 = zeros(L,2*L+1);

% sigma points are generated for MRP error, gyro bias and noise terms
if ASPKF_opt.use_acc(1) == 1
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
lambda = ASPKF_opt.alpha^2*(L-ASPKF_opt.kappa)-L;

for ii = 1:L
     
    %make sigma points
    Sigma_pts_k_1(:,ii+1) = Sigma_pts_k_1(:,1) + sqrt(L+lambda)*S(:,ii);
    Sigma_pts_k_1(:,L+ii+1) = Sigma_pts_k_1(:,1) - sqrt(L+lambda)*S(:,ii);
    
    %convert MRP sigma points into quaternions by crassidis' paper chose a = f = 1
    eta_k_d_pos = (1-Sigma_pts_k_1(1:3,ii+1)'*Sigma_pts_k_1(1:3,ii+1))/(1+Sigma_pts_k_1(1:3,ii+1)'*Sigma_pts_k_1(1:3,ii+1));
    eta_k_d_neg = (1-Sigma_pts_k_1(1:3,L+ii+1)'*Sigma_pts_k_1(1:3,L+ii+1))/(1+Sigma_pts_k_1(1:3,L+ii+1)'*Sigma_pts_k_1(1:3,L+ii+1));
    
    q_k_1_err(:,ii+1) = [eta_k_d_pos; (1+eta_k_d_pos)*Sigma_pts_k_1(1:3,ii+1)]; %positive error quaternion
    q_k_1_err(:,L+ii+1) = [eta_k_d_neg; (1+eta_k_d_neg)*Sigma_pts_k_1(1:3,L+ii+1)]; %neg error quaternion
    
    %compute quaternion sigma pts as deviations from current
    q_k_1_sig(:,ii+1) = quatmultiply(q_k_1_err(:,ii+1),q_k_1_sig(:,1)); 
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




%find state and covariance predictions
X_k_m = 1/(L+lambda)*(lambda*Sigma_pts_k_m(1:6,1)+0.5*sum(Sigma_pts_k_m(1:6,2:2*L+1),2));

P_k_m = (lambda/(L+lambda)+(1-ASPKF_opt.alpha^2+ASPKF_opt.beta))*((Sigma_pts_k_m(1:6,1)-X_k_m(1:6))*(Sigma_pts_k_m(1:6,1)-X_k_m(1:6))');
for ii = 2:2*L+1
    P_k_m = P_k_m+1/(L+lambda)*(0.5*(Sigma_pts_k_m(1:6,ii)-X_k_m(1:6))*(Sigma_pts_k_m(1:6,ii)-X_k_m(1:6))');
end 

%%
%measurement sigma and correct


% bound so if there's large acceleration (besides gravity) dont use
if ASPKF_opt.use_acc(1) == 0
    
%     
    Sigma_Y = zeros(3,2*L+1);
    
    for ii = 1:2*L+1
        rotMat = quat2rotmat(q_k_m_sig(:,ii));       
        Sigma_Y(1:3,ii) = rotMat*(mag) + Sigma_pts_k_m(13:15,ii); %magnetometer        
    end
    
    y_k = u_b_mag;
    
    
else
    
    Sigma_Y = zeros(6,2*L+1);
    
    for ii = 1:2*L+1
        
        rotMat = quat2rotmat(q_k_m_sig(:,ii));
        
        Sigma_Y(1:3,ii) = rotMat*([0;0;g]) + bias_acc + Sigma_pts_k_m(13:15,ii); %accel
        Sigma_Y(4:6,ii) = rotMat*(mag) + Sigma_pts_k_m(16:18,ii); %magnetometer
        
    end
    y_k = [u_b_acc; u_b_mag];
    
end

y_k_hat = 1/(L+lambda)*(lambda*Sigma_Y(:,1)+0.5*sum(Sigma_Y(:,2:2*L+1),2));

%now find matrices needed for kalman gain

V_k = (lambda/(L+lambda)+(1-ASPKF_opt.alpha^2+ASPKF_opt.beta))*(Sigma_Y(:,1)-y_k_hat)*(Sigma_Y(:,1)-y_k_hat)';
for ii = 2:2*L+1
    V_k = V_k+.5/(L+lambda)*(Sigma_Y(:,ii)-y_k_hat)*(Sigma_Y(:,ii)-y_k_hat)';
end
% V_k = V_k + R_k;

U_k = (lambda/(L+lambda)+(1-ASPKF_opt.alpha^2+ASPKF_opt.beta))*(Sigma_pts_k_m(1:6,1)-X_k_m(1:6))*(Sigma_Y(:,1)-y_k_hat)';

for ii =2:2*L+1
    U_k = U_k + .5/(L+lambda)*(Sigma_pts_k_m(1:6,ii)-X_k_m(1:6))*(Sigma_Y(:,ii)-y_k_hat)';
end

%kalman gain and update
K_k = U_k/V_k;

innov = y_k - y_k_hat;

DX_k = K_k*innov;


%adapt R_k gain ( adaptation based on Hajiyev 2013 Robust adaptive kalman
% filt.) ish
% update innovation error matrix
% ASPKF_opt.innov_error(:,:,2:end) = ASPKF_opt.innov_error(:,:,1:end-1);
% 
% 
% if  ASPKF_opt.use_acc(1) == 1
% ASPKF_opt.innov_error(:,:,1) = innov*innov'; % use full innovation matrix
% 
% else
% ASPKF_opt.innov_error(:,:,1) = zeros(6); % only have magnetometer error in innovation matrix
% ASPKF_opt.innov_error(4:6,4:6,1) = innov*innov'; 
% 
% end
% 
% %update each block of adaptive gain individually as there are not as many
% %acceleromter measurements used as there are magnetometer
% if ASPKF_opt.use_acc(1) == 0
%     
%     innov_sum = sum(ASPKF_opt.innov_error(4:6,4:6),3)/ASPKF_opt.moving_win ;
%     ASPKF_opt.S_k(4:6,4:6) = innov_sum - V_k;
%     
% else
%     innov_sum(1:3,1:3) = sum(ASPKF_opt.innov_error(1:3,1:3),3)/sum(ASPKF_opt.use_acc);
%     innov_sum(4:6,1:3) = sum(ASPKF_opt.innov_error(4:6,1:3),3)/sum(ASPKF_opt.use_acc);
%     innov_sum(1:3,4:6) = sum(ASPKF_opt.innov_error(1:3,4:6),3)/sum(ASPKF_opt.use_acc);
%     
%     innov_sum(4:6,4:6) = sum(ASPKF_opt.innov_error(4:6,4:6),3)/ASPKF_opt.moving_win;
%     ASPKF_opt.S_k = innov_sum - V_k;
% 
% end
ASPKF_opt.innov_error(:,:,2:end) = ASPKF_opt.innov_error(:,:,1:end-1);


if  ASPKF_opt.use_acc(1) == 1
    temp = innov*innov' - V_k; % use full innovation matrix
    temp(temp < 0) = 0; % only use positive terms - doesn't make sense to shrink covariance
    ASPKF_opt.innov_error(:,:,1) = temp;

else
    temp = zeros(6); % only have magnetometer error in innovation matrix
    temp(4:6,4:6) = innov*innov' - V_k;
    temp(temp < 0) = 0; % only use positive terms - doesn't make sense to shrink covariance
    ASPKF_opt.innov_error(:,:,1) = temp;
end

%update each block of adaptive gain individually as there are not as many
%acceleromter measurements used as there are magnetometer
if sum(ASPKF_opt.use_acc) ~= 0
    innov_sum(1:3,1:3) = sum(ASPKF_opt.innov_error(1:3,1:3),3)/sum(ASPKF_opt.use_acc);
    innov_sum(4:6,1:3) = sum(ASPKF_opt.innov_error(4:6,1:3),3)/sum(ASPKF_opt.use_acc);
    innov_sum(1:3,4:6) = sum(ASPKF_opt.innov_error(1:3,4:6),3)/sum(ASPKF_opt.use_acc);
else
    innov_sum = zeros(6);
end

innov_sum(4:6,4:6) = sum(ASPKF_opt.innov_error(4:6,4:6),3)/ASPKF_opt.moving_win;

ASPKF_opt.S_k = innov_sum;
    
% only use diagonal terms of gain and set to 0 if less than 0
S_k_tmp = diag(ASPKF_opt.S_k);
for ii = 1:length(S_k_tmp)
    if S_k_tmp(ii)< 0
        S_k_tmp(ii) = 0;
    end
end
ASPKF_opt.S_k = diag(S_k_tmp);



%adapt Q_k gain

% % attempt #1 - didn't really work. but mighta shoulda?
% temp = P_k_m\U_k;
% 
% ASPKF_opt.Lam_k = temp'\(innov_sum - R_k)/temp - P_k_m;
% 
% % only use diagonal terms of gain and set to 0 if less than 0
% Lam_k_tmp = diag(ASPKF_opt.Lam_k);
% for ii = 1:length(Lam_k_tmp)
%     if Lam_k_tmp(ii)< 0
%         Lam_k_tmp(ii) = 0;
%     end
% end
% ASPKF_opt.Lam_k = diag(Lam_k_tmp);



% update 
ASPKF_opt.X_hat.bias_gyr = X_k_m(4:6) + DX_k(4:6);

ASPKF_opt.X_hat.omega_hat = u_b_gyr - ASPKF_opt.X_hat.bias_gyr; %ang vel is just gyro minus bias

%need to convert MRP to quaternions

q_k_upd(1) = (1-DX_k(1:3)'*DX_k(1:3))/(1+DX_k(1:3)'*DX_k(1:3));

q_k_upd(2:4,1) = (1+q_k_upd(1))*DX_k(1:3);

ASPKF_opt.X_hat.q_hat = quatmultiply(q_k_upd,q_k_m_sig(:,1));


% % attempt 2 to update Q - following hajiyev's exactly using linearized
% matrices about updated states

% update covariance
upd =  K_k*V_k*K_k';
P_hat = P_k_m - upd;


ASPKF_opt.P_hat = 0.5*(P_hat + P_hat');


