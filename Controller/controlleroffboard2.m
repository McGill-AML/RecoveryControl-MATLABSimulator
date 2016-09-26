function Control = controlleroffboard(state,stateDeriv,iSim,timeInit,tStep,Control)

% based on "Minimum Snap Trajectory Generation and Control for Quadrotors"
% by Mellinger & Kumar, to match px4 offboard control

global m g Ixx Iyy Izz u2RpmMat

%% Save inputs 
posnDes = Control.pose.posn;
yawDes = Control.pose.attEuler(3);

posnDerivDes = Control.twist.linVel;
worldAccDes = Control.twist.worldAcc;

angVelDes = Control.twist.angVel;

pos = state(7:9);
vel = stateDeriv(7:9);

q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
R = quat2rotmat(q);

errPosnDerivPrev = Control.errPosnDeriv;
errAngVelPrev = Control.errAngVel;

vel_sp_prev = Control.vel_sp;
vel_err_prev = Control.vel_err;
vel_prev = Control.vel;



%% Controller Parameters
% Kposn_P = diag([1.8,1.8,2]); %Positive definite gain matrix on position
% Kvel_P = diag([2,2,2.5]); %Positive definite gain matrix on linear velocity
% % Kvel_I = 50*diag([0.02,0.02,0.02]);
% Kvel_I = diag([0,0,0]);
% Kvel_D = 5*diag([0.01,0.01,0]);
% 
% Katt_P = diag([10,10,0.3]); %Positive definite gain matrix on orientation
% Kattrate_P = diag([2,1,0.2]); %Positive definite gain matrix on p,q,r
% % Kattrate_I = diag([0.1, 0.1, 0.1]);
% Kattrate_I = zeros(3,3);
% Kattrate_D = diag([0.0055, 0.0055, 0]);

Kposn_P = [1.8,1.8,2]'; %Positive definite gain matrix on position
Kvel_P = [2,2,2.5]'; %Positive definite gain matrix on linear velocity
Kvel_I = [0,0,0]';
Kvel_D = [0.01,0.01,0]';

Katt_P = [10,10,0.3]'; %Positive definite gain matrix on orientation
Kattrate_P = [2,1,0.2]'; %Positive definite gain matrix on p,q,r
Kattrate_I = zeros(3,1);
Kattrate_D = [0.0055, 0.0055, 0]';

% Max values
VEL_MAX_XY = [2,2]; %m/s
VEL_MAX_Z = [2,2,]; %[max up, max down]
ACC_MAX_XY = 2;
TILT_MAX = deg2rad(30);
TILT_COS_MAX = 0.7;
THR_MAX = 0.95;
SIGMA = 0.000001;
ANGVEL_MAX = 13;

%% Control
% Calculate errors on position and velocity
vel_sp = [(posnDes(1) - pos(1))*Kposn_P(1);... %mc_pos_control_main line 1429
          (posnDes(2) - pos(2))*Kposn_P(2);...
          posnDerivDes(3)];

ft_vel = [posnDerivDes(1);posnDerivDes(2);0];
cos_ratio = dot(ft_vel,vel_sp) / (norm(ft_vel)*norm(vel_sp));

if cos_ratio > 0 %mc_pos_control_main line 1453
    ft_vel = ft_vel*cos_ratio;
    ft_vel = ft_vel + 1.5*(ft_vel/norm(ft_vel));
else
    ft_vel = [0;0;0];
end

for iAxis = 1:2 %mc_pos_control_main line 1462
    if abs(ft_vel(iAxis)) > abs(vel_sp(iAxis))
        vel_sp(iAxis) = ft_vel(iAxis);
    else
        vel_sp(iAxis) = vel_sp(iAxis);
    end
end

vel_sp(3) = (posnDes(3) - pos(3))*Kposn_P(3);

vel_norm_xy = norm(vel_sp(1:2));
if vel_norm_xy > VEL_MAX_XY(1)
    vel_sp(1) = vel_sp(1) * VEL_MAX_XY(1) / vel_norm_xy;
    vel_sp(2) = vel_sp(2) * VEL_MAX_XY(2) / vel_norm_xy;
end

if vel_sp(3) > VEL_MAX_Z(1) %up
    vel_sp(3) = VEL_MAX_Z(1);
end

if vel_sp(3) < -VEL_MAX_Z(2) %down
    vel_sp(3) = -VEL_MAX_Z(2);
end

%mc_pos_control_main line 1562
acc_tot = (vel_sp - vel_sp_prev)/tStep;
acc_hor = acc_tot(1:2);

if norm(acc_hor) > ACC_MAX_XY
    acc_hor = acc_hor/norm(acc_hor);
    acc_hor = acc_hor * ACC_MAX_XY;
    vel_sp(1:2) = acc_hor*tStep + vel_sp_prev(1:2);
end

acc_v = acc_tot(3);
if abs(acc_v) > 2*ACC_MAX_XY
    acc_v = acc_v / abs(acc_v);
    vel_sp(3) = acc_v * 2 * ACC_MAX_XY * tStep + vel_sp_prev(3);
end

vel_err = vel_sp - vel;
%mc_pos_control_main -> need line 1639 to 1656 for attitude continuity?

vel_err_d = (vel_err - vel_err_prev)/tStep;

thrust_sp = Kvel_P.*vel_err + Kvel_D.*vel_err_d + Control.thrust_int; %mc_pos_control_main line 1665

Control.vel_z_lp = Control.vel_z_lp*(1 - tStep*8) + tStep*8*vel(3);
vel_z_change = (vel(3) - vel_prev(3))/tStep;
Control.acc_z_lp = Control.acc_z_lp*(1 - tStep*8) + tStep*8*vel_z_change;

saturation_xy = false;
saturation_z = false;

% limit max tilt, mc_pos_control_main line 1761
if TILT_MAX < pi/2 -0.05
    thrust_sp_xy_len = norm(thrust_sp(1:2));
    if thrust_sp_xy_len > 0.01
        thrust_xy_max = -thrust_sp(3)*tan(TILT_MAX);
        
        if thrust_sp_xy_len > thrust_xy_max
            k = thrust_xy_max / thrust_sp_xy_len;
            thrust_sp(1:2) = thrust_sp(1:2)*k;
            saturation_xy = true;
        end
    end
end

% thrust compensation for altitude control, mc_pos_control_main line 1779
if R(3,3) > TILT_COS_MAX
    att_comp = 1/R(3,3);
elseif R(3,3) > 0
        att_comp = ((1/TILT_COS_MAX - 1)/TILT_COS_MAX) * R(3,3) + 1;
else
    att_comp = 1;
end
thrust_sp(3) = thrust_sp(3) * att_comp;

% limit max thrust
thrust_abs = norm(thrust_sp);
if thrust_abs > THR_MAX
    if thrust_sp(3) < 0
        if (-thrust_sp(3) > THR_MAX)
            thrust_sp = [0;0;-THR_MAX];
            saturation_xy = true;
            saturation_z = true;
        else
            thrust_xy_max = sqrt(THR_MAX^2 - thrust_sp(3)^2);
            thrust_xy_abs = norm(thrust_sp(1:2));
            k = thrust_xy_max / thrust_xy_abs;
            thrust_sp(1:2) = thrust_sp(1:2)*k;
            saturation_xy = true;
        end
    else
        k = THR_MAX / thrust_abs;
        thrust = thrust_sp*k;
        saturation_xy = true;
        saturation_z = true;
    end
    thrust_abs = THR_MAX;
end


if saturation_xy == false
    Control.thrust_int(1:2) = Control.thrust_int(1:2) + vel_err(1:2).*Kvel_I(1:2) * tStep;
end

if saturation_z == false
    Control.thrust_int(3) = Control.thrust_int(3) + vel_err(3)*Kvel_I(3) * tStep;
end

% Calculate attitude setpoint from thrust vector, mc_pos_control_main line 1848
if thrust_abs > SIGMA
    body_z = -thrust_sp / thrust_abs;
else
    body_z = [0;0;1];    
end

y_C = [-sin(yawDes); cos(yawDes); 0];

if abs(body_z) > SIGMA
    body_x = cross(y_C,body_z);
    
    if body_z(3) < 0
        body_x = - body_x;
    end
    
    body_x = body_x / norm(body_x);
else
    body_x = [0;0;1];
end

body_y = cross(body_z,body_x);

%******************% R transpose?, mc_pos_control_main line 1888
R_sp = [body_x, body_y, body_z];
q_sp = rotmat2quat(R_sp);
att_sp.thrust = thrust_abs;

R_sp_z = body_z;

eR_matrix = (R_sp'*R' - R*R_sp);
eR = 0.5*[eR_matrix(3,2) - eR_matrix(2,3); ...
          eR_matrix(1,3) - eR_matrix(3,1); ...
          eR_matrix(2,1) - eR_matrix(1,2)];
      
eR_z_sin = norm(eR);
eR_z_cos = dot(R(3,:)',R_sp(:,3));
yaw_w = R_sp(3,3)^2;

if (eR_z_sin > 0)
    eR_z_angle = atan2(eR_z_sin,eR_z_cos);
    eR_z_axis = eR / eR_z_sin;
    eR = eR_z_axis * eR_z_angle;
    eR_cp = [0, -eR_z_axis(3), eR_z_axis(2); ...
             eR_z_axis(3), 0, -eR_z_axis(1); ...
             -eR_z_axis(2), eR_z_axis(1), 0];
    R_rp = R'*(eye(3) + eR_cp*eR_z_sin + eR_cp*eR_cp*(1-eR_z_cos));
else
    R_rp = R';
end
R_sp_x = body_x;
R_rp_x = R_rp(:,1);
eR(3) = atan2(dot(cross(R_rp_x,R_sp_x),R_sp_z),dot(R_rp_x,R_sp_x))*yaw_w;

rates_sp = Katt_P.*eR;
rates_sp_locs = find(abs(rates_sp)>ANGVEL_MAX);
if numel(rates_sp_locs) > 0
    rates_sp(rates_sp_locs) = sign(rates_sp(rates_sp_locs))*ANGVEL_MAX;
end


% errPosn = state(7:9) - posnDes;
% errPosnDeriv = stateDeriv(7:9) - posnDerivDes;
% 
% if iSim == timeInit
%     errPosnDerivIntegral = [0;0;0];
%     errPosnDerivDeriv = [0;0;0];    
% else
%     errPosnDerivIntegral = errPosnDeriv + errPosnDerivPrev;
%     errPosnDerivDeriv = (errPosnDeriv - errPosnDerivPrev)/tStep;
% end
% 
% Control.integralErrPosnDeriv = Control.integralErrPosnDeriv + errPosnDerivIntegral*tStep*0.5;
% 
% % Compute desired force vector
% zW = [0;0;1]; %unit vector pointing in world upwards direction
% % Fdes = -(-Kposn_P*errPosn - Kvel_P*errPosnDeriv + m*g*zW + m*worldAccDes);
% Fdes = (-Kposn_P*errPosn - Kvel_P*errPosnDeriv -Kvel_I*Control.integralErrPosnDeriv...
%          -Kvel_D*errPosnDerivDeriv + m*g*zW + m*worldAccDes);
% zB = R'*[0;0;-1]; %unit vector pointing in upwards direction on body
% u1 = -dot(Fdes,zB);
% 
% % Find desired body axes
% zB_des = Fdes/norm(Fdes);
% xC_des = [cos(yawDes); sin(yawDes); 0];
% yB_des = cross(zB_des,xC_des)/norm(cross(zB_des,xC_des));
% xB_des = cross(yB_des,zB_des);
% R_des = [xB_des, -yB_des, -zB_des];
% R_des2 = invar2rotmat('x',pi)*[xB_des, yB_des, zB_des];
% 
% % Define error on orientation
% eR_matrix = (R_des'*R' - R*R_des);
% eR = 0.5*[eR_matrix(3,2) - eR_matrix(2,3); ...
%           eR_matrix(1,3) - eR_matrix(3,1); ...
%           eR_matrix(2,1) - eR_matrix(1,2)];
% 
% % Angular velocity error
e_omega = state(4:6) - rates_sp;

% error rates and integrals
if iSim == timeInit
    errAngVelIntegral = [0;0;0];
    errAngVelDeriv = [0;0;0];    
else
    errAngVelIntegral = e_omega + errAngVelPrev;
    errAngVelDeriv = (e_omega - errAngVelPrev)/tStep;
end

Control.integralErrAngVel =  Control.integralErrAngVel + errAngVelIntegral*tStep*0.5;

moments = -Katt_P.*eR - Kattrate_P.*e_omega - Kattrate_I.*Control.integralErrAngVel - Kattrate_D.*errAngVelDeriv;

u1 = -9.81;
u2 = moments(1);
u3 = moments(2);
u4 = moments(3);

%% Generate Control Signal
%Thrust and Moment Control Signal
u = [u1;u2;u3;u4];

%Propeller RPM Control Signal
temp = u2RpmMat*u;
rpmsquare = temp.*(temp>0);
rpm = sqrt(rpmsquare);
rpm = max(min(sqrt(rpmsquare),7800),3600); %saturate motor speeds
rpm = [-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM

%% Assign values to output Control
Control.rpm = rpm;
Control.u = [u1;u2;u3;u4];
Control.errEuler = eR;
Control.errAngVel = e_omega;
% Control.errPosnDeriv = errPosnDeriv;

Control.vel_sp = vel_sp;
Control.vel_err = vel_err;
Control.vel = vel;

Control.vel_z_lp = Control.vel_z_lp;
Control.acc_z_lp = Control.acc_z_lp;
Control.thrust_int = Control.thrust_int;
end

