function [signal_c,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,u1,u2,u3,u4,eyaw2] = ControllerZhang2(x,i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,eyaw_prev,er_prev,omega_prev,accel,eyaw2_prev)

global m g Kt Kr prop_loc Kp Kq Jr Dt Ixx Iyy Izz


% prop_accel_sat = 10000000000; %(20000*2*pi/60); %in rad/s^2

% Add noise to state estimates
% x(1) = x(1) + (0.01).*randn(1);
% x(2) = x(2) + (0.01).*randn(1);
% x(3) = x(3) + (0.01).*randn(1);
% x(4) = x(4) + (1*pi/180).*randn(1);
% x(5) = x(5) + (1*pi/180).*randn(1);
% x(6) = x(6) + (1*pi/180).*randn(1);
% x(7) = x(7) + (0.01).*randn(1);
% x(8) = x(8) + (0.01).*randn(1);
% x(9) = x(9) + (0.01).*randn(1);

% x(10) = x(10) + (1*pi/180).*randn(1);
% x(11) = x(11) + (1*pi/180).*randn(1);
% x(12) = x(12) + (1*pi/180).*randn(1);
% x(13) = x(13) + (1*pi/180).*randn(1);

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

[roll, pitch, yaw] = quat2angle(q,'xyz');


%% Tait-Bryan Euler Angle rates
roll_w = x(4) + tan(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));
pitch_w = x(5)*cos(roll) - x(6)*sin(roll);
yaw_w = sec(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));

%% Zhang 2014
%% Altitude Controller Parameters
Kpz = 2;%20; %Zhang x4 value = 1
Kiz = 0;%40;
Kdz = 0.4;

Kpvz = 1.6;%10; %Zhang x4 value = 2.8
Kivz = 60;%10; %Zhang x4 value = 4

sat_v_des = 3; %Zhang x4 value = 0.6

%% Horizontal Position Controller Parameters
Kps = 1;%0.6; %Zhang x4 value = 0.6
Kpvx = 2; %Zhang x4 value = 2
Kpvy = 2; %Zhang x4 value = 2

Kpyaw = 1.2; %Zhang x4 value = 0.7
Kiyaw = 1;
Kdyaw = 0.2;

sat_vpos_des = 2.5; %Zhang x4 value = 1
sat_roll_des = 1;%0.2 %Zhang x4 value = 0.1
sat_pitch_des = 1;%0.2; %Zhang x4 value = 0.1
sat_r_des = 0.3; %Zhang x4 value = 0.3

%% Attitude Controller Parameters
Kprp = 7.2; %Zhang x4 value = 7.2
Kirp = 4; %Zhang x4 value = 4
Kdrp = 4.2; %Zhang x4 value = 4.2

Kpvyaw = 1.8; %Zhang x4 value = 2.8
Kivyaw = 2; %Zhang x4 value = 4;
Kdvyaw = 0; %Zhang x4 value = 0;

%% PID Altitude Controller
ez = ref_r(3) - x(9);
if i == t0
    i_z = 0;
    d_z = 0;    
else
    i_z = ez + ez_prev;
    d_z = ez - ez_prev;
end

v_des = Kpz*ez + Kiz*dt*(i_z)*0.5 + Kdz*(d_z)/dt;

% if v_des < 0
%     v_des = max([-sat_v_des,v_des]);
% else
%     v_des = min([v_des,sat_v_des]);
% end

evz = v_des - R(:,3)'*x(1:3)';
if i == t0
    evz_prev = -evz;
end

az = Kpvz*evz + Kivz*dt*(evz_prev+evz)*0.5;
az = R(3,3)*(az+g);

%% PI Horizontal Position Controller

% Roll & Pitch
ex = ref_r(1) - x(7);
ey = ref_r(2) - x(8);
es = sqrt(ex^2 + ey^2);    
if i == t0
    es_prev = -es;
end

vpos_des = Kps*es;
% if vpos_des < 0
%     vpos_des = max([-sat_vpos_des,vpos_des]);
% else
%     vpos_des = min([vpos_des,sat_vpos_des]);
% end

vx_des = vpos_des*cos(atan2(ey,ex));
vy_des = vpos_des*sin(atan2(ey,ex));

evx = vx_des - R(:,1)'*x(1:3)';
evy = vy_des - R(:,2)'*x(1:3)';

if i == t0
    evx_prev = -evx;
    evy_prev = -evy;
end

ax = Kpvx*evx;
ay = Kpvy*evy;
a = [ax;ay;0];

ax = R(1,:)*a;
ay = R(2,:)*a;
az2 = R(3,:)*a;
u1 = m*(az+az2);

% pitch_des = -(cos(yaw)*ax + sin(yaw)*ay)/g;
% roll_des = -(sin(yaw)*ax - cos(yaw)*ay)/g;

roll_des = ay/g;
pitch_des = -ax/g;

% if pitch_des < 0
%     pitch_des = max([-sat_pitch_des,pitch_des]);
% else
%     pitch_des = min([pitch_des,sat_pitch_des]);
% end
% 
% if roll_des < 0
%     roll_des = max([-sat_roll_des,roll_des]);
% else
%     roll_des = min([roll_des,sat_roll_des]);
% end


% Yaw

eyaw = ref_head - yaw;

if eyaw > pi
    eyaw = -(2*pi - eyaw);
end

% 
% if abs(eyaw) > pi
%     Kpyaw = -Kpyaw;
% %     Kiyaw = -Kiyaw;
% %     Kdyaw = -Kdyaw;
% end

if i == t0
    i_yaw = 0;
    d_yaw = 0;
else
    i_yaw = eyaw + eyaw_prev;
    d_yaw = eyaw - eyaw_prev;
end

r_des = Kpyaw*eyaw + Kiyaw*dt*(i_yaw) + Kdyaw*(d_yaw)/dt;
% if r_des < 0
%     r_des = max([-sat_r_des,r_des]);
% else
%     r_des = min([r_des,sat_r_des]);
% end

%% Quaternion based attitude control

Axis_err_prev = [eroll_prev;epitch_prev;eyaw2_prev];

q_des = angle2quat(-(roll_des+pi),pitch_des,ref_head,'xyz')';
q_err_w = quatmultiply(q_des,quatinv(q));

q_err_w = sign(q_err_w(1))*q_err_w;
alpha = acos(q_err_w(1))*2;

if alpha == 0
    q_err_b = q_err_w;
else
    r_b = quatmultiply(quatinv(q),quatmultiply(([0;q_err_w(2:4)]./sin(alpha/2)),q));
    q_err_b = [cos(alpha/2);sin(alpha/2).*r_b(2:4)];
end

Axis_err = q_err_b(2:4);

if i == t0
    i_axis = 0;
    d_axis = 0;    
else
    i_axis = Axis_err + Axis_err_prev;
    d_axis = Axis_err - Axis_err_prev;
end

% v_des = Kpz*ez + Kiz*dt*(i_z)*0.5 + Kdz*(d_z)/dt;

torque =  [-40;40;4.3].*Axis_err + [-22.2;22.2;8]*dt.*i_axis*0.5 + [-23.3;23.3;0].*d_axis/dt; % + [10;10;5]*dt.*i_axis*0.5; 
% 0.007406

Pw = 1;

% torque(3) = -torque(3);

% u2 = torque(1) - Pw*x(4);
% u3 = torque(2) - Pw*x(5);
% u4 = torque(3) - Pw*x(6);

u2 = (torque(1) - Pw*x(5)*x(6)*(Iyy-Izz)/Ixx)*Ixx;
u3 = (torque(2) - Pw*x(4)*x(6)*(Izz-Ixx)/Iyy)*Iyy;
u4 = (torque(3) - Pw*x(4)*x(5)*(Ixx-Iyy)/Izz)*Izz;
% u4 = 0;

eroll = Axis_err(1);
epitch = Axis_err(2);
eyaw2 = Axis_err(3);
er = 0;

% eroll = alpha_h*cos(beta_h);
% epitch = alpha_h*sin(beta_h);
% eyaw2 = psi_h;
% er = 0;


%% For Gareth testing Attitude Controller only:
% roll_des = 0.1;
% pitch_des = 0.1;
% r_des = 0.1;

% 
% %% Attitude Controller
% %inputs: roll_des, pitch_des, r_des
% %output: prop_speed, prop_accel
% 
% eroll = roll_des - roll;
% epitch = pitch_des - pitch;
% er = r_des - x(6);
% 
% if i == t0
%     i_roll = 0;
%     i_pitch = 0;
%     i_r = 0;
%     
%     d_roll = 0;
%     d_pitch = 0;
%     d_r = 0;
% else
%     i_roll = eroll + eroll_prev;
%     i_pitch = epitch + epitch_prev;
%     i_r = er + er_prev;
%     
%     d_roll = eroll - eroll_prev;
%     d_pitch = epitch - epitch_prev;
%     d_r = er - er_prev;
% end
% 
% v_roll = Kprp*eroll + Kirp*dt*(i_roll)*0.5 + Kdrp*(d_roll)/dt;
% v_pitch = Kprp*epitch + Kirp*dt*(i_pitch)*0.5 + Kdrp*(d_pitch)/dt;
% v_r =  Kpvyaw*er + Kivyaw*dt*(i_r)*0.5 + Kdvyaw*(d_r)/dt;
% 
% u2 = (v_roll - x(5)*x(6)*(Iyy-Izz)/Ixx)*Ixx;
% u3 = (v_pitch - x(4)*x(6)*(Izz-Ixx)/Iyy)*Iyy;
% u4 = (v_r - x(4)*x(5)*(Ixx-Iyy)/Izz)*Izz;

%Thrust and Moment Control Signal
signal_c = [u1;u2;u3;u4];

%Propeller Speed and Acceleration Control Signal
A = [-Kt -Kt -Kt -Kt;...
    -Kt*prop_loc(2,1) -Kt*prop_loc(2,2) -Kt*prop_loc(2,3) -Kt*prop_loc(2,4);...
    Kt*prop_loc(1,1) Kt*prop_loc(1,2) Kt*prop_loc(1,3) Kt*prop_loc(1,4);...
    -Dt Dt -Dt Dt];

temp = inv(A)*signal_c;
omegasquare = temp.*(temp>0);
omega = sqrt(omegasquare);
omega = [-omega(1);omega(2);-omega(3);omega(4)]; %in RPM

omega_rad = omega * (2*pi/60);
omega_prev_rad = omega_prev * (2*pi/60);

omegadot = (omega_rad - omega_prev_rad)/dt; %in rad/s^2

% % Saturate propeller acceleration
% 
% prop_accel = zeros(4,1);
% prop_accel(1) = min(abs(omegadot(1)),prop_accel_sat)*sign(omegadot(1));
% prop_accel(2) = min(abs(omegadot(2)),prop_accel_sat)*sign(omegadot(2));
% prop_accel(3) = min(abs(omegadot(3)),prop_accel_sat)*sign(omegadot(3));
% prop_accel(4) = min(abs(omegadot(4)),prop_accel_sat)*sign(omegadot(4));
% 
% % Recalculate prop speed based on saturated propeller acceleration
% omegadot = prop_accel;
% omega_rad = omegadot*dt + omega_prev_rad;
% omega = omega_rad * (60/(2*pi));

signal_c = [omega;omegadot];


end

