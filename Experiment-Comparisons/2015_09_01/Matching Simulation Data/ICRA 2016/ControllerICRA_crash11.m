function [control,ez,evz,eroll,epitch,eyaw,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,u1,u2,u3,u4] = ControllerICRA(x,i,t0,dt,ref_z,ref_head,ref_att,ez_prev,evz_prev,eroll_prev,epitch_prev,eyaw_prev,er_prev,omega_prev,recover,off_control,Tc_act)

global m g Kt Kr prop_loc Kp Kq Jr Dt Ixx Iyy Izz

% recover = 0;
roll_des = ref_att(1);
pitch_des = ref_att(2);

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

% sat_v_des = 3; %Zhang x4 value = 0.6

%% Heading Controller Parameters
Kpyaw = 1.2; %1.2
Kiyaw = 1; %1
Kdyaw = 0.2; %0.2

%% Attitude Controller Parameters
Kprp = 60; %Zhang x4 value = 7.2
Kirp = 0; %Zhang x4 value = 4
Kdrp = 6; %Zhang x4 value = 4.2

Kpvyaw = 1.8; %1.8 %Zhang x4 value = 2.8
Kivyaw = 0.2; %2 %Zhang x4 value = 4;
Kdvyaw = 0; %Zhang x4 value = 0;

if off_control == 0
    %% PID Altitude Controller
    ez = ref_z - x(9);
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
    u1 = m*R(3,3)*(az+g);

    %% PID Heading Controller
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
    if recover == 1
        u1 = -m*10.5;
        roll_des = deg2rad(-0.5);
        pitch_des = deg2rad(12);
        r_des = 0;
    end
    
else %Recovery Controller
    u1 = -m*g; %m*R(3,3)*g
    roll_des = 0;
    pitch_des = 0;
    r_des = 0;
    
    ez = 0;
    evz = 0;
    eyaw = 0;
    
end


%% For Gareth testing Attitude Controller only:
% roll_des = 0;
% pitch_des = -0.2;
% r_des = 0;

% 
%% Attitude Controller
%inputs: roll_des, pitch_des, r_des
%output: prop_speed, prop_accel

eroll = roll_des - roll;
epitch = pitch_des - pitch;
er = r_des - x(6);

if i == t0
    i_roll = 0;
    i_pitch = 0;
    i_r = 0;

    d_roll = 0;
    d_pitch = 0;
    d_r = 0;
else
    i_roll = eroll + eroll_prev;
    i_pitch = epitch + epitch_prev;
    i_r = er + er_prev;

    d_roll = eroll - eroll_prev;
    d_pitch = epitch - epitch_prev;
    d_r = er - er_prev;
end

v_roll = Kprp*eroll + Kirp*dt*(i_roll)*0.5 + Kdrp*(d_roll)/dt;
v_pitch = Kprp*epitch + Kirp*dt*(i_pitch)*0.5 + Kdrp*(d_pitch)/dt;
v_r =  Kpvyaw*er + Kivyaw*dt*(i_r)*0.5 + Kdvyaw*(d_r)/dt;

u2 = (v_roll - x(5)*x(6)*(Iyy-Izz)/Ixx)*Ixx;
u3 = (v_pitch - x(4)*x(6)*(Izz-Ixx)/Iyy)*Iyy;
u4 = (v_r - x(4)*x(5)*(Ixx-Iyy)/Izz)*Izz;

%% Generate Control Signal
%Thrust and Moment Control Signal
control = [u1;u2;u3;u4];

%Propeller Speed and Acceleration Control Signal
A = [-Kt -Kt -Kt -Kt;...
    -Kt*prop_loc(2,1) -Kt*prop_loc(2,2) -Kt*prop_loc(2,3) -Kt*prop_loc(2,4);...
    Kt*prop_loc(1,1) Kt*prop_loc(1,2) Kt*prop_loc(1,3) Kt*prop_loc(1,4);...
    -Dt Dt -Dt Dt];

temp = inv(A)*control;
omegasquare = temp.*(temp>0);
omega = sqrt(omegasquare);

omega = max(min(omega,6500),4000);

omega = [-omega(1);omega(2);-omega(3);omega(4)]; %in RPM
% if recover == 1
%     if i >= Tc_act + 0;
%         omega(2) = 0;
%     end
% 
%     if i >= Tc_act + 0.0984;
%         omega(3) = 0;
%     end
%     
%     if i >= Tc_act + 0.0984;
%         omega(4) = 0;
%     end
% 
%     if i >= Tc_act + 0.2084;
%         omega(1) = 0;
%     end
% end


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

control = [omega;omegadot];


end

