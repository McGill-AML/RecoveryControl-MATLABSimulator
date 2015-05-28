function [signal_c,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega] = ControllerZhang(x,i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,er_prev,omega_prev)

global m g Kt Kr prop_loc Kp Kq Jr Dt Ixx Iyy Izz
q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
T = quatRotMat(q);

% roll = atan2(q(1)*q(2)+q(3)*q(4),0.5-(q(2)^2+q(3)^2));
% % disp('error_roll:');
% % disp(x(14)-roll);
% pitch = asin(-2*(-q(1)*q(3)+q(4)*q(2)));
% % disp('error_pitch:');
% % disp(x(15)-pitch);
% yaw = atan2(q(1)*q(4)+q(2)*q(3),0.5-(q(3)^2+q(4)^2));
% % disp('error_yaw:');
% % disp(x(16)-yaw);
% 
% if abs(2*(q(1)*q(3)+q(4)*q(2))) == 1
%     if sign(2*(q(1)*q(3)+q(4)*q(2))) == 1
%         pitch = pi/2;
%     else
%         pitch = -pi/2;
%     end
% end
% 
roll = x(14);
pitch = x(15);
yaw = x(16);

roll_w = x(4) + tan(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));
pitch_w = x(5)*cos(roll) - x(6)*sin(roll);
yaw_w = sec(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));

%% Zhang 2014
%% Altitude Controller Gains
Kpz = 20;%10; %Zhang x4 value = 1
Kiz = 40;
Kpvz = 10;%10; %Zhang x4 value = 2.8
Kivz = 10;%10; %Zhang x4 value = 4

%% Horizontal Position Controller Gains
Kps = 0.6; %Zhang x4 value = 0.6
Kpvx = 2; %Zhang x4 value = 2
Kpvy = 2; %Zhang x4 value = 2

Kpyaw = 0.7; %Zhang x4 value = 0.7

Kprp = 7.2; %Zhang x4 value = 7.2
Kirp = 4; %Zhang x4 value = 4
Kdrp = 4.2; %Zhang x4 value = 4.2

Kpvyaw = 2.8; %Zhang x4 value = 2.8
Kivyaw = 4; %Zhang x4 value = 4;
Kdvyaw = 0; %Zhang x4 value = 0;

%% PID Altitude Controller
ez = ref_r(3) - x(9);
if i == t0
    ez_prev = -ez;
end

v_des = Kpz*ez + Kiz*dt*(ez_prev + ez)*0.5;
if v_des < 0
    v_des = max([-2.5,v_des]);
else
    v_des = min([v_des,2.5]);
end

evz = v_des - T(:,3)'*x(1:3)';
if i == t0
    evz_prev = -evz;
end
az = Kpvz*evz + Kivz*dt*(evz_prev+evz)*0.5;

u1 = m*(az + g);
% u1 = m*(g-az)/(cos(roll)*cos(pitch));

omega = sqrt(u1/(4*-Kt));
prop_speed = [omega;-omega;omega;-omega];
prop_accel = zeros(4,1);

%% PI Horizontal Position Controller

% Roll & Pitch
ex = ref_r(1) - x(7);
ey = ref_r(2) - x(8);
es = sqrt(ex^2 + ey^2);    
if i == t0
    es_prev = -es;
end

vpos_des = Kps*es;
if vpos_des < 0
    vpos_des = max([-1,vpos_des]);
else
    vpos_des = min([vpos_des,1]);
end

vx_des = vpos_des*cos(atan2(ey,ex));
vy_des = vpos_des*sin(atan2(ey,ex));

evx = vx_des - T(:,1)'*x(1:3)';
evy = vy_des - T(:,2)'*x(1:3)';

if i == t0
    evx_prev = -evx;
    evy_prev = -evy;
end
ax = Kpvx*evx;
ay = Kpvy*evy;

pitch_des = -(cos(yaw)*ax + sin(yaw)*ay)/g;
roll_des = -(sin(yaw)*ax - cos(yaw)*ay)/g;

if pitch_des < 0
    pitch_des = max([-0.1,pitch_des]);
else
    pitch_des = min([pitch_des,0.1]);
end

if roll_des < 0
    roll_des = max([-0.1,roll_des]);
else
    roll_des = min([roll_des,0.1]);
end

% Yaw
eyaw = ref_head - yaw;
r_des = Kpyaw*eyaw;
if r_des < 0
    r_des = max([-0.3,r_des]);
else
    r_des = min([r_des,0.3]);
end


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

%Thrust and Moment Control Signal
signal_c = [u1;u2;u3;u4];
% signal_c = [u1;Mx;My;Mz];

%Propeller Speed and Acceleration Control Signal
A = [-Kt -Kt -Kt -Kt;...
    -Kt*prop_loc(2,1) -Kt*prop_loc(2,2) -Kt*prop_loc(2,3) -Kt*prop_loc(2,4);...
    Kt*prop_loc(1,1) Kt*prop_loc(1,2) Kt*prop_loc(1,3) Kt*prop_loc(1,4);...
    Dt -Dt Dt -Dt];
temp = inv(A)*signal_c;
omegasquare = temp.*(temp>0);
omega = sqrt(omegasquare);
omega = [omega(1);-omega(2);omega(3);-omega(4)];
omegadot = (omega - omega_prev)/dt;
signal_c = [omega;omegadot];


end

