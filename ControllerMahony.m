function [signal_c,evz,es,evx,evy,eyaw] = ControllerMahony(x,i,t0,dt,evz_prev,es_prev,evx_prev,evy_prev,eyaw_prev,ref_r,ref_head)

global m g

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
T = quatRotMat(q);

%% Zhang 2014
% Altitude Controller Gains
Kpz = 10;
Kpvz = 10;
Kivz = 10;

%Horizontal Position Controller Gains
Kps = 10;
Kis = 10;
Kpvx = 20;
Kivx = 15;
Kpvy = 20;
Kivy = 15;
Kpr = 5;
Kir = 5;

Kr = 0.1;
Kw = 0.1;

Pxy = 30;
Pz = 30;
Dxy = 6;
Dz = 6;
Ppos = diag([Pxy,Pxy,Pz]);
Dpos = diag([Dxy,Dxy,Dz]);
Prp = 10;
Pyaw = 20;
Ppq = 50;
Pr = 50;
Patt = diag([Ppq,Ppq,Pr]);

% PID Altitude Controller
ez = ref_r(3) - x(9);
v_des = Kpz*ez;

evz = v_des - T(:,3)'*x(1:3)';
if i == t0
    evz_prev = -evz;
end
az = Kpvz*evz + Kivz*dt*(evz_prev+evz)*0.5;
u1 = m*(az + g);

%PI Horizontal Position Controller
ex = ref_r(1) - x(7);
ey = ref_r(2) - x(8);
es = sqrt(ex^2 + ey^2);    
if i == t0
    es_prev = -es;
end
v_des = Kps*es + Kis*dt*(es_prev+es)*0.5;
vx_des = (ex/es)*v_des;
vy_des = (ey/es)*v_des;

evx = vx_des - T(:,1)'*x(1:3)';
evy = vy_des - T(:,2)'*x(1:3)';
eyaw = ref_head - x(6);
if i == t0
    evx_prev = -evx;
    evy_prev = -evy;
    eyaw_prev = -eyaw;
end
ax = Kpvx*evx + Kivx*dt*(evx_prev+evx)*0.5;
ay = Kpvy*evy + Kivy*dt*(evy_prev+evy)*0.5;

a_des = Ppos*(ref_r-x(7:9)')+Dpos*([v_des;vx_des;vy_des]-(T'*x(1:3)'))+(g);
ax = a_des(1);
ay = a_des(2);


roll_des = (ax*sin(ref_head)-ay*cos(ref_head))/g;
pitch_des = (ax*cos(ref_head)+ay*sin(ref_head))/g;
yaw_des = ref_head;

roll = atan2(2*q(1)*q(2)+q(3)*q(4),1-2*(q(2)^2+q(3)^2));
pitch = asin(2*(q(1)*q(3)-q(4)*q(2)));
yaw = atan2(2*q(1)*q(4)+q(2)*q(3),1-2*(q(3)^2+q(4)^2));

roll_w = x(4) + tan(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));
pitch_w = x(5)*cos(roll) - x(6)*sin(roll);
yaw_w = sec(pitch)*(x(5)*sin(roll)+x(6)*cos(roll));

er = [roll_des;pitch_des;yaw_des]-[roll;pitch;yaw];

ew = [0;0;0]-[roll_w;pitch_w;yaw_w];

u2 = -Kr*er - Kw*ew;


%theta
pitch_c = (cos(eyaw)*ax + sin(eyaw)*ay)/g;
%phi
roll_c = (sin(eyaw)*ax - cos(eyaw)*ay)/g;
%psi
yaw_c = Kpr*eyaw + Kir*dt*(eyaw_prev+eyaw)*0.5;

q0_c = cos(roll_c/2)*cos(pitch_c/2)*cos(yaw_c/2) + sin(roll_c/2)*sin(pitch_c/2)*sin(yaw_c/2);
q1_c = sin(roll_c/2)*cos(pitch_c/2)*cos(yaw_c/2) - cos(roll_c/2)*sin(pitch_c/2)*sin(yaw_c/2);
q2_c = cos(roll_c/2)*sin(pitch_c/2)*cos(yaw_c/2) + sin(roll_c/2)*cos(pitch_c/2)*sin(yaw_c/2);
q3_c = cos(roll_c/2)*cos(pitch_c/2)*sin(yaw_c/2) - sin(roll_c/2)*sin(pitch_c/2)*cos(yaw_c/2);

signal_c = [u1;u2];

end

