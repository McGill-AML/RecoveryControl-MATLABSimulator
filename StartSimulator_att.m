% function [tilt, Vc_act, defl_init, defl_max, Fn_init, Fn_max, numContacts, stable ] = StartSimulator(tilt_des, e, Vc_des, recover_control,traj_head)
clear all

global m g Kt Rbumper prop_loc
global flag_c_ct1 vi_c_ct1 flag_c_ct2 vi_c_ct2 flag_c_ct3 vi_c_ct3 flag_c_ct4 vi_c_ct4
global Tc_act prop_speed_chkpt prop_speed_chkpt_flag

% clear X

%% Spiri System Parameters
InitSpiriParams;
stable = 1;
e = 0.9; 
% 
% % ---- Crash 3 ---- %
% Vc_des = 0.6856;
% recover_control = 0;
% traj_head = -pi + deg2rad(26); %deg2rad(20);
% roll0 = deg2rad(-1.3);%deg2rad(5);
% pitch0 = deg2rad(10);%deg2rad(6.3);
% z_start = 0.2;
% z_des = 1.1; %roll0 = -6: 2.1
% ax_given = 1.7170;%1.4896; %pitch0 = 6.3: 1.4896
% tfinal = 1.2;
% 
% cmds_time = [0;0.5];
% cmd_roll = [deg2rad(2.5);0];
% cmd_pitch = [deg2rad(3.5);0];
% cmd_r = [deg2rad(-10);0];
% cmd_thrust = [-m*9.12;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash3_motorslopes.mat');
% Change Fc2 = 2*Fc2 in SpiriMotion_4Circles

% % ---- Crash 5 ---- %
% Vc_des = 1.1056;
% recover_control = 0;
% traj_head = -pi + deg2rad(4.6); %deg2rad(20);
% roll0 = deg2rad(0.6);%deg2rad(5);
% pitch0 = deg2rad(4.4);%deg2rad(6.3);
% z_start = 0.2;
% z_des = 0.51; %roll0 = -6: 2.1
% ax_given = 0.7811;%1.4896; %pitch0 = 6.3: 1.4896
% tfinal = 1.2;
% 
% cmds_time = [0;0.2];
% cmd_roll = [deg2rad(2.5);0];
% cmd_pitch = [deg2rad(3.5);0];
% cmd_r = [0;0];
% cmd_thrust = [-m*g;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash5_motorslopes.mat');

% % ---- Crash 6 ---- %
% Vc_des = 1.15;%1.17;
% recover_control = 0;
% traj_head = -pi + deg2rad(11); %deg2rad(20);
% roll0 = deg2rad(4);%deg2rad(5);
% pitch0 = deg2rad(6);%deg2rad(6.3);
% z_start = 0.7;
% z_des = 1.37; %roll0 = -6: 2.1
% ax_given = 1.2595;%1.4896; %pitch0 = 6.3: 1.4896
% tfinal = 1.2;
% 
% cmds_time = [0;0.4];
% cmd_roll = [deg2rad(3);0];
% cmd_pitch = [deg2rad(5.5);0];
% cmd_r = [deg2rad(-1);0];
% cmd_thrust = [-m*g;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash6_motorslopes.mat');

% % ---- Crash 7 ---- %
% Vc_des = 1.351;
% recover_control = 0;
% traj_head = -pi + deg2rad(8.86);
% roll0 = deg2rad(-5.1);
% pitch0 = deg2rad(6.7);
% z_start = 0.6;
% z_des = 0.88; %roll0 = -6: 2.1
% ax_given =1.0580;%1.4896; %pitch0 = 6.3: 1.4896
% tfinal = 1.2;
% 
% cmds_time = [0;0.35];
% cmd_roll = [deg2rad(-1);0];
% cmd_pitch = [deg2rad(8);0];
% cmd_r = [deg2rad(7);0];
% cmd_thrust = [-m*10.11;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash7_motorslopes.mat');

% % ---- Crash 9 ---- %
% Vc_des = 2.0283;
% recover_control = 0;
% traj_head = -pi + deg2rad(17);
% roll0 = deg2rad(-2.7);
% pitch0 = deg2rad(9.4);
% z_start = 0.6;
% z_des = 1.02; 
% ax_given = 1.5356;
% tfinal = 1.2;
% 
% cmds_time = [0;0.2];
% cmd_roll = [deg2rad(2);0];
% cmd_pitch = [deg2rad(12);0];
% cmd_r = [0;0];
% cmd_thrust = [-m*g;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash9_motorslopes.mat');

% % ---- Crash 10 ---- %
% Vc_des = 2.6799;
% recover_control = 0;
% traj_head = -pi + deg2rad(4);
% roll0 = deg2rad(-1.2);
% pitch0 = deg2rad(16);
% z_start = 0.6;
% z_des = 0.66; %roll0 = -6: 2.1
% ax_given = 2.7158;%1.4896; %pitch0 = 6.3: 1.4896
% tfinal = 1.2;
% 
% cmds_time = [0;0.65];
% cmd_roll = [deg2rad(2);0];
% cmd_pitch = [deg2rad(11);0];
% cmd_r = [deg2rad(3);0];
% cmd_thrust = [-m*g;-m*g];
% cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];
% 
% load('crash10_motorslopes.mat');

% ---- Crash 11 ---- %
e = 0.9; 
Vc_des = 1.98; %1.98
recover_control = 0;
traj_head = -pi + deg2rad(11-2.5); %10
roll0 = deg2rad(-4.4+2.2); %-6
pitch0 = deg2rad(20.2); %23.7
z_start = 0;
z_des = 0.92; %1.08
ax_given = 3.8768; %roll0 = -6: 3.995, roll0 = 6: 4.891
tfinal = 1.2;

cmds_time = [0;0.14;0.3];
cmd_roll = [deg2rad(0.6883);deg2rad(0.6883);0];
cmd_pitch = [deg2rad(12);deg2rad(12);0];
cmd_r = [deg2rad(-0.1337);0;0];
cmd_thrust = [-m*10.5;-m*10.5;-m*g];
cmds = [cmd_thrust,cmd_roll,cmd_pitch,cmd_r];

motors_time = [0;0.0616;0.0984;0.2084];
motors_slope = [[-3863.6;-20897;-2345.5;0],[0;0;0;0],...
                [-31218;-9592.4;0;0],[-1704.5;13342;0;0]];

prop_speed_chkpt = zeros(4,size(motors_time,1));
prop_speed_chkpt_flag = zeros(1,size(motors_time,1));




%% Simulation Parameters

% traj_posn = [0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0];
% traj_head = [0;pi/2;pi;-pi/2;0;-pi/2];
% 
% traj_time = [0;5;10;15;20;25];

% %Contact with max pitch and velocity of ~1 m/s
% traj_posn = [0 0 2;4 0 2];
% traj_head = [0; 0];
% traj_time = [0; 2.5];
% wall_loc = 0.5; %2.87 + 0.29; %1.5822 + 0.29; %0.29*0.95 + 0.2613;
% wall_plane = 'YZ';


traj_posn = [0 0 z_start]; %traj_posn(1) gets reassigned later
traj_time = [0;tfinal]; %traj_time(1) gets reassigned later
wall_loc = 1.5;
wall_plane = 'YZ';
Tc = 0.5;
Vc = Vc_des;

% sim_idx = 40;
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/200;

Tc_act = 10000;

traj_att = [roll0;pitch0];

q0 = angle2quat(-(roll0+pi),pitch0,traj_head,'xyz')';
R0 = quatRotMat(q0);
[roll, pitch, yaw] = quat2angle(q0,'xyz');

[traj_posn(1), vx_w0, t0 ] = FindRxVx( Tc, Vc, wall_loc, q0, traj_head, t0, ax_given);
vy_w0 = 0;
vz_w0 = 0;

v_b0 =  R0*[vx_w0;vy_w0;vz_w0];

% q0 = quatmultiply([0;-1;0;0],[cos(traj_head(1)/2);0;0;sin(traj_head(1)/2)]);
% q0 = q0/norm(q0);

omega0 = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  %Start with hovering RPM
prop_speed = omega0;
x0 = [v_b0;zeros(3,1);traj_posn(1,:)';q0];

Xtotal = x0';
ttotal = t0;

% q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)]/norm(Xtotal(end,10:13));
% R = quatRotMat(q);

% [yaw, pitch, roll] = quat2angle(q);

% Controller Initial Variables
x0_step = x0;
ez_prev = 0;
evz_prev = 0;
eroll_prev = 0;
epitch_prev = 0;
eyaw_prev = 0;
er_prev = 0;
omega_prev = omega0;

% Contact Variable Values

flag_c1 = 0;
vi_c1 = 0;
flag_c_ct1 = 0;
vi_c_ct1 = 0;

flag_c2 = 0;
vi_c2 = 0;
flag_c_ct2 = 0;
vi_c_ct2 = 0;

flag_c3 = 0;
vi_c3 = 0;
flag_c_ct3 = 0;
vi_c_ct3 = 0;

flag_c4 = 0;
vi_c4 = 0;
flag_c_ct4 = 0;
vi_c_ct4 = 0;

recover = 0;
body_accel = [0;0;0];

%% Initialize History Arrays

% States
roll_hist = roll;
pitch_hist = pitch;
yaw_hist = yaw;

rolldes_hist = traj_att(1);
pitchdes_hist = traj_att(2);
rdes_hist = 0;

% Controller
u1_hist = -m*g; %thrust required for hover
u2_hist = 0;
u3_hist = 0;
u4_hist = 0;
prop_speed_hist = prop_speed;
prop_accel_hist = [0;0;0;0];

% Contact
pint11_hist = [100;100;0];
pint12_hist = [100;100;0];
pc_w1_hist = [100;100;0];
defl1_hist = 0;
theta11_hist = 0;
theta12_hist = 0;

pint21_hist = [0;0;0];
pint22_hist = [0;0;0];
pc_w2_hist = [100;100;0];
defl2_hist = 0;
theta21_hist = 0;
theta22_hist = 0;

pint31_hist = [0;0;0];
pint32_hist = [0;0;0];
pc_w3_hist = [100;100;0];
defl3_hist = 0;
theta31_hist = 0;
theta32_hist = 0;

pint41_hist = [0;0;0];
pint42_hist = [0;0;0];
pc_w4_hist = [100;100;0];
defl4_hist = 0;
theta41_hist = 0;
theta42_hist = 0;

% Continuous Time
t_ct = [];
X_ct = [];
dX_ct = [];

defl1_ct = [];
defl_rate1_ct = [];
Fc1_ct = [];
pc1_ct = [];
flag_c1_ct = [];
vi_c1_ct = [];

defl2_ct = [];
defl_rate2_ct = [];
Fc2_ct = [];
pc2_ct = [];
flag_c2_ct = [];
vi_c2_ct = [];

defl3_ct = [];
defl_rate3_ct = [];
Fc3_ct = [];
pc3_ct = [];
flag_c3_ct = [];
vi_c3_ct = [];

defl4_ct = [];
defl_rate4_ct = [];
Fc4_ct = [];
pc4_ct = [];
flag_c4_ct = [];
vi_c4_ct = [];

%% Controller Response Params
Se = 0.05;

% display(sim_idx)
%% Simulation Loop
for i = t0:dt:tf-dt
%     display(size(ttotal))
    display(i)
    
   
    %Find Control Signal based on ref_r, ref_head
    if i ~= t0
        x0_step = X(end,:);
        ez_prev = ez;
        evz_prev = evz;
        eroll_prev = eroll;
        epitch_prev = epitch;
        eyaw_prev = eyaw;
        er_prev = er;
        omega_prev = omega;      
        
    end

    
    if recover*recover_control == 1
        ref_r = [0 traj_posn(2:3)];
        ref_head = traj_head;
        [control,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,u1,u2,u3,u4] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,eyaw_prev,er_prev,omega_prev,recover,body_accel);
    else
        [control,ez,evz,eroll,epitch,eyaw,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,u1,u2,u3,u4] = ControllerICRA_att(Xtotal(end,:),i,t0,dt,z_des,traj_head,traj_att,ez_prev,evz_prev,eroll_prev,epitch_prev,eyaw_prev,er_prev,omega_prev,recover,Tc_act, cmds_time, cmds);
%         if i >= Tc_act
%             u1 = -m*g;
%         end
%         
%         if i >= Tc_act + 0.3
%             u2 = 0;
%             u3 = 0;
%             u4 = 0;
%         end
    end
    
%     if i >= Tc_act + 0.08
%         wall_loc = 50;
%     end
    
           
    
    %Re-Initialize Contact Dynamics Variables
    [pint11,pint12,pc_w1,theta11,theta12,defl1,Fc1] = InitContactVar; 
    flag_c1 = flag_c_ct1;
    vi_c1 = vi_c_ct1;
    
    [pint21,pint22,pc_w2,theta21,theta22,defl2,Fc2] = InitContactVar; 
    flag_c2 = flag_c_ct2;
    vi_c2 = vi_c_ct2;
    
    [pint31,pint32,pc_w3,theta31,theta32,defl3,Fc3] = InitContactVar; 
    flag_c3 = flag_c_ct3;
    vi_c3 = vi_c_ct3;
    
    [pint41,pint42,pc_w4,theta41,theta42,defl4,Fc4] = InitContactVar; 
    flag_c4 = flag_c_ct4;
    vi_c4 = vi_c_ct4;
   
    %Propagate Dynamics
    options = odeset('RelTol',1e-3);
    [t,X] = ode45(@(t, X) SpiriMotion_4Circles_att(t,X,control,wall_loc,wall_plane,e,prop_speed,motors_time,motors_slope),[i i+dt],x0_step,options);
    
    %Reset contact flags for continuous time recording
    flag_c_ct1 = flag_c1;
    vi_c_ct1 = vi_c1;
    
    flag_c_ct2 = flag_c2;
    vi_c_ct2 = vi_c2;
    
    flag_c_ct3 = flag_c3;
    vi_c_ct3 = vi_c3;
    
    flag_c_ct4 = flag_c4;
    vi_c_ct4 = vi_c4;
    
    %Continuous time recording
    for j = 1:size(X,1)
        [dx,defl1_fine,Fc1,pc1,defl1_rate,defl2_fine,Fc2,pc2,defl2_rate,defl3_fine,Fc3,pc3,defl3_rate,defl4_fine,Fc4,pc4,defl4_rate, prop_speed] = SpiriMotion_4Circles_att(t(j),X(j,:),control,wall_loc,wall_plane,e, prop_speed, motors_time, motors_slope);
        
        dX_ct = [dX_ct;dx'];
        defl1_ct = [defl1_ct;defl1_fine];
        defl_rate1_ct = [defl_rate1_ct;defl1_rate];
        Fc1_ct = [Fc1_ct;Fc1];
        pc1_ct = [pc1_ct,pc1];        
        flag_c1_ct = [flag_c1_ct;flag_c_ct1];
        vi_c1_ct = [vi_c1_ct;vi_c_ct1];
        
        defl2_ct = [defl2_ct;defl2_fine];
        defl_rate2_ct = [defl_rate2_ct;defl2_rate];
        Fc2_ct = [Fc2_ct;Fc2];
        pc2_ct = [pc2_ct,pc2];        
        flag_c2_ct = [flag_c2_ct;flag_c_ct2];
        vi_c2_ct = [vi_c2_ct;vi_c_ct2];
        
        defl3_ct = [defl3_ct;defl3_fine];
        defl_rate3_ct = [defl_rate3_ct;defl3_rate];
        Fc3_ct = [Fc3_ct;Fc3];
        pc3_ct = [pc3_ct,pc3];        
        flag_c3_ct = [flag_c3_ct;flag_c_ct3];
        vi_c3_ct = [vi_c3_ct;vi_c_ct3];
        
        defl4_ct = [defl4_ct;defl4_fine];
        defl_rate4_ct = [defl_rate4_ct;defl4_rate];
        Fc4_ct = [Fc4_ct;Fc4];
        pc4_ct = [pc4_ct,pc4];        
        flag_c4_ct = [flag_c4_ct;flag_c_ct4];
        vi_c4_ct = [vi_c4_ct;vi_c_ct4];
    end
    
    if (flag_c1 || flag_c2 || flag_c3 || flag_c4) == 1
        recover = 1;
%         Tc_act = i;
    end
    
    t_ct = [t_ct;t];
    X_ct = [X_ct;X];
    
    body_accel = dX_ct(end,1:3)';
        
    q = [X(end,10);X(end,11);X(end,12);X(end,13)]/norm(X(end,10:13));
    R = quatRotMat(q);
   
    %Discrete Time recording @ 200 Hz
    
    u1_hist = [u1_hist,u1];
    u2_hist = [u2_hist,u2];
    u3_hist = [u3_hist,u3];
    u4_hist = [u4_hist,u4];
    
%     prop_speed = control(1:4); %in RPM;
    prop_accel = control(5:8); %in rad/s^2;
    
    prop_speed_hist = [prop_speed_hist,prop_speed];
    prop_accel_hist = [prop_accel_hist,prop_accel];
    
    roll_hist = [roll_hist;roll];
    pitch_hist = [pitch_hist;pitch];
    yaw_hist = [yaw_hist;yaw];
    
    rolldes_hist = [rolldes_hist;roll_des];
    pitchdes_hist = [pitchdes_hist;pitch_des];
    rdes_hist = [rdes_hist;r_des];
    
    Xtotal = [Xtotal;X(end,:)];
    ttotal = [ttotal;t(end)];
   
    pint11_hist = [pint11_hist,pint11];
    pint12_hist = [pint12_hist,pint12];
    pc_w1_hist = [pc_w1_hist,pc_w1];
    defl1_hist = [defl1_hist;defl1];
    theta11_hist = [theta11_hist;theta11];
    theta12_hist = [theta12_hist;theta12];
    
    pint21_hist = [pint21_hist,pint21];
    pint22_hist = [pint22_hist,pint22];
    pc_w2_hist = [pc_w2_hist,pc_w2];
    defl2_hist = [defl2_hist;defl2];
    theta21_hist = [theta21_hist;theta21];
    theta22_hist = [theta22_hist;theta22];
    
    pint31_hist = [pint31_hist,pint31];
    pint32_hist = [pint32_hist,pint32];
    pc_w3_hist = [pc_w3_hist,pc_w3];
    defl3_hist = [defl3_hist;defl3];
    theta31_hist = [theta31_hist;theta31];
    theta32_hist = [theta32_hist;theta32];
    
    pint41_hist = [pint41_hist,pint41];
    pint42_hist = [pint42_hist,pint42];
    pc_w4_hist = [pc_w4_hist,pc_w4];
    defl4_hist = [defl4_hist;defl4];
    theta41_hist = [theta41_hist;theta41];
    theta42_hist = [theta42_hist;theta42];

    %End loop if Spiri has crashed
    if Xtotal(end,9) <= 0
        display('Spiri has hit the floor :(');
        stable = 0;
        break;
    end  
    
end

theta1_hist = [theta11_hist,theta12_hist];
theta2_hist = [theta21_hist,theta22_hist];
theta3_hist = [theta31_hist,theta32_hist];
theta4_hist = [theta41_hist,theta42_hist];

[Ts, PO] = ControllerStats(ttotal,Xtotal,Se,traj_posn,traj_head);

% Graphs( ttotal,Xtotal,roll_hist,pitch_hist,yaw_hist,rolldes_hist,pitchdes_hist,rdes_hist,u1_hist,u2_hist,u3_hist,u4_hist);

% figure()
% plot(t_ct,dX_ct(:,7),t_ct,dX_ct(:,8),t_ct,dX_ct(:,9));
% title('World Velocities');
% ylabel('Velocity (m/s)');
% xlabel('Time (s)');
% legend('x^W','y^W','z^W');

% savestring = strcat('BatchSim_',num2str(sim_idx,'%03i'));
% print(savestring,'-dpng');
% savefig(savestring);
% close;

%plot prop speed and accelerations
% figure();
% subplot(2,1,1);
% plot(ttotal,prop_speed_hist);
% title('Prop Speeds (RPM)');
% subplot(2,1,2);
% plot(ttotal,prop_accel_hist);
% title('Prop Accelerations (rad/s^2)');
% grid on;

if sum(defl1_hist)+sum(defl2_hist)+sum(defl3_hist)+sum(defl4_hist) > 0
   
%     Graphs_Contact(ttotal,Xtotal,dX_ct,wall_loc,t_ct,defl1_ct,Fc1_ct,defl_rate1_ct,flag_c1_ct,vi_c1_ct,defl2_ct,Fc2_ct,defl_rate2_ct,flag_c2_ct,vi_c2_ct,defl3_ct,Fc3_ct,defl_rate3_ct,flag_c3_ct,vi_c3_ct,defl4_ct,Fc4_ct,defl_rate4_ct,flag_c4_ct,vi_c4_ct);
end
%     Graphs_Contact(ttotal,Xtotal,dX_ct,wall_loc,t_ct,defl1_ct,Fc1_ct,defl_rate1_ct,flag_c1_ct,vi_c1_ct,defl2_ct,Fc2_ct,defl_rate2_ct,flag_c2_ct,vi_c2_ct,defl3_ct,Fc3_ct,defl_rate3_ct,flag_c3_ct,vi_c3_ct,defl4_ct,Fc4_ct,defl_rate4_ct,flag_c4_ct,vi_c4_ct);

% SpiriVis(0,ttotal,Xtotal,'V2',wall_loc,'YZ',pint11_hist,pint12_hist,pc_w1_hist,pint21_hist,pint22_hist,pc_w2_hist,pc_w3_hist,pc_w4_hist);

[ti1, ~, defls1, Fns1, ~, Vx1, ~, ~, ~, ~, Tilt1, numContacts1] = ContactStats( t_ct,X_ct,defl1_ct, Fc1_ct,vi_c1_ct,traj_head);
[ti2, ~, defls2, Fns2, ~, Vx2, ~, ~, ~, ~, Tilt2, numContacts2] = ContactStats( t_ct,X_ct,defl2_ct, Fc2_ct,vi_c2_ct,traj_head);
[ti3, ~, defls3, Fns3, ~, Vx3, ~, ~, ~, ~, Tilt3, numContacts3] = ContactStats( t_ct,X_ct,defl3_ct, Fc3_ct,vi_c3_ct,traj_head);
[ti4, ~, defls4, Fns4, ~, Vx4, ~, ~, ~, ~, Tilt4, numContacts4] = ContactStats( t_ct,X_ct,defl4_ct, Fc4_ct,vi_c4_ct,traj_head);

if traj_head == pi/4    
    tilt = Tilt4(1);
    defl_init = defls4(1);
    Fn_init = Fns4(1);
    Vc_act = Vx4(1);
elseif traj_head == 0
    
    tilt = (defls1(1)>=defls4(1))*Tilt1(1) + (defls1(1)<defls4(1))*Tilt4(1);
    defl_init = (defls1(1)>=defls4(1))*defls1(1) + (defls1(1)<defls4(1))*defls4(1);
    Fn_init = (defls1(1)>=defls4(1))*Fns1(1) + (defls1(1)<defls4(1))*Fns4(1);
    Vc_act = (defls1(1)>=defls4(1))*Vx1(1) + (defls1(1)<defls4(1))*Vx4(1);
    
end

Fn_max = max([Fns1,Fns2,Fns3,Fns4]);
defl_max = max([defls1,defls2,defls3,defls4]);

numContacts = numContacts1 + numContacts2 + numContacts3 + numContacts4;

% end
% 