% function [Ts, PO] = StartSimulator(traj_posn,traj_head,traj_time,sim_idx)

% clear all;
% close all;
% clc;
clear X

global m g Kt Rbumper flag_c_fine1 vi_c_fine1 flag_c_fine2 vi_c_fine2

%% Spiri System Parameters
InitSpiriParams;

%% Simulation Parameters

% traj_posn = [0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0];
% traj_head = [0;pi/2;pi;-pi/2;0;-pi/2];
% 
% traj_time = [0;5;10;15;20;25];

%Contact with max pitch and velocity of ~1.2 m/s
traj_posn = [0 0 2;4 0 2];
traj_head = [pi/4; pi/4];
traj_time = [0; 2];
wall_loc = 0.29*0.95 + 0.2613;
wall_plane = 'YZ';


% sim_idx = 40;
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/200;

%% Create Trajectory
[posn,head] = CreateTrajectory(traj_posn,traj_head,traj_time,dt);
traj_index = 1;

%% Initial Variable Values

% States
q0 = quatmultiply([0;-1;0;0],[cos(traj_head(1)/2);0;0;sin(traj_head(1)/2)]);
q0 = q0/norm(q0);

omega0 = [-1;1;-1;1].*repmat(sqrt(m*g/(4*Kt)),4,1);  %Start with hovering RPM
prop_speed = omega0;
x0 = [zeros(6,1);traj_posn(1,:)';q0];

Xtotal = x0';
ttotal = t0;

q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)]/norm(Xtotal(end,10:13));
R = quatRotMat(q);
[roll, pitch, yaw] = quat2angle(q,'xyz');
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
flag_c_fine1 = 0;
vi_c_fine1 = 0;

flag_c2 = 0;
vi_c2 = 0;
flag_c_fine2 = 0;
vi_c_fine2 = 0;

recovery = 0;
accel = 0;

%% Initialize History Arrays

% States
roll_hist = roll;
pitch_hist = pitch;
yaw_hist = yaw;

rolldes_hist = 0;
pitchdes_hist = 0;
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

% Continuous Time
t_tot = [];
X_tot = [];
dX_tot = [];

defl1_tot = [];
defl_rate1_tot = [];
Fc1_tot = [];
pc1_tot = [];
flag_c1_tot = [];
vi_c1_tot = [];

defl2_tot = [];
defl_rate2_tot = [];
Fc2_tot = [];
pc2_tot = [];
flag_c2_tot = [];
vi_c2_tot = [];

%% Controller Response Params
Se = 0.05;

% display(sim_idx)
%% Simulation Loop
for i = t0:dt:tf-dt
%     display(size(ttotal))
    display(i)
    
    %Trajectory Control Position
    ref_r = posn(traj_index,:)';
    ref_head = head(traj_index);
    traj_index = traj_index + 1;
    
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
    
    [signal_c,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,u1,u2,u3,u4,recovery] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,eyaw_prev,er_prev,omega_prev,accel,recovery);

    %Re-Initialize Contact Dynamics Variables
    pint11 = [100;100;0];
    pint12 = [100;100;0];
    pc_w1 = [100;100;0];
    theta11 = 0;
    theta12 = 0;
    defl1 = 0;
    Fc1 = 0;
    flag_c1 = flag_c_fine1;
    vi_c1 = vi_c_fine1;
    
    pint21 = [100;100;0];
    pint22 = [100;100;0];
    pc_w2 = [100;100;0];
    theta21 = 0;
    theta22 = 0;
    defl2 = 0;
    Fc2 = 0;
    flag_c2 = flag_c_fine1;
    vi_c2 = vi_c_fine1;
   
    %Propagate Dynamics
    
    [t,X] = ode45(@(t, X) SpiriMotion_4Circles(t,X,signal_c,wall_loc,wall_plane),[i i+dt],x0_step);
    
    %Reset contact flags for continuous time recording
    flag_c_fine1 = flag_c1;
    vi_c_fine1 = vi_c1;
    
    flag_c_fine2 = flag_c2;
    vi_c_fine2 = vi_c2;
    
    %Continuous time recording
    for j = 1:size(X,1)
        [dx,defl1_fine,Fc1,pc1,defl1_rate,defl2_fine,Fc2,pc2,defl2_rate] = SpiriMotion_4Circles(t(j),X(j,:),signal_c,wall_loc,wall_plane);
        
        dX_tot = [dX_tot;dx'];
        defl1_tot = [defl1_tot;defl1_fine];
        defl_rate1_tot = [defl_rate1_tot;defl1_rate];
        Fc1_tot = [Fc1_tot;Fc1];
        pc1_tot = [pc1_tot,pc1];        
        flag_c1_tot = [flag_c1_tot;flag_c_fine1];
        vi_c1_tot = [vi_c1_tot;vi_c_fine1];
        
        defl2_tot = [defl2_tot;defl2_fine];
        defl_rate2_tot = [defl_rate2_tot;defl2_rate];
        Fc2_tot = [Fc2_tot;Fc2];
        pc2_tot = [pc2_tot,pc2];        
        flag_c2_tot = [flag_c2_tot;flag_c_fine2];
        vi_c2_tot = [vi_c2_tot;vi_c_fine2];
    end
    
    t_tot = [t_tot;t];
    X_tot = [X_tot;X];
    
    accel = sqrt(sum(dX_tot(end,1).^2 + dX_tot(end,2).^2 + dX_tot(end,3).^2,2));
        
    q = [X(end,10);X(end,11);X(end,12);X(end,13)]/norm(X(end,10:13));
    R = quatRotMat(q);
   
    %Discrete Time recording @ 200 Hz
    
    u1_hist = [u1_hist,u1];
    u2_hist = [u2_hist,u2];
    u3_hist = [u3_hist,u3];
    u4_hist = [u4_hist,u4];
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

    %End loop if Spiri has crashed
    if Xtotal(end,9) <= -1
        display('Spiri has hit the floor :(');
        break;
    end  
    
end

theta1_hist = [theta11_hist,theta12_hist];
theta2_hist = [theta21_hist,theta22_hist];

[Ts, PO] = ControllerStats(ttotal,Xtotal,Se,traj_posn,traj_head);

% Graphs( ttotal,Xtotal,roll_hist,pitch_hist,yaw_hist,rolldes_hist,pitchdes_hist,rdes_hist,u1_hist,u2_hist,u3_hist,u4_hist);

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

if sum(defl1_hist) > 0
    defl_rate_check = zeros(size(defl1_tot));
    for i=2:size(defl1_tot);
        defl_rate_check(i) = (defl1_tot(i) - defl1_tot(i-1))/dt;
    end    
    
    Graphs_Contact(ttotal,Xtotal,dX_tot,wall_loc,t_tot,defl1_tot,Fc1_tot,defl_rate1_tot,flag_c1_tot,vi_c1_tot,defl2_tot,Fc2_tot,defl_rate2_tot,flag_c2_tot,vi_c2_tot);
end

% SpiriVisualization1(record,ttotal,Xtotal,'XZ',wall_loc,'YZ',pint1_hist,pint2_hist,pc_w_hist)
SpiriVisualization1(0,ttotal,Xtotal,'XY',wall_loc,'YZ',pint11_hist,pint12_hist,pc_w1_hist,pint21_hist,pint22_hist,pc_w2_hist)
% end
% 
