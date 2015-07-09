function [Ts, PO] = StartSimulator(traj_posn,traj_head,traj_time,sim_idx)

% clear all;
% close all;
% clc;

global Rbumper

%% Spiri System Parameters
InitSpiriParams;

%% Simulation Parameters
<<<<<<< HEAD
traj_posn = [0 0 5;0 0 6];
traj_head = [0;0];
traj_time = [0;5];
wall_loc = 10000;
sim_idx = 40;

=======
% traj_posn = [0 0 5;5 5 5];
% traj_head = [0;0];
% traj_time = [0;5];
>>>>>>> parent of fc3d37d... Controller working
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/100;
wall_loc = 10000;
wall_plane = 'YZ';

%% Create Trajectory
[posn,head] = CreateTrajectory(traj_posn,traj_head,traj_time,dt);
traj_index = 1;

%% Initial Variable Values

q0 = quatmultiply([0;-1;0;0],[cos(traj_head(1)/2);0;0;sin(traj_head(1)/2)]);
q0 = q0/norm(q0);
<<<<<<< HEAD
x0 = [zeros(6,1);traj_posn(1,:)';q0;0];
=======
x0 = [zeros(6,1);traj_posn(1,:)';q0;zeros(3,1)];
>>>>>>> parent of fc3d37d... Controller working
omega0 = zeros(4,1);

Xtotal = x0';
ttotal = t0;

q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)]/norm(Xtotal(end,10:13));
R = quatRotMat(q);
[roll, pitch, yaw] = quat2angle(q,'xyz');

% Controller Initial Variables
x0_step = x0;
ez_prev = 0;
evz_prev = 0;
eroll_prev = 0;
epitch_prev = 0;
er_prev = 0;
omega_prev = omega0;

ez_i = 0;

%% Initialize History Arrays

u1_hist = 0;
ez_hist = 0;
evz_hist = 0;

roll_hist = roll;
pitch_hist = pitch;
yaw_hist = yaw;

rolldes_hist = 0;
pitchdes_hist = 0;
rdes_hist = 0;

pint1_hist = [0;0;0];
pint2_hist = [0;0;0];
pc_w_hist = [0;0;0];
defl_hist = 0;
theta_hist = 0;

%% Controller Response Params
Se = 0.05;

display(sim_idx)
%% Simulation Loop
for i = t0:dt:tf-dt
%     display(size(ttotal))
%     display(i)
    
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
        er_prev = er;
        omega_prev = omega; 
        
        ez_i = X(end,14);
    end
    
    [signal_c3,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des,icomp] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,er_prev,omega_prev,ez_i);
    
    %Initialize Contact Dynamics Variables
    pint1 = [0;0;0];
    pint2 = [0;0;0];
    pc_w = [0;0;0];
    defl = 0;
    theta1 = 0;
    
    %Propagate Dynamics
%     [ pt1,pt2,Pc_w] = DetectContact1(Xtotal(end,:),wall_loc,wall_plane);    
    [t,X] = ode45(@(t, X) SpiriMotion(t,X,signal_c3,wall_loc,wall_plane,ref_r),[i i+dt],x0_step);

    q = [X(end,10);X(end,11);X(end,12);X(end,13)]/norm(X(end,10:13));
    R = quatRotMat(q);
   
    disp(strcat('controller: ',num2str(icomp),'   ode: ',num2str(X(end,14))));
    %Record Data
    pint1_hist = [pint1_hist,pint1];
    pint2_hist = [pint2_hist,pint2];
    pc_w_hist = [pc_w_hist,pc_w];
    defl_hist = [defl_hist;defl];
    theta_hist = [theta_hist;theta1];
    
    roll_hist = [roll_hist;roll];
    pitch_hist = [pitch_hist;pitch];
    yaw_hist = [yaw_hist;yaw];
    
    rolldes_hist = [rolldes_hist;roll_des];
    pitchdes_hist = [pitchdes_hist;pitch_des];
    rdes_hist = [rdes_hist;r_des];
    
    Xtotal = [Xtotal;X(end,:)];
    ttotal = [ttotal;t(end)];
   
    %End loop if Spiri has crashed
    if Xtotal(end,9) <= -1
        display('Spiri has hit the floor :(');
        break;
    end  
    
end

[Ts, PO] = ControllerStats(ttotal,Xtotal,Se,traj_posn,traj_head);

Graphs( ttotal,Xtotal,roll_hist,pitch_hist,yaw_hist,rolldes_hist,pitchdes_hist,rdes_hist);
savestring = strcat('BatchSim_',num2str(sim_idx,'%03i'));
print(savestring,'-dpng');
savefig(savestring);
close;

% figure();
% plot(ttotal,roll_hist,ttotal,pitch_hist,ttotal,yaw_hist);
% legend('roll','pitch','yaw');
% 
% figure();
% plot(ttotal,rolldes_hist,ttotal,pitchdes_hist,ttotal,rdes_hist);
% legend('roll_des','pitch_des','r_des');

if sum(defl_hist) > 0
    figure();
    plot(ttotal,defl_hist)
    hold on
    plot(ttotal,Xtotal(:,7)+Rbumper-wall_loc)
    title('Deflection and Unrotated Deflection');
    legend('Calculated Defl','Defl if not rotated');
end

% SpiriVisualization1(record,ttotal,Xtotal,'XZ',wall_loc,'YZ',pint1_hist,pint2_hist,pc_w_hist)

end
% 
