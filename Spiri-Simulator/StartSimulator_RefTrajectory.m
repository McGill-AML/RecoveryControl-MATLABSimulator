clear all;
% close all;
clc;

%%Spiri System Parameters
InitSpiriParams;

%%Simulation Parameters
traj_posn = [0 1 0;2 2 -2;2 4 -4];
traj_head = [0;pi/4;-pi/4];
traj_time = [0;10;20];
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/30;
% ref_r = [2 2 -5]';
% ref_head = pi/4;
x0 = [zeros(6,1);traj_posn(1,:)';[1;0;0;0];zeros(3,1)];
omega0 = zeros(4,1);

[posn,head] = CreateTrajectory(traj_posn,traj_head,traj_time,dt);

%%Initial Variable Values
x0_step = x0;
Xtotal = x0';
ttotal = t0;
vztotal = [];
ez_prev = 0;
evz_prev = 0;
evx_prev = 0;
evy_prev = 0;
eyaw_prev = 0;
eroll_prev = 0;
epitch_prev = 0;
er_prev = 0;
omega_prev = omega0;

index = 1;
for i = t0:dt:tf-dt
    display(i)
    
    ref_r = posn(index,:)';
    ref_head = head(index);
    index = index + 1;
    
    %Find Control Signal based on ref_r, ref_head
    if i ~= t0
        x0_step = X(end,:);
        ez_prev = ez;
        evz_prev = evz;
        evx_prev = evx;
        evy_prev = evy;
        eyaw_prev = eyaw;
        eroll_prev = eroll;
        epitch_prev = epitch;
        er_prev = er;
        omega_prev = omega;
    end
    [signal_c3,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,er_prev,omega_prev);
    
    %Use Control Signal to propagate dynamics
    [t,X] = ode45(@(t, X) SpiriMotion(t,X,signal_c3),[i i+dt],x0_step);

    Xtotal = [Xtotal;X(end,:)];
    ttotal = [ttotal;t(end)];    

end

figure();
subplot(2,2,1);
plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
legend('u','v','w');
subplot(2,2,2);
plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
legend('p','q','r');
subplot(2,2,3);
plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,-Xtotal(:,9),ttotal,Xtotal(:,16));
legend('x','y','-z','yaw');
subplot(2,2,4);
plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13));
legend('q_0','q_1','q_2','q_3');


% SpiriVisualization(ttotal,Xtotal);





