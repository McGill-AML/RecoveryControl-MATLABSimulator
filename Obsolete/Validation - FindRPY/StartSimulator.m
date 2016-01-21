clear all;
% close all;
clc;

global CM prop_loc m

%%Spiri System Parameters
InitSpiriParams;
r_ribbon = 0.31;

%%Simulation Parameters
traj_posn = [0 2.5 5;0 2.5 5];
traj_head = [0;0];
traj_time = [0;10];
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/50;
% ref_r = [2 2 -5]';
% ref_head = pi/4;

q0 = [0.5;0.5;0;0.5];
q0 = q0/norm(q0);
[roll0, pitch0, yaw0] = quat2angle(q0,'xyz');
x0 = [zeros(3,1);[pi/2;0;0];traj_posn(1,:)';q0;roll0;pitch0;yaw0];

%%Initial Variable Values
x0_step = x0;
Xtotal = x0';
ttotal = t0;

q = [x0(10);x0(11);x0(12);x0(13)]/norm(x0(10:13));
R = quatRotMat(q);
[roll, pitch, yaw] = quat2angle(q,'xyz');

ag_hist =  (R*[0;0;-9.81])';

roll_hist = roll;
roll_hist2 = x0(14);
pitch_hist = pitch;
pitch_hist2 = x0(15);
yaw_hist = yaw;
yaw_hist2 = x0(16);

    
for i = t0:dt:tf-dt
%     display(i)

    if i ~= t0
        x0_step = X(end,:);
    end


       
    %Use Control Signal to propagate dynamics
[t,X] = ode113(@(t, X) SpiriMotion(t,X,[],[],[],[],[],[]),[i i+dt],x0_step);

q = [X(end,10);X(end,11);X(end,12);X(end,13)]/norm(X(end,10:13));
R = quatRotMat(q);
% R2 = RotMat('X',X(end,14))*RotMat('Y',X(end,15))*RotMat('Z',X(end,16));
% R_disp = [R,[0;0;0],R2];
% disp(R_disp);

[roll, pitch, yaw] = quat2angle(q,'xyz');

ag = R*[0;0;-9.81];

Xtotal = [Xtotal;X(end,:)];
ttotal = [ttotal;t(end)];
roll_hist = [roll_hist;roll];
pitch_hist = [pitch_hist;pitch];
yaw_hist = [yaw_hist;yaw];

ag_hist = [ag_hist;ag'];

roll_hist2 = [roll_hist2;X(end,14)];
pitch_hist2 = [pitch_hist2;X(end,15)];
yaw_hist2 = [yaw_hist2;X(end,16)];

end

roll_hist2 = mod(roll_hist2,pi);
pitch_hist2 = mod(pitch_hist2,pi);
yaw_hist2 = mod(yaw_hist2,pi);


% Graphs( ttotal,Xtotal,ttotal,[],[] );
close all;
figure();
plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13),ttotal,sqrt(Xtotal(:,10).^2+Xtotal(:,11).^2+Xtotal(:,12).^2+Xtotal(:,13).^2));
legend('q_0','q_1','q_2','q_3','|q|');
title('q');

figure();
plot(ttotal,roll_hist,ttotal,roll_hist2);
legend('Quaternion','RPY');
title('Roll');

figure();
plot(ttotal,pitch_hist,ttotal,pitch_hist2);
legend('Quaternion','RPY');
title('Pitch');

figure();
plot(ttotal,yaw_hist,ttotal,yaw_hist2);
legend('Quaternion','RPY');
title('Yaw');

figure();
plot(ttotal,ag_hist(:,1),ttotal,ag_hist(:,2),ttotal,ag_hist(:,3));

% SpiriVisualization(ttotal,Xtotal);





