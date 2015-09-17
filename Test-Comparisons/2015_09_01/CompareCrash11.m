%--------------------%
%%Simulator Settings
% e = 0.9; 
% Vc_des = 2.1; %1.98
% recover_control = 0;
% traj_head = -pi + deg2rad(10);
% roll0 = deg2rad(-6);
% pitch0 = deg2rad(23.7);
% z_start = 0.6;
% z_des = 1.65; %roll0 = -6: 2.1
% ax_given = 5.0097; %roll0 = -6: 3.995, roll0 = 6: 4.891
% tfinal = 1.2;
%
%Initial Velocities
% Xdot_C = 1.9769 m/s
% Zdot_c = 0.4413 m/s
%
%Control Settings
%0s after impact: u1 = -m*10.5, r_des = 0, roll_des = -0.5deg, pitch_des =
%12deg
%0.3s after impact: u1 = m*g, r_des = 0, roll_des = 0, pitch_des = 0;


load('crash11_sim.mat');
load('crash_11.mat');

Tc_exp = 13.1 + 0.0616;
Tc_sim = 0.5311;%0.44988;

figure()
v_accel___time = v_accel___time - Tc_exp;
t_ct = t_ct - Tc_sim;


subplot(2,1,1);
plot(v_accel___time,v_accel_x/10,v_accel___time,v_accel_y/10,v_accel___time,v_accel_z/10);
hold on;
plot([0.27 0.27],[-35 35],'r-');
xlim([0 0.5]);
ylim([-35 35])
xlabel('Time after impact (s)');
ylabel('Acceleration (m/s^2)');
str = {'$$ \ddot{x} $$', '$$ \ddot{y} $$', '$$ \ddot{z} $$'};
legend(str, 'Interpreter','latex', 'Location','eastoutside')
title('Experimental Body Accelerations - Crash 11');

subplot(2,1,2);
plot(t_ct,dX_ct(:,1),t_ct,dX_ct(:,2),t_ct,dX_ct(:,3))
hold on;
plot([0.27 0.27],[-35 35],'r-');
xlim([0 0.5]);
ylim([-35 35])
xlabel('Time after impact (s)');
ylabel('Acceleration (m/s^2)');
str = {'$$ \ddot{x} $$', '$$ \ddot{y} $$', '$$ \ddot{z} $$'};
legend(str, 'Interpreter','latex', 'Location','eastoutside')
title('Simulated Body Accelerations - Crash 11');
%%--------------------------------------------------------------------------------%
figure()
v_gyros___time = v_gyros___time - Tc_exp;

subplot(2,1,1);
plot(v_gyros___time,v_gyros_x/10,v_gyros___time,v_gyros_y/10,v_gyros___time,v_gyros_z/10);
hold on;
plot([0.27 0.27],[-35 35],'r-');
xlim([0 0.5]);
ylim([-35 35])
xlabel('Time after impact (s)');
ylabel('Angular Velocity (rad/s)');
legend('p','q','r','Location','eastoutside');
title('Experimental Body Angular Velocities - Crash 11');

subplot(2,1,2);
plot(t_ct,X_ct(:,4),t_ct,X_ct(:,5),t_ct,X_ct(:,6))
hold on;
plot([0.27 0.27],[-35 35],'r-');
xlim([0 0.5]);
ylim([-35 35])
xlabel('Time after impact (s)');
ylabel('Angular Velocity (rad/s)');
legend('p','q','r','Location','eastoutside');
title('Simulated Body Angular Velocities - Crash 11');

%%--------------------------------------------------------------------------------%
figure()
vangles___time = vangles___time - Tc_exp;
ttotal = ttotal - Tc_sim;

subplot(2,1,1);
plot(vangles___time,vangles_x,vangles___time,vangles_y);
hold on;
plot([0.27 0.27],[-180 180],'r-');
xlim([0 0.5]);
ylim([-180 180])
xlabel('Time after impact (s)');
ylabel('Euler Angle (rad)');
legend('\phi','\theta','Location','eastoutside');
title('Experimental Euler Angles - Crash 11');

subplot(2,1,2);
plot(ttotal,rad2deg(roll_hist),ttotal,rad2deg(pitch_hist))
hold on;
plot([0.27 0.27],[-180 180],'r-');
xlim([0 0.5]);
ylim([-180 180])
xlabel('Time after impact (s)');
ylabel('Euler Angle (rad)');
legend('\phi','\theta','Location','eastoutside');
title('Simulated Euler Angles - Crash 11');
