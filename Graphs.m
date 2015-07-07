function [  ] = Graphs( ttotal,Xtotal,roll_hist,pitch_hist,yaw_hist,rolldes_hist,pitchdes_hist,rdes_hist)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

figure('units','normalized','outerposition',[0 0 1 1])
% figure();
subplot(2,3,1);
plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
legend('u','v','w');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

subplot(2,3,2);
plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
legend('p','q','r');
xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');
grid on;

subplot(2,3,3);
ax=gca;
plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,Xtotal(:,9));
legend('X^w','Y^w','Z^w');
% ax.XTick = [0 1 2 3 4 5 6 7 8 9 10];
xlabel('Time (s)');
ylabel('World Position (m)');
grid on;

subplot(2,3,4);
plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13));
legend('q_0','q_1','q_2','q_3');
xlabel('Time (s)');
grid on;
% ylabel('');

subplot(2,3,5);
plot(ttotal,rolldes_hist,ttotal,pitchdes_hist,ttotal,rdes_hist);
legend('roll_des','pitch_des','r_des');
xlabel('Time (s)');
ylabel('Desired Angle (rad)');
grid on;
% yl = ylim;

subplot(2,3,6);
plot(ttotal,roll_hist,ttotal,pitch_hist,ttotal,yaw_hist);
legend('roll','pitch','yaw');
xlabel('Time (s)');
ylabel('Angle (rad)');
% ylim(yl);
grid on;

end

