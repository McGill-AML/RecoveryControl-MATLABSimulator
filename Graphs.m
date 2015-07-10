function [  ] = Graphs( ttotal,Xtotal,roll_hist,pitch_hist,yaw_hist,rolldes_hist,pitchdes_hist,rdes_hist)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

figure('units','normalized','outerposition',[0 0 1 1])
% figure();
subplot(2,4,1);
plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
legend('u','v','w','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Body linear velocities');
grid on;

subplot(2,4,2);
plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
legend('p','q','r','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');
title('Body angular velocities');
grid on;

subplot(2,4,3);
ax=gca;
plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,Xtotal(:,9));
legend('X^w','Y^w','Z^w','Location','southoutside','Orientation','horizontal');
% ax.XTick = [0 1 2 3 4 5 6 7 8 9 10];
xlabel('Time (s)');
ylabel('World Position (m)');
title('World positions');
grid on;

% subplot(2,3,4);
% plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13));
% legend('q_0','q_1','q_2','q_3');
% xlabel('Time (s)');
% grid on;
% ylabel('');

subplot(2,4,4)
plot(ttotal,yaw_hist);
legend('yaw','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angle (rad');
title('Actual Yaw');
grid on;

subplot(2,4,5)
plot(ttotal,rolldes_hist,ttotal,roll_hist);
legend('roll_{des}','roll','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Des. & Act. \phi');
grid on;

subplot(2,4,6)
plot(ttotal,pitchdes_hist,ttotal,pitch_hist);
legend('pitch_{des}','pitch','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Des. & Act. \theta');
grid on;

subplot(2,4,7)
plot(ttotal,rdes_hist,ttotal,Xtotal(:,6));
legend('r_{des}','r','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Des. & Act. r');
grid on;
% 
% subplot(2,3,5);
% plot(ttotal,rolldes_hist,ttotal,pitchdes_hist,ttotal,rdes_hist);
% legend('roll_des','pitch_des','r_des');
% xlabel('Time (s)');
% ylabel('Desired Angle (rad)');
% grid on;
% % yl = ylim;
% 
% subplot(2,3,6);
% plot(ttotal,roll_hist,ttotal,pitch_hist,ttotal,yaw_hist);
% legend('roll','pitch','yaw');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% % ylim(yl);
% grid on;

end

