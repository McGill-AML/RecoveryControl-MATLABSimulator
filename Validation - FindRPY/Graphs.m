function [  ] = Graphs( ttotal,Xtotal,defl_time,defl,Fc )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure();
subplot(2,2,1);
plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
legend('u','v','w');
subplot(2,2,2);
plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
legend('p','q','r');
subplot(2,2,3);
plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,Xtotal(:,9));
legend('X^w','Y^w','Z^w');
subplot(2,2,4);
plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13),ttotal,sqrt(Xtotal(:,10).^2+Xtotal(:,11).^2+Xtotal(:,12).^2+Xtotal(:,13).^2));
legend('q_0','q_1','q_2','q_3','|q|');
% 
% figure('position', [600, 300, 800, 500]) 
% set(gca,'FontSize',14)
% subplot(1,2,1);
% plot(defl_time,defl*100,'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Deflection (cm)');
% % axis([2 4 0 0.2]);
% grid on;
% % set(gca,'FontSize',14)
% subplot(1,2,2);
% plot(defl_time,Fc,'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Contact Force (N)');
% grid on;
% % axis([2 4 0 40]);
% % set(gca,'FontSize',14)
% suptitle({'Bumper Deflection and Contact Force During Impact'});
% 
% 
% figure('position', [0, 200, 600, 500]) 
% plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,Xtotal(:,9),'LineWidth',2);
% hold on;
% plot(ttotal,repmat(4,size(ttotal)),'LineWidth',2);
% % set(gca,'FontSize',14)
% title({'Quadrotor CoM Position','Prescribed Position: (5.0, 2.5, 5.0)m'})
% legend('X','Y','Z','Wall in X-Z Plane','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Position (m)');
% grid on;
% 
% figure('position', [600, 200, 600, 500]) 
% % plot(ttotal,mod(Xtotal(:,14),pi),ttotal,mod(Xtotal(:,15),pi),ttotal,mod(Xtotal(:,16),pi),'LineWidth',2);
% plot(ttotal,Xtotal(:,14),ttotal,Xtotal(:,15),ttotal,Xtotal(:,16),'LineWidth',2);
% % set(gca,'FontSize',14)
% title({'Quadrotor Attitude','Prescribed Position: (5.0, 2.5, 5.0)m'})
% legend('\phi','\theta','\psi','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% grid on;


end

