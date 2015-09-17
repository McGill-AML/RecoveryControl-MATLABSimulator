% figure('Position',[580 50 1250 895]);
figure();

load('fig_response_D.mat');

%C & D
x_range = [0 5];
Fn_range = [0 15];
v_range = [-10 10];
w_range = [-10 10];
r_range = [-2.5 2.5];
angle_range = [-2 2];
tick_space = 1;
% 
% subplot(2,3,1)
% plot(t_ct,defl1_ct,'-','Color',[0 102 0]/256,'LineWidth',2);
% hold on;
% plot(t_ct,defl2_ct,'b--','LineWidth',2);
% plot(t_ct,defl3_ct,'m-','LineWidth',2);
% plot(t_ct,defl4_ct,'r--','LineWidth',2);
% xlabel('Time (s)');
% ylabel('\delta (m)');
% legend('i = 1','i = 2','i = 3','i = 4','Location','northwest');
% set(gca,'LineWidth',0.75','FontSize',12);
% xlim(x_range);
% set(gca,'XTick',x_range(1):tick_space:x_range(2));
% set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));
% 
% subplot(2,3,4);
% plot(t_ct,Fc1_ct,'-','Color',[0 102 0]/256,'LineWidth',2);
% hold on;
% plot(t_ct,Fc2_ct,'b--','LineWidth',2);
% plot(t_ct,Fc3_ct,'m-','LineWidth',2);
% plot(t_ct,Fc4_ct,'r--','LineWidth',2);
% xlabel('Time (s)');
% ylabel('F_n (N)');
% legend('i = 1','i = 2','i = 3','i = 4','Location','northwest');
% set(gca,'LineWidth',0.75','FontSize',12);
% xlim(x_range);
% ylim(Fn_range);
% set(gca,'XTick',x_range(1):tick_space:x_range(2));
% set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));

subplot(2,1,1);
plot(ttotal,Xtotal(:,7),'r-','LineWidth',2);
hold on;
plot(ttotal,Xtotal(:,8),'b--','LineWidth',2);
plot(ttotal,Xtotal(:,9),'m-.','LineWidth',2);
xlabel('Time (s)');
ylabel('Position (m)');
legend('X','Y','Z','Location','eastoutside');
set(gca,'LineWidth',0.75','FontSize',12);
xlim(x_range);
ylim(r_range);
set(gca,'XTick',x_range(1):tick_space:x_range(2));
set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));

% subplot(2,3,5);
% plot(ttotal,Xtotal(:,1),'r-','LineWidth',2);
% hold on;
% plot(ttotal,Xtotal(:,2),'b--','LineWidth',2);
% plot(ttotal,Xtotal(:,3),'m-.','LineWidth',2);
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% legend1 = sprintf('\\bfv\\rm_{CX} = 0.50 \\pm 5%% (m/s)');
% legend('u','v','w','Location','northwest');
% set(gca,'LineWidth',0.75','FontSize',12);
% xlim(x_range);
% ylim(v_range);
% set(gca,'XTick',x_range(1):tick_space:x_range(2));
% set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));

subplot(2,1,2);
plot(ttotal,roll_hist,'r-','LineWidth',2);
hold on;
plot(ttotal,pitch_hist,'b--','LineWidth',2);
plot(ttotal,yaw_hist,'m-.','LineWidth',2);
plot(ttotal,rolldes_hist,'r-','LineWidth',1);
plot(ttotal,pitchdes_hist,'b--','LineWidth',1);
xlabel('Time (s)');
ylabel('Euler Angle (rad)');
legend('\phi','\theta','\psi','\phi_{des}','\theta_{des}','Location','eastoutside');
set(gca,'LineWidth',0.75','FontSize',12);
xlim(x_range);
ylim(angle_range);
set(gca,'XTick',x_range(1):tick_space:x_range(2));
set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));

% 
% 
% subplot(2,3,6);
% plot(ttotal,Xtotal(:,4),'r-','LineWidth',2);
% hold on;
% plot(ttotal,Xtotal(:,5),'b--','LineWidth',2);
% plot(ttotal,Xtotal(:,6),'m-.','LineWidth',2);
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% legend('p','q','r','Location','northwest');
% set(gca,'LineWidth',0.75','FontSize',12);
% xlim(x_range);
% ylim(w_range);
% set(gca,'XTick',x_range(1):tick_space:x_range(2));
% set(gca,'XTickLabel',sprintf('%3.1f|',x_range(1):tick_space:x_range(2)));

