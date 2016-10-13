% Plot scripts
font_size = 15;
line_size = 15;
line_width = 2;


%% Position
figure
subplot(3,1,1)
plot(Plot.times,Plot.posns(1,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.posns(2,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.posns(3,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% Velocity

figure
subplot(3,1,1)
plot(Plot.times,Plot.linVels(1,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.linVels(2,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.linVels(3,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
title('Body velocities')

%% acceleration

figure
subplot(3,1,1)
plot(Plot.times,Plot.bodyAccs(1,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.bodyAccs(2,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.bodyAccs(3,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
title('Body accelerations');


%% quaternion

figure
subplot(4,1,1)
plot(Plot.times,Plot.quaternions(1,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_0$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,2)
plot(Plot.times,Plot.quaternions(2,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_x$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,3)
plot(Plot.times,Plot.quaternions(3,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_y$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,4)
plot(Plot.times,Plot.quaternions(4,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_z$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% angular velocity
figure
subplot(3,1,1)
plot(Plot.times,Plot.angVels(1,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.angVels(2,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.angVels(3,:),'Linewidth',line_width);
hold on
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% animate

animate(0,Hist,'XZ',ImpactParams,timeImpact,'' )

