% Plot scripts

plot_SPKF = 1;
plot_ASPKF = 1;
plot_EKF = 1;
plot_AEKF = 1;

%% Position
figure
subplot(3,1,1)
plot(Plot.times,Plot.posns(1,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_pos(1,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
     plot(Plot.times,Plot.AEKF_pos(1,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.posns(2,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_pos(2,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
    plot(Plot.times,Plot.AEKF_pos(2,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.posns(3,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_pos(3,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
    plot(Plot.times,Plot.AEKF_pos(3,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$r_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% covariance
figure;plot(Plot.times, Plot.SPKF_P_hat);grid on;
legend('w_x','w_y','w_z','b_{gyr_x}','b_{gyr_y}','b_{gyr_z}');
title('SPKF covariances');

%% Velocity

figure
subplot(3,1,1)
plot(Plot.times,Plot.linVels(1,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_vel(1,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
    plot(Plot.times,Plot.AEKF_vel(1,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.linVels(2,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_vel(2,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
    plot(Plot.times,Plot.AEKF_vel(2,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.linVels(3,:),'Linewidth',line_width);
hold on
if plot_EKF == 1
    plot(Plot.times,Plot.EKF_vel(3,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_AEKF == 1
    plot(Plot.times,Plot.AEKF_vel(3,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$v_{b3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% quaternion

figure
subplot(4,1,1)
plot(Plot.times,Plot.quaternions(1,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_quat(1,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_quat(1,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_0$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,2)
plot(Plot.times,Plot.quaternions(2,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_quat(2,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_quat(2,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_x$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,3)
plot(Plot.times,Plot.quaternions(3,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_quat(3,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_quat(3,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_y$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(4,1,4)
plot(Plot.times,Plot.quaternions(4,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_quat(4,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_quat(4,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_z$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%%  plot ASPKF gain

figure;plot(Plot.times, Plot.ASPKF_G_k);grid on;
title('ASPKF gain');

%% plot ASPKF innov

figure;plot(Plot.times, Plot.ASPKF_innov);grid on;
title('ASPKF innovation sum');

%% angular velocity
figure
subplot(3,1,1)
plot(Plot.times,Plot.angVels(1,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_omega(1,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_omega(1,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.angVels(2,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
    plot(Plot.times,Plot.SPKF_omega(2,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_omega(2,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.angVels(3,:),'Linewidth',line_width);
hold on
if plot_SPKF == 1
     plot(Plot.times,Plot.SPKF_omega(3,:),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(Plot.times,Plot.ASPKF_omega(3,:),'Linewidth',line_width, 'Color', 'y');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


%% Euler angles

figure;
plot(Plot.times, Plot.eulerAngles);grid on;
title('Euler angles');
legend('theta','phi','psi');


%% Sensors

figure
subplot(3,1,1)
plot(Plot.times,Plot.bodyAccs(1,:),'Linewidth',line_width);
hold on
plot(Plot.times,Plot.accelerometers(1,:),'Linewidth',line_width, 'Color', 'r');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b1}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,2)
plot(Plot.times,Plot.bodyAccs(2,:),'Linewidth',line_width);
hold on
plot(Plot.times,Plot.accelerometers(2,:),'Linewidth',line_width,'Color', 'r');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b2}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
subplot(3,1,3)
plot(Plot.times,Plot.bodyAccs(3,:),'Linewidth',line_width);
hold on
plot(Plot.times,Plot.accelerometers(3,:),'Linewidth',line_width,'Color', 'r');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$a_{b3}$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
title('Accelerometer / body accs');
legend('body acc','acceleroms');

%% crash occured
figure;plot(Plot.times, Plot.crash);grid on;
title('crash occurs for sensor model');



%% animate

animate(0,Hist,'XZ',ImpactParams,timeImpact,'' )

