% Plot scripts

plot_SPKF = 1;
plot_ASPKF = 1;
plot_EKF = 1;
plot_AEKF = 0;
plot_COMP = 1;
plot_HINF = 0;
plot_SPKF_full = 1;
plot_EKF_att = 1;

plot_pos =0;
plot_cov = 0;
plot_vel = 0;
plot_quat = 0;
plot_ASPKF_stuff = 0;
plot_angvel = 0;
plot_eul_ang = 0;
plot_acceleroms = 0;
plot_gyros = 0;
plot_mag = 0;
plot_crash_occur = 0;
plot_gyro_bias = 1;
plot_accel_bias = 0;
plot_gps = 0;
plot_baro = 0;
plot_animate = 0;

%% Position
if plot_pos == 1
    figure
    title(['Position, run no', num2str(loop_no)]);
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
    
end

%% covariance
if plot_cov == 1
    figure;plot(Plot.times, Plot.SPKF_P_hat);grid on;
    legend('w_x','w_y','w_z','b_{gyr_x}','b_{gyr_y}','b_{gyr_z}');
    title(['SPKF covariances, run no', num2str(loop_no)]);
end

%% Velocity
if plot_vel == 1
    title(['Linear velocity, run no', num2str(loop_no)]);
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
    
end

%% quaternion
if plot_quat == 1
    
    figure
    title(['Quat estimate, run no', num2str(loop_no)]);
    subplot(4,1,1)
    plot(Plot.times,Plot.quaternions(1,:),'Linewidth',line_width);
    hold on
    if plot_SPKF == 1
        plot(Plot.times,Plot.SPKF_quat(1,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_ASPKF == 1
        plot(Plot.times,Plot.ASPKF_quat(1,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_quat(1,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_quat(1,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_quat(1,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_quat(1,:),'Linewidth',line_width, 'Color', 'k');
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
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_quat(2,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_quat(2,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_quat(2,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_quat(2,:),'Linewidth',line_width, 'Color', 'k');
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
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_quat(3,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_quat(3,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_quat(3,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_quat(3,:),'Linewidth',line_width, 'Color', 'k');
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
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_quat(4,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_quat(4,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_quat(4,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_quat(4,:),'Linewidth',line_width, 'Color', 'k');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$q_z$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    
end

%%  plot ASPKF gain

if plot_ASPKF_stuff == 1
    figure;plot(Plot.times, Plot.ASPKF_G_k);grid on;
    title(['ASPKF gain, run no', num2str(loop_no)]);
    
    % plot ASPKF innov
    
    figure;plot(Plot.times, Plot.ASPKF_innov);grid on;
    title(['ASPKF innovation sum, run no', num2str(loop_no)]);
    
end
%% angular velocity
if plot_angvel == 1
    
    figure
    subplot(3,1,1)
    title(['Ang vel, run no', num2str(loop_no)]);
    plot(Plot.times,Plot.angVels(1,:),'Linewidth',line_width);
    hold on
    if plot_SPKF == 1
        plot(Plot.times,Plot.SPKF_omega(1,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_ASPKF == 1
        plot(Plot.times,Plot.ASPKF_omega(1,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_omega(1,:),'Linewidth',line_width, 'Color', 'b');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_omega(1,:),'Linewidth',line_width, 'Color', 'g');
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
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_omega(2,:),'Linewidth',line_width, 'Color', 'b');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_omega(2,:),'Linewidth',line_width, 'Color', 'g');
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
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_omega(3,:),'Linewidth',line_width, 'Color', 'b');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_omega(3,:),'Linewidth',line_width, 'Color', 'g');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    
end

%% Euler angles

if plot_eul_ang == 1
    figure;
    plot(Plot.times, Plot.eulerAngles);grid on;
    title(['Euler angles, run no', num2str(loop_no)]);
    legend('theta','phi','psi');
end

%% Sensors
if plot_acceleroms ==1
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
    title(['Accelerometer / body accs, run no', num2str(loop_no)]);
    legend('body acc','acceleroms');
end

%% Sensor - gyro

if plot_gyros == 1
    figure
    title(['Gyroscope measurements, run no', num2str(loop_no)]);
    subplot(3,1,1)
    plot(Plot.times,Plot.angVels(1,:),'Linewidth',line_width);
    hold on
    plot(Plot.times,Plot.gyros(1,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,2)
    plot(Plot.times,Plot.angVels(2,:),'Linewidth',line_width);
    hold on
    plot(Plot.times,Plot.gyros(2,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,3)
    plot(Plot.times,Plot.angVels(3,:),'Linewidth',line_width);
    hold on
    plot(Plot.times,Plot.gyros(3,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    legend('\omega','Gyro reading');
end

%% magnetometers

if plot_mag == 1
    figure
    title(['Magnetometer measurements, run no', num2str(loop_no)]);
    subplot(3,1,1)
    
    plot(Plot.times,Plot.mag(1,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,2)
    
    plot(Plot.times,Plot.mag(2,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,3)
    
    plot(Plot.times,Plot.mag(3,:),'Linewidth',line_width, 'Color', 'r');
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
end

%% crash occured
if plot_crash_occur ==1
    figure;plot(Plot.times, Plot.crash);grid on;
    title(['crash occurs for sensor model, run no', num2str(loop_no)]);
    
    figure;plot(Plot.times, Plot.time_since_cont);
    title(['time since crash occurs for sensor model, run no', num2str(loop_no)]);
end

%% plot sensor bias estimates

if plot_gyro_bias ==1
    figure
%     title(['Gyroscope bias estimates, run no', num2str(loop_no)]);
    hax1 = subplot(3,1,1);
    title(hax1, 'Gyroscope bias estimate');
    plot(Plot.times,sensParams.bias.gyr(1)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_SPKF == 1
        plot(Plot.times,Plot.SPKF_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_ASPKF == 1
        plot(Plot.times,Plot.ASPKF_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_gyr_bias(1,:),'Linewidth',line_width, 'Color', 'k');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,2)
    plot(Plot.times,sensParams.bias.gyr(2)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_SPKF == 1
        plot(Plot.times,Plot.SPKF_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_ASPKF == 1
        plot(Plot.times,Plot.ASPKF_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_gyr_bias(2,:),'Linewidth',line_width, 'Color', 'k');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,3)
    plot(Plot.times,sensParams.bias.gyr(3)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_SPKF == 1
        plot(Plot.times,Plot.SPKF_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_ASPKF == 1
        plot(Plot.times,Plot.ASPKF_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_COMP == 1
        plot(Plot.times,Plot.COMP_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'm');
    end
    if plot_HINF == 1
        plot(Plot.times,Plot.HINF_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'g');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'c');
    end
    if plot_EKF_att == 1
        plot(Plot.times,Plot.EKF_att_gyr_bias(3,:),'Linewidth',line_width, 'Color', 'k');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$\omega_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    legend('true bias','SPKF', 'ASPKF', 'Comp', 'H-Inf', 'SPKF Full', 'EKF');
    
    
end

%% plot accel bias estimates
if plot_accel_bias == 1
    figure
%     title(['Accelerometer bias estimates, run no', num2str(loop_no)]);
    hax1 = subplot(3,1,1);
    title(hax1, 'Accelerometer bias estimate');
    plot(Plot.times,sensParams.bias.acc(1)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_EKF == 1
        plot(Plot.times,Plot.EKF_acc_bias(1,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_AEKF == 1
        plot(Plot.times,Plot.AEKF_acc_bias(1,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_acc_bias(1,:),'Linewidth',line_width, 'Color', 'c');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$a_{1}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,2)
    plot(Plot.times,sensParams.bias.acc(2)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_EKF == 1
        plot(Plot.times,Plot.EKF_acc_bias(2,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_AEKF == 1
        plot(Plot.times,Plot.AEKF_acc_bias(2,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_acc_bias(2,:),'Linewidth',line_width, 'Color', 'c');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$a_{2}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    subplot(3,1,3)
    plot(Plot.times,sensParams.bias.acc(3)*ones(length(Plot.times),1),'Linewidth',line_width);
    hold on
    if plot_EKF == 1
        plot(Plot.times,Plot.EKF_acc_bias(3,:),'Linewidth',line_width, 'Color', 'r');
    end
    if plot_AEKF == 1
        plot(Plot.times,Plot.AEKF_acc_bias(3,:),'Linewidth',line_width, 'Color', 'y');
    end
    if plot_SPKF_full == 1
        plot(Plot.times,Plot.SPKF_full_acc_bias(3,:),'Linewidth',line_width, 'Color', 'c');
    end
    xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
    ylabel('$a_{3}$ (m)','fontsize',font_size,'Interpreter','latex');
    set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
    grid on
    legend('True bias','EKF','SPKF full');
    
end


%% plot GPS coordinates
if plot_gps == 1
    figure;plot(Plot.times, Plot.gps);
    grid on;
    title(['all gps stuff, run no', num2str(loop_no)]);
end

%% plot GPS coordinates
if plot_baro == 1
    figure;plot(Plot.times, Plot.baro);
    grid on;
    title(['barometer, run no', num2str(loop_no)]);
end


%% animate
if plot_animate == 1
    animate(0,Hist,'XZ',ImpactParams,timeImpact,'' )
end
