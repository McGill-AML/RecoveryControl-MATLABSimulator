%% plot script for plotting RMSE

plot_SPKF = 1;
plot_ASPKF = 1;
plot_EKF = 1;
plot_AEKF = 0;
plot_COMP = 1;
plot_HINF = 1;
plot_SPKF_full = 1;
plot_EKF_att = 1;


%% plot quaternion crash RMSE


figure;
hold on;
if plot_SPKF == 1
    plot(mean(PlotRMSE.crash.SPKF_quat),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(PlotRMSE.crash.ASPKF_quat),'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(mean(PlotRMSE.crash.COMP_quat),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(PlotRMSE.crash.HINF_quat),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(PlotRMSE.crash.SPKF_full_quat),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(PlotRMSE.crash.EKF_att_quat),'Linewidth',line_width, 'Color', 'k');
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE during crash');


%% plot quaternion not crashing RMSE


figure;
hold on;
if plot_SPKF == 1
    plot(mean(PlotRMSE.not_crash.SPKF_quat),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(PlotRMSE.not_crash.ASPKF_quat),'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(mean(PlotRMSE.not_crash.COMP_quat),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(PlotRMSE.not_crash.HINF_quat),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(PlotRMSE.not_crash.SPKF_full_quat),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(PlotRMSE.not_crash.EKF_att_quat),'Linewidth',line_width, 'Color', 'k');
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE not crash');


%% plot quaternion total RMSE


figure;
hold on;
if plot_SPKF == 1
    plot(mean(PlotRMSE.total.SPKF_quat),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(PlotRMSE.total.ASPKF_quat),'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(mean(PlotRMSE.total.COMP_quat),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(PlotRMSE.total.HINF_quat),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(PlotRMSE.total.SPKF_full_quat),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(PlotRMSE.total.EKF_att_quat),'Linewidth',line_width, 'Color', 'k');
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE total');


%% plot total gyr bias rmse
figure;
hold on;
if plot_SPKF == 1
    plot(mean(PlotRMSE.total.SPKF_gyr_bias),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(PlotRMSE.total.ASPKF_gyr_bias),'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(mean(PlotRMSE.total.COMP_gyr_bias),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(PlotRMSE.total.HINF_gyr_bias),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(PlotRMSE.total.SPKF_full_gyr_bias),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(PlotRMSE.total.EKF_att_gyr_bias),'Linewidth',line_width, 'Color', 'k');
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('bias RMSE during crash');
legend('SPKF', 'ASPKF', 'Comp', 'H-Inf', 'SPKF Full', 'EKF');