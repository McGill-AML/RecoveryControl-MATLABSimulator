%% plot script for plotting RMSE

plot_SPKF = 1;
plot_ASPKF = 0;
plot_EKF = 1;
plot_AEKF = 0;
plot_COMP = 0;
plot_HINF =0;
plot_SPKF_full =1;
plot_EKF_att = 0;
plot_ASPKF_opt = 0;
plot_AHINF = 0;
plot_SPKF_norm = 0;


%% plot quaternion crash RMSE


figure;
hold on;
if plot_SPKF == 1
    plot(PlotRMSE.crash.SPKF_quat,'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(PlotRMSE.crash.ASPKF_quat,'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(PlotRMSE.crash.COMP_quat,'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(PlotRMSE.crash.HINF_quat,'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(PlotRMSE.crash.SPKF_full_quat,'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(PlotRMSE.crash.EKF_att_quat,'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(PlotRMSE.crash.ASPKF_opt_quat,'y--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(PlotRMSE.crash.AHINF_quat,'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(PlotRMSE.crash.SPKF_norm_quat,'r:','Linewidth',line_width);
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
    plot(PlotRMSE.not_crash.SPKF_quat,'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(PlotRMSE.not_crash.ASPKF_quat,'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(PlotRMSE.not_crash.COMP_quat,'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(PlotRMSE.not_crash.HINF_quat,'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(PlotRMSE.not_crash.SPKF_full_quat,'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(PlotRMSE.not_crash.EKF_att_quat,'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(PlotRMSE.not_crash.ASPKF_opt_quat,'y--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(PlotRMSE.not_crash.AHINF_quat,'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(PlotRMSE.not_crash.SPKF_norm_quat,'r:','Linewidth',line_width);
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
    plot(PlotRMSE.total.SPKF_quat,'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(PlotRMSE.total.ASPKF_quat,'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(PlotRMSE.total.COMP_quat,'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(PlotRMSE.total.HINF_quat,'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(PlotRMSE.total.SPKF_full_quat,'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(PlotRMSE.total.EKF_att_quat,'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(PlotRMSE.total.ASPKF_opt_quat,'y--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(PlotRMSE.total.AHINF_quat,'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(PlotRMSE.total.SPKF_norm_quat,'r:','Linewidth',line_width);
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
    plot(mean(PlotRMSE.crash.SPKF_gyr_bias),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(PlotRMSE.crash.ASPKF_gyr_bias),'Linewidth',line_width, 'Color', 'y');
end
if plot_COMP == 1
     plot(mean(PlotRMSE.crash.COMP_gyr_bias),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(PlotRMSE.crash.HINF_gyr_bias),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(PlotRMSE.crash.SPKF_full_gyr_bias),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(PlotRMSE.crash.EKF_att_gyr_bias),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(mean(PlotRMSE.crash.ASPKF_opt_gyr_bias),'y--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(mean(PlotRMSE.crash.AHINF_gyr_bias),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(mean(PlotRMSE.crash.SPKF_norm_gyr_bias),'r:','Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('bias RMSE during crash');
% legend('H-Inf', 'EKF', 'AH-Inf', 'H-Inf', 'SPKF Full', 'EKF', 'Opt ASPKF', 'AHINF');

 legend('SPKF', 'ASPKF', 'Opt ASPKF', 'H-Inf', 'SPKF Full', 'EKF', 'Opt ASPKF', 'AHINF');

%% used for computing total RMSE over all of the runs

% tmp = struct2cell(PlotRMSE);
% tmp2 = struct2cell(tmp{3});
% 
% for ii = 1:length(tmp2)/2
%     crash_RMSE(ii,1) = max(tmp2{ii});
% end
% 
% for ii = length(tmp2)/2+1:length(tmp2)
%     crash_RMSE(ii,1) = max(mean(tmp2{ii}));
% end
