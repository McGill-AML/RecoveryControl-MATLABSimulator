%% plot script for plotting RMSE

plot_SPKF =1;
plot_ASPKF =1;
plot_EKF = 0;
plot_AEKF = 0;
plot_COMP =1;
plot_HINF = 1;
plot_SPKF_full =0;
plot_EKF_att =1;
plot_ASPKF_opt = 1;
plot_AHINF = 1;
plot_SPKF_norm =1;
plot_SRSPKF =0;

plot_gyr_bias = 0;

% separate indices to plot based on the longest time for which vicon
% loses tracking. then choose to either plot the long drops or the data
% with no drops

% plotLargeVicDrop = [];
% plotSmallVicDrop = [];
% 
% DontPlotIndices = [41, 10,36]; 59 and 66, 78 maybe? % you can take out indices with bad vicon data, etc. , for sure 41, 66, 59
% for ii = 1:length(maxDropLength)
%     if any(ii == DontPlotIndices)
%         %do nothing
%     elseif maxDropLength(ii) > 60
%         plotLargeVicDrop = [plotLargeVicDrop, ii];
%     elseif maxDropLength(ii) > 0 %dont plot indices we dont have data for
%         plotSmallVicDrop = [plotSmallVicDrop, ii];
%     end
% end
% 
% plotThese = plotSmallVicDrop; % set which indices to plot
% % plotThese = plotLargeVicDrop;
% plotThese(plotThese > length(rmseEUL.crash.SPKF_eul(1,:))) = []; %remove any indices larger than amount of data

plotThese = 1:length(rmse.total.EKF_att_quat);


%% plot quaternion crash RMSE


figure;
subplot(3,1,1)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.crash.SPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.crash.ASPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.crash.COMP_eul(1,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.crash.HINF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.crash.SPKF_full_eul(1,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.crash.EKF_att_eul(1,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.crash.ASPKF_opt_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.crash.AHINF_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.crash.SPKF_norm_eul(1,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.crash.SRSPKF_eul(1,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Yaw RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE during crash');

subplot(3,1,2)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.crash.SPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.crash.ASPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.crash.COMP_eul(2,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.crash.HINF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.crash.SPKF_full_eul(2,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.crash.EKF_att_eul(2,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.crash.ASPKF_opt_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.crash.AHINF_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.crash.SPKF_norm_eul(2,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.crash.SRSPKF_eul(2,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Pitch RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;


subplot(3,1,3)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.crash.SPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.crash.ASPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.crash.COMP_eul(3,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.crash.HINF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.crash.SPKF_full_eul(3,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.crash.EKF_att_eul(3,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.crash.ASPKF_opt_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.crash.AHINF_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.crash.SPKF_norm_eul(3,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.crash.SRSPKF_eul(3,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Roll RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;

%% plot quaternion not crashing RMSE


figure;
subplot(3,1,1)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.pre_crash.COMP_eul(1,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.pre_crash.HINF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_full_eul(1,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.pre_crash.EKF_att_eul(1,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_opt_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.pre_crash.AHINF_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_norm_eul(1,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SRSPKF_eul(1,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Yaw RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE pre crash');


subplot(3,1,2)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.pre_crash.COMP_eul(2,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.pre_crash.HINF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_full_eul(2,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.pre_crash.EKF_att_eul(2,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_opt_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.pre_crash.AHINF_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_norm_eul(2,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SRSPKF_eul(2,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Pitch RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;


subplot(3,1,3)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.pre_crash.COMP_eul(3,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.pre_crash.HINF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_full_eul(3,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.pre_crash.EKF_att_eul(3,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.pre_crash.ASPKF_opt_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.pre_crash.AHINF_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.pre_crash.SPKF_norm_eul(3,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.pre_crash.SRSPKF_eul(3,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Roll RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
%% plot quaternion not crashing RMSE


figure;
subplot(3,1,1)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.post_crash.COMP_eul(1,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.post_crash.HINF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_full_eul(1,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.post_crash.EKF_att_eul(1,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_opt_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.post_crash.AHINF_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_norm_eul(1,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.post_crash.SRSPKF_eul(1,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Yaw RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE post crash');


subplot(3,1,2)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.post_crash.COMP_eul(2,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.post_crash.HINF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_full_eul(2,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.post_crash.EKF_att_eul(2,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_opt_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.post_crash.AHINF_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_norm_eul(2,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.post_crash.SRSPKF_eul(2,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Pitch RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;


subplot(3,1,3)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.post_crash.COMP_eul(3,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.post_crash.HINF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_full_eul(3,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.post_crash.EKF_att_eul(3,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.post_crash.ASPKF_opt_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.post_crash.AHINF_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.post_crash.SPKF_norm_eul(3,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.post_crash.SRSPKF_eul(3,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Roll RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;

%% plot quaternion total RMSE


figure;

subplot(3,1,1)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.total.SPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.total.ASPKF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.total.COMP_eul(1,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.total.HINF_eul(1,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.total.SPKF_full_eul(1,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.total.EKF_att_eul(1,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.total.ASPKF_opt_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.total.AHINF_eul(1,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.total.SPKF_norm_eul(1,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.total.SRSPKF_eul(1,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Yaw RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('Quaternion RMSE total');

subplot(3,1,2)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.total.SPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.total.ASPKF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.total.COMP_eul(2,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.total.HINF_eul(2,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.total.SPKF_full_eul(2,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.total.EKF_att_eul(2,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.total.ASPKF_opt_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.total.AHINF_eul(2,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.total.SPKF_norm_eul(2,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.total.SRSPKF_eul(2,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Pitch RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;

subplot(3,1,3)
hold on;
if plot_SPKF == 1
    plot(plotThese,rmseEUL.total.SPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(plotThese,rmseEUL.total.ASPKF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(plotThese,rmseEUL.total.COMP_eul(3,plotThese),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(plotThese,rmseEUL.total.HINF_eul(3,plotThese),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(plotThese,rmseEUL.total.SPKF_full_eul(3,plotThese),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(plotThese,rmseEUL.total.EKF_att_eul(3,plotThese),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(plotThese,rmseEUL.total.ASPKF_opt_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(plotThese,rmseEUL.total.AHINF_eul(3,plotThese),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(plotThese,rmseEUL.total.SPKF_norm_eul(3,plotThese),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(plotThese,rmseEUL.total.SRSPKF_eul(3,plotThese),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('Roll RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;


%% plot total gyr bias rmseEUL
 if plot_gyr_bias == 1

figure;
subplot(3,1,3)
hold on;
if plot_SPKF == 1
    plot(mean(plotThese,rmseEUL.total.SPKF_gyr_bias),'Linewidth',line_width, 'Color', 'r');
end
if plot_ASPKF == 1
    plot(mean(plotThese,rmseEUL.total.ASPKF_gyr_bias),'Linewidth',line_width, 'Color', 'g');
end
if plot_COMP == 1
     plot(mean(plotThese,rmseEUL.total.COMP_gyr_bias),'Linewidth',line_width, 'Color', 'm');
end
if plot_HINF == 1
     plot(mean(plotThese,rmseEUL.total.HINF_gyr_bias),'Linewidth',line_width, 'Color', 'g');
end
if plot_SPKF_full == 1
    plot(mean(plotThese,rmseEUL.total.SPKF_full_gyr_bias),'Linewidth',line_width, 'Color', 'c');
end
if plot_EKF_att == 1
    plot(mean(plotThese,rmseEUL.total.EKF_att_gyr_bias),'Linewidth',line_width, 'Color', 'k');
end
if plot_ASPKF_opt == 1
    plot(mean(plotThese,rmseEUL.total.ASPKF_opt_gyr_bias),'g--','Linewidth',line_width);
end
if plot_AHINF == 1
     plot(mean(plotThese,rmseEUL.total.AHINF_gyr_bias),'g--','Linewidth',line_width);
end
if plot_SPKF_norm == 1
    plot(mean(plotThese,rmseEUL.total.SPKF_norm_gyr_bias),'r:','Linewidth',line_width);
end
if plot_SRSPKF == 1
    plot(mean(plotThese,rmseEUL.total.SRSPKF_gyr_bias),'r--', 'Linewidth',line_width);
end
xlabel('Run No','fontsize',font_size,'Interpreter','latex');
ylabel('RMSE','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on;
title('bias RMSE during crash');
% legend('H-Inf', 'EKF', 'AH-Inf', 'H-Inf', 'SPKF Full', 'EKF', 'Opt ASPKF', 'AHINF');

 legend('SPKF', 'ASPKF', 'Opt ASPKF', 'H-Inf', 'SPKF Full', 'EKF', 'Opt ASPKF', 'AHINF');
 end
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
