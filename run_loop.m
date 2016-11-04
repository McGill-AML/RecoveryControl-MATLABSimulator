% loop to test variables

for loop_no = 2
    SPKF.kappa = (loop_no-3)*2;
    ASPKF.kappa = (loop_no-3)*2;
    
    startsim_trajectory;
    
    plot_SPKF = 0;
    plot_ASPKF = 0;
    plot_EKF = 1;
    plot_AEKF = 1;
    plot_COMP = 1;
    plot_HINF = 1;


    
    
    
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
if plot_COMP == 1
     plot(Plot.times,Plot.COMP_quat(1,:),'Linewidth',line_width, 'Color', 'b');
end
if plot_HINF == 1
     plot(Plot.times,Plot.HINF_quat(1,:),'Linewidth',line_width, 'Color', 'g');
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
     plot(Plot.times,Plot.COMP_quat(2,:),'Linewidth',line_width, 'Color', 'b');
end
if plot_HINF == 1
     plot(Plot.times,Plot.HINF_quat(2,:),'Linewidth',line_width, 'Color', 'g');
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
     plot(Plot.times,Plot.COMP_quat(3,:),'Linewidth',line_width, 'Color', 'b');
end
if plot_HINF == 1
     plot(Plot.times,Plot.HINF_quat(3,:),'Linewidth',line_width, 'Color', 'g');
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
     plot(Plot.times,Plot.COMP_quat(4,:),'Linewidth',line_width, 'Color', 'b');
end
if plot_HINF == 1
     plot(Plot.times,Plot.HINF_quat(4,:),'Linewidth',line_width, 'Color', 'g');
end
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('$q_z$ (m)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on


    
end