function [] = Graphs_Contact(ttotal,Xtotal,wall_loc,t_tot,defl_tot,Fc_tot,dX_tot,defl_rate_tot,flag_c_tot,vi_c_tot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global Rbumper
    
    figure();
    ax = subplot(3,1,1);
    plot(t_tot,defl_tot*100,t_tot,flag_c_tot)
    hold on
%     plot(ttotal,Xtotal(:,7)+Rbumper-wall_loc)
    title('Deflection');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl = ylim;
    ylim(ax,[0 yl(2)]); 
%     axis([ax.XLim 0 ax.YLim(2)]);
    grid on;
    
    subplot(3,1,2);
    plot(t_tot,defl_rate_tot);
    title('Deflection Rate');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    grid on;
    
    accel_tot = sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    subplot(3,1,3);
    plot(t_tot,Fc_tot,t_tot,accel_tot);
    title('Contact Force Magnitude');
    legend('Contact Force','Body Accel Mag');
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;
    
    figure()
    plot(t_tot,dX_tot(:,1),t_tot,dX_tot(:,2),t_tot,dX_tot(:,3))
    title('Body Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('x^B axis','y^B axis','z^B axis');
    grid on;

end

