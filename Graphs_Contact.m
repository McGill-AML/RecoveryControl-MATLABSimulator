function [] = Graphs_Contact(ttotal,Xtotal,dX_tot,wall_loc,t_tot,defl1_tot,Fc1_tot,defl_rate1_tot,flag_c1_tot,vi_c1_tot,defl2_tot,Fc2_tot,defl_rate2_tot,flag_c2_tot,vi_c2_tot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global Rbumper m
    
    figure('Position',[10,400,1500,1000]);
    defl1 = subplot(3,2,1);
    plot(t_tot,defl1_tot*100,t_tot,flag_c1_tot)
    hold on
    title('Deflection - Pt 1');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl1 = ylim;
    grid on;
    
    deflrate1 = subplot(3,2,3);
    plot(t_tot,defl_rate1_tot);
    title('Deflection Rate - Pt 1');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate1 = ylim;
    grid on;
    
    accel_tot = m*sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force1 = subplot(3,2,5);
    plot(t_tot,Fc1_tot,t_tot,accel_tot,t_tot,Fc1_tot+Fc2_tot);
    title('Contact Force Magnitude - Pt 1');
    legend('Contact Force 1','m*(Body Accel Mag)','Total Contact Force');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force1 = ylim;
    grid on;
    
    defl2 = subplot(3,2,2);
    plot(t_tot,defl2_tot*100,t_tot,flag_c2_tot)
    hold on
    title('Deflection - Pt 2');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl2 = ylim;
    ylim(defl2,[0, max([yl_defl1(2),yl_defl2(2)])]); 
    grid on;    
    ylim(defl1,[0, max([yl_defl1(2),yl_defl2(2)])]); 
    
    deflrate2 = subplot(3,2,4);
    plot(t_tot,defl_rate2_tot);
    title('Deflection Rate - Pt 2');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate2 = ylim;
    ylim(deflrate2,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
    grid on;
    ylim(deflrate1,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
    
    accel_tot = sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force2 = subplot(3,2,6);
    plot(t_tot,Fc2_tot,t_tot,accel_tot,t_tot,Fc1_tot+Fc2_tot);
    title('Contact Force Magnitude - Pt 2');
    legend('Contact Force 2','m*(Body Accel Mag)','Total Contact Force');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force2 = ylim;
    ylim(force2,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]); 
    grid on;
    ylim(force1,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]);    
    
    
    
    
    figure();
    plot(t_tot,dX_tot(:,1),t_tot,dX_tot(:,2),t_tot,dX_tot(:,3))
    title('Body Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('x^B axis','y^B axis','z^B axis');
    grid on;

end

