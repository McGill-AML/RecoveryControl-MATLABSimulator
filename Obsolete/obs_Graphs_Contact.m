function [] = Graphs_Contact(ttotal,Xtotal,dX_tot,wall_loc,t_tot,defl1_tot,Fc1_tot,defl_rate1_tot,flag_c1_tot,vi_c1_tot,defl2_tot,Fc2_tot,defl_rate2_tot,flag_c2_tot,vi_c2_tot,defl3_tot,Fc3_tot,defl_rate3_tot,flag_c3_tot,vi_c3_tot,defl4_tot,Fc4_tot,defl_rate4_tot,flag_c4_tot,vi_c4_tot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global Rbumper m
    
    figure('Position',[10,400,2000,1000]);
%%---------------------------------------------------------------------------------%%    
    defl1 = subplot(3,4,1);
    plot(t_tot,defl1_tot*100,t_tot,flag_c1_tot)
    hold on
    title('Deflection - Bumper 1');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl1 = ylim;
    grid on;
%%---------------------------------------------------------------------------------%%    
    deflrate1 = subplot(3,4,5);
    plot(t_tot,defl_rate1_tot);
    title('Deflection Rate - Bumper 1');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate1 = ylim;
    grid on;
%%---------------------------------------------------------------------------------%%    
    accel_tot = m*sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force1 = subplot(3,4,9);
    plot(t_tot,accel_tot);
    hold on;
    plot(t_tot,Fc1_tot,'r-','LineWidth',1);      
    title('Contact Force Magnitude - Bumper 1');
    legend('m*(Body Accel Mag)','Contact Force 1');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force1 = ylim;
    grid on;
%%---------------------------------------------------------------------------------%%    
    defl2 = subplot(3,4,2);
    plot(t_tot,defl2_tot*100,t_tot,flag_c2_tot)
    hold on
    title('Deflection - Bumper 2');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl2 = ylim;
    ylim(defl2,[0, max([yl_defl1(2),yl_defl2(2)])]); 
    grid on;    
    ylim(defl1,[0, max([yl_defl1(2),yl_defl2(2)])]); 
%%---------------------------------------------------------------------------------%%    
    deflrate2 = subplot(3,4,6);
    plot(t_tot,defl_rate2_tot);
    title('Deflection Rate - Bumper 2');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate2 = ylim;
    ylim(deflrate2,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
    grid on;
    ylim(deflrate1,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
%%---------------------------------------------------------------------------------%%    
    accel_tot = sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force2 = subplot(3,4,10);
    plot(t_tot,accel_tot);
    hold on;
    plot(t_tot,Fc2_tot,'r-','LineWidth',1);
    title('Contact Force Magnitude - Bumper 2');
    legend('m*(Body Accel Mag)','Contact Force 2');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force2 = ylim;
    ylim(force2,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]); 
    grid on;
    ylim(force1,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]);   
    
%%---------------------------------------------------------------------------------%%    
    defl3 = subplot(3,4,3);
    plot(t_tot,defl3_tot*100,t_tot,flag_c3_tot)
    hold on
    title('Deflection - Bumper 3');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl3 = ylim;
%     ylim(defl3,[0, max([yl_defl1(2),yl_defl2(2)])]); 
    grid on;    
%     ylim(defl1,[0, max([yl_defl1(2),yl_defl2(2)])]); 
%%---------------------------------------------------------------------------------%%    
    deflrate3 = subplot(3,4,7);
    plot(t_tot,defl_rate3_tot);
    title('Deflection Rate - Bumper 3');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate3 = ylim;
%     ylim(deflrate2,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
    grid on;
%     ylim(deflrate1,[min([yl_deflrate1(1),yl_deflrate2(1)]), max([yl_deflrate1(2),yl_deflrate2(2)])]); 
%%---------------------------------------------------------------------------------%%    
    accel_tot = sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force3 = subplot(3,4,11);
    plot(t_tot,accel_tot);
    hold on;
    plot(t_tot,Fc3_tot,'r-','LineWidth',1);    
    title('Contact Force Magnitude - Bumper 3');
    legend('m*(Body Accel Mag)','Contact Force 3');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force3 = ylim;
%     ylim(force2,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]); 
    grid on;
%     ylim(force1,[min([yl_force1(1),yl_force2(1)]), max([yl_force1(2),yl_force2(2)])]);  
    
%%---------------------------------------------------------------------------------%%    
    defl4 = subplot(3,4,4);
    plot(t_tot,defl4_tot*100,t_tot,flag_c4_tot)
    hold on
    title('Deflection - Bumper 4');
    xlabel('Time (s)');
    ylabel('\delta (cm)');
    legend('CT Defl');
    yl_defl4 = ylim;
    ylim(defl4,[0, max([yl_defl1(2),yl_defl2(2),yl_defl3(2),yl_defl4(2)])]); 
    grid on;    
    ylim(defl1,[0, max([yl_defl1(2),yl_defl2(2),yl_defl3(2),yl_defl4(2)])]); 
    ylim(defl2,[0, max([yl_defl1(2),yl_defl2(2),yl_defl3(2),yl_defl4(2)])]); 
    ylim(defl3,[0, max([yl_defl1(2),yl_defl2(2),yl_defl3(2),yl_defl4(2)])]); 
%%---------------------------------------------------------------------------------%%    
    deflrate4 = subplot(3,4,8);
    plot(t_tot,defl_rate4_tot);
    title('Deflection Rate - Bumper 4');
    xlabel('Time(s)');
    ylabel('Rate (m/s)');
    legend('Defl Rate');
    yl_deflrate4 = ylim;
    ylim(deflrate4,[min([yl_deflrate1(1),yl_deflrate2(1),yl_deflrate3(1),yl_deflrate4(1)]), max([yl_deflrate1(2),yl_deflrate2(2),yl_deflrate3(2),yl_deflrate4(2)])]); 
    grid on;
    ylim(deflrate1,[min([yl_deflrate1(1),yl_deflrate2(1),yl_deflrate3(1),yl_deflrate4(1)]), max([yl_deflrate1(2),yl_deflrate2(2),yl_deflrate3(2),yl_deflrate4(2)])]); 
    ylim(deflrate2,[min([yl_deflrate1(1),yl_deflrate2(1),yl_deflrate3(1),yl_deflrate4(1)]), max([yl_deflrate1(2),yl_deflrate2(2),yl_deflrate3(2),yl_deflrate4(2)])]); 
    ylim(deflrate3,[min([yl_deflrate1(1),yl_deflrate2(1),yl_deflrate3(1),yl_deflrate4(1)]), max([yl_deflrate1(2),yl_deflrate2(2),yl_deflrate3(2),yl_deflrate4(2)])]); 

%%---------------------------------------------------------------------------------%%    
    accel_tot = sqrt(sum(dX_tot(:,1).^2 + dX_tot(:,2).^2 + dX_tot(:,3).^2,2));
    
    force4 = subplot(3,4,12);
    plot(t_tot,accel_tot);
    hold on;
    plot(t_tot,Fc4_tot,'r-','LineWidth',1);
    title('Contact Force Magnitude - Bumper 4');
    legend('m*(Body Accel Mag)','Contact Force 4');
    xlabel('Time (s)');
    ylabel('Force (N)');
    yl_force4 = ylim;
    ylim(force2,[min([yl_force1(1),yl_force2(1),yl_force3(1),yl_force4(1)]), max([yl_force1(2),yl_force2(2),yl_force3(2),yl_force4(2)])]); 
    grid on;
    ylim(force1,[min([yl_force1(1),yl_force2(1),yl_force3(1),yl_force4(1)]), max([yl_force1(2),yl_force2(2),yl_force3(2),yl_force4(2)])]); 
    ylim(force2,[min([yl_force1(1),yl_force2(1),yl_force3(1),yl_force4(1)]), max([yl_force1(2),yl_force2(2),yl_force3(2),yl_force4(2)])]); 
    ylim(force3,[min([yl_force1(1),yl_force2(1),yl_force3(1),yl_force4(1)]), max([yl_force1(2),yl_force2(2),yl_force3(2),yl_force4(2)])]); 

%%---------------------------------------------------------------------------------%%
%%---------------------------------------------------------------------------------%%   
    figure('Position',[500,10,1000,500])
    plot(t_tot,accel_tot,'-','Color',[102 250 120]/256,'LineWidth',2);
    hold on;
    
    plot(t_tot,Fc1_tot,t_tot,Fc2_tot,t_tot,Fc3_tot,t_tot,Fc4_tot);
    plot(t_tot,Fc1_tot + Fc2_tot + Fc3_tot + Fc4_tot,'m--');
    
    
    legend('m*(Body Accel Mag)','Bumper 1','Bumper 2','Bumper 3','Bumper 4','Total Contact Force');
    title('Contact Forces');
    ylabel('Force (N)');
    xlabel('Time (s)');
    grid on;

    
%%---------------------------------------------------------------------------------%%
%%---------------------------------------------------------------------------------%%   
    figure('Position',[10,400,1000,500])
    subplot(1,2,1);
    plot(t_tot,dX_tot(:,1),t_tot,dX_tot(:,2),t_tot,dX_tot(:,3))
    title('Body Linear Accelerations');
    xlabel('Time (s)');
    ylabel('Linear Acceleration (m/s^2)');
    legend('x^B axis','y^B axis','z^B axis');
    grid on;
    
    subplot(1,2,2);
    plot(t_tot,dX_tot(:,4),t_tot,dX_tot(:,5),t_tot,dX_tot(:,6))
    title('Body Angular Accelerations');
    xlabel('Time (s)');
    ylabel('Angular Acceleration (rad/s^2)');
    legend('x^B axis','y^B axis','z^B axis');
    grid on;
end

