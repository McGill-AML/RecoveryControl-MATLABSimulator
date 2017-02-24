% % % loop to test variables
% clear;
% 
timer = zeros(11,1);

angle_head = 1;
posn_hit = 0;

rmseEUL = [];

for loop_no = 1
    loop_no
    clearvars -except loop_no timer rmse angle_head posn_hit rmse_position rmseEUL
    


    angle_head = pi/3; %rand*pi;
    posn_hit =  4.25;

    
    
    % set waypoints for run. 
    Setpoint = initsetpoint;
    
    
    % Setpt 1
    Setpoint.head = angle_head;
    Setpoint.time = 5;
    Setpoint.posn = [-0.15;0;0.5];
    Trajectory = Setpoint;
    
        % Setpt 1
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 5;
    Setpoint.posn = [-0.15;0;5];
    Trajectory = [Trajectory;Setpoint];
    
%     % Setpt 1
%     Setpoint.head = angle_head;
%     Setpoint.time = 60;
%     Setpoint.posn = [-0.15;0;5];
%     Trajectory = [Trajectory;Setpoint];
%     
% %     Setpt 2 - crash
%     Setpoint.head = angle_head;
%     Setpoint.time = Setpoint.time + 10;
%     Setpoint.posn = [posn_hit;0;5];
%     Trajectory = [Trajectory;Setpoint];
%     
%     Setpt 2 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 10;
    Setpoint.posn = [-5.15;0;5];
    Trajectory = [Trajectory;Setpoint];
    
%     Setpt 3 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 10;
    Setpoint.posn = [-5.15;-5;5];
    Trajectory = [Trajectory;Setpoint];
    
%     Setpt 4 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 6;
    Setpoint.posn = [-1.5;-5; 5];
    Trajectory = [Trajectory;Setpoint];
    
%     Setpt 5 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 2;
    Setpoint.posn = [-1.5;-5; 5];
    Trajectory = [Trajectory;Setpoint];
    
%     Setpt 5 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 2;
    Setpoint.posn = [-0.4;-5;5];
    Trajectory = [Trajectory;Setpoint];
    
%     Setpt 5 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 10;
    Setpoint.posn = [-0.4;0;5];
    Trajectory = [Trajectory;Setpoint];
    
% %     Setpt 5 - square
%     Setpoint.head = angle_head;
%     Setpoint.time = Setpoint.time + 5;
%     Setpoint.posn = [-0.6;0;1.5];
%     Trajectory = [Trajectory;Setpoint];
%     
% %     Setpt 5 - square
%     Setpoint.head = angle_head;
%     Setpoint.time = Setpoint.time + 5;
%     Setpoint.posn = [-0.6;0;0.6];
%     Trajectory = [Trajectory;Setpoint];
    
%     Setpt 2 - crash
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 10;
    Setpoint.posn = [posn_hit;0;5];
    Trajectory = [Trajectory;Setpoint];
%     
    startsim_trajectory;
    
  
    if ~exist('time_of_recovery','var');
        time_of_recovery = iSim;
    end
    rmse(loop_no,:) = rmse_att(Plot,sensParams,time_of_recovery/tStep);
    
%     rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);

    
    rmseEUL = rmse_att_euler(Plot, sensParams, time_of_recovery/tStep, rmseEUL, loop_no);
    

%     save('upd_crsh_low','rmse','rmseEUL');
    



    
end
% 
% save('upd_crsh_low_all');

PlotRMSE = RMSE_to_plot(rmse);

