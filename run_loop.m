% loop to test variables
% clear;

% timer = zeros(17,1);

angle_head = 1;
posn_hit = 0;

for loop_no = 297:300
    loop_no
    clearvars -except loop_no timer rmse angle_head posn_hit rmse_position
    


    angle_head = rand*pi;
    posn_hit = rand*3 + 2;

    
    
    % set waypoints for run. 
    Setpoint = initsetpoint;
    
    
    % Setpt 1
    Setpoint.head = angle_head;
    Setpoint.time = 0.5;
    Setpoint.posn = [-0.6;0;5];
    Trajectory = Setpoint;
    
    % Setpt 1
    Setpoint.head = angle_head;
    Setpoint.time = 4;
    Setpoint.posn = [-0.6;0;5];
    Trajectory = [Trajectory;Setpoint];
    
% %     Setpt 2 - crash
%     Setpoint.head = angle_head;
%     Setpoint.time = Setpoint.time + 10;
%     Setpoint.posn = [posn_hit;0;5];
%     Trajectory = [Trajectory;Setpoint];
    
    % Setpt 2 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 5;
    Setpoint.posn = [-1.6;0;5];
    Trajectory = [Trajectory;Setpoint];
    
    % Setpt 3 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 5;
    Setpoint.posn = [-1.6;-1;5];
    Trajectory = [Trajectory;Setpoint];
    
    % Setpt 4 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 5;
    Setpoint.posn = [-0.6;-1;5];
    Trajectory = [Trajectory;Setpoint];
    
    % Setpt 5 - square
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 5;
    Setpoint.posn = [-0.6;0;5];
    Trajectory = [Trajectory;Setpoint];
    
    startsim_trajectory;
    
  
    if ~exist('time_of_recovery','var');
        time_of_recovery = iSim;
    end
    rmse(loop_no,:) = rmse_att(Plot,sensParams,time_of_recovery/tStep);
    
%     rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);
    
    
    

    save('square_low_ICs','rmse');
    



    
end

save('square_low_ICs_all');

PlotRMSE = RMSE_to_plot(rmse);

