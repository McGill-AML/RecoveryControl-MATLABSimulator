% loop to test variables
clear;

timer = zeros(11,1);

angle_head = 1;
posn_hit = 0;

for loop_no = 41:46
  
    clearvars -except loop_no timer rmse angle_head posn_hit 
    


    angle_head = rand*pi;
    posn_hit = rand*2 + 2;

    
    
    % set waypoints for run. 
    Setpoint = initsetpoint;
    
    
    % Setpt 1
    Setpoint.head = angle_head;
    Setpoint.time = 20;
    Setpoint.posn = [0;0;5];
    Trajectory = Setpoint;
    
    % Setpt 2
    Setpoint.head = angle_head;
    Setpoint.time = Setpoint.time + 10;
    Setpoint.posn = [posn_hit;0;5];
    Trajectory = [Trajectory;Setpoint];
    
    
    startsim_trajectory;
    
  
    if ~exist('time_of_recovery','var');
        time_of_recovery = iSim;
    end
    rmse(loop_no,:) = rmse_att(Plot,sensParams,time_of_recovery/tStep);
    
    rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);
    
    
    

%     save('acc_bound_0.5_12-21','rmse');
    



    
end

PlotRMSE = RMSE_to_plot(rmse);

