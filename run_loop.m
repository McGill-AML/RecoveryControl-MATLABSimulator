% % % % % loop to test variables
clear;

timer = zeros(11,1);

angle_head = 1;
posn_hit = 0;

rmseEUL = [];
rmse =[];
loop_no = 1;
% for change_one = 1:1
%     for change_two = 1:1
        for ICset = 1:100
            tic;
            loop_no
            clearvars -except loop_no timer rmse angle_head posn_hit rmse_position rmseEUL change_one change_two ICset
            running_a_loop = 1;
            
            
            angle_head = pi/3; %rand*pi;
            posn_hit =  3.25; % changed from 4.25
            
            
            
            % set waypoints for run.
            Setpoint = initsetpoint;
            
            
            % Setpt 1
            Setpoint.head = angle_head;
            Setpoint.time = 5;
            Setpoint.posn = [-0.15;0;0.15];
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
            Setpoint.time = Setpoint.time + 10; % changed this from + 5
            Setpoint.posn = [-5.15;0;5];
            Trajectory = [Trajectory;Setpoint];
            
            %     Setpt 3 - square
            Setpoint.head = angle_head;
            Setpoint.time = Setpoint.time + 10;  % changed this from +5
            Setpoint.posn = [-5.15;-5;5];
            Trajectory = [Trajectory;Setpoint];
            
            %     Setpt 4 - square
            Setpoint.head = angle_head;
            Setpoint.time = Setpoint.time + 6; % changed this from + 3
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
            Setpoint.time = Setpoint.time + 10; %changed from +5
            Setpoint.posn = [-0.4;0;5];
            Trajectory = [Trajectory;Setpoint];
            
            %     Setpt 5 - square
            Setpoint.head = angle_head;
            Setpoint.time = Setpoint.time + 5;
            Setpoint.posn = [-0.6;0;1.5];
            Trajectory = [Trajectory;Setpoint];
            
            %     Setpt 5 - square
            Setpoint.head = angle_head;
            Setpoint.time = Setpoint.time + 5;
            Setpoint.posn = [-0.6;0;0.6];
            Trajectory = [Trajectory;Setpoint];
            
%             %     Setpt 2 - crash
%                 Setpoint.head = angle_head;
%                 Setpoint.time = Setpoint.time + 10;
%                 Setpoint.posn = [posn_hit;0;5];
%                 Trajectory = [Trajectory;Setpoint];
%             %
            startsim_trajectory;
            
            
            if ~exist('time_of_recovery','var');
                time_of_recovery = iSim;
            end
            rmse = rmse_att(Plot,sensParams, rmse,time_of_recovery/tStep, useExpData, loop_no,0, 0, 0, 0);
            
            %     rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);
            
            
            rmseEUL = rmse_att_euler(Plot,  rmseEUL, time_of_recovery/tStep, useExpData, loop_no, 0, 0, 0, 0);
            
            
            save('scenario_1_fix_PX4','rmse','rmseEUL');
            loop_no = loop_no+1;


        end
%     end
%     
% end
% 
% save('upd_crsh_low_all');

% PlotRMSE = RMSE_to_plot(rmse);

