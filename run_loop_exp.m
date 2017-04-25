% loop to run through all of the experimental data


% timer = zeros(11,1);

% clear;
% rmseEUL = [];
% rmse = [];
% ind4loop = 1;

for change_one = 1:1
    for change_two = 1:1
        
             
            for  fileNo = [2, 6, 9, 15, 16,  22,  30,  34, 45, 49, 54, 60, 73] %fileNo = [1:13,15:67, 71:78] %fileNo = [1:13,15:67, 71:78] %fileNo = [2, 6, 9, 15, 16,  22,  30,  34, 45, 49, 54, 60, 73]%fileNo = [2, 6, 9, 15, 16, 18, 21, 22, 28, 30, 31, 34, 45,47, 49, 54, 60, 73] %%% picked these dudes based on max drop length   %fileNo = [1:13,15:67, 71:78]
                ind4loop
                fileNo
                clearvars -except fileNo rmse rmseEUL change_two change_one ind4loop
                tic;
                running_a_loop = 1;
                sim_w_exp_data;
                
                %             rmse = rmse_att(Plot,sensParams, rmse, 0, useExpData, fileNo, crashIndex, viconDropIndex);
                
                rmse = rmse_att(Plot,sensParams, rmse, 0, useExpData, ind4loop, fileNo, crashIndex, viconDropIndex, vicDelay);
                %     rmse_position(loop_no,:) = rmse_pos(Plot,sensParams,time_of_recovery/tStep);
                
                
                %             rmseEUL = rmse_att_euler(Plot, rmseEUL, 0, useExpData, fileNo, crashIndex, viconDropIndex);
                
                rmseEUL = rmse_att_euler(Plot, rmseEUL, 0, useExpData, ind4loop, fileNo, crashIndex, viconDropIndex, vicDelay);
                
%                 save('full_exp_data','rmse','rmseEUL');
                
                ind4loop = ind4loop+1;
            end
        
    end
end
    
    
    
    