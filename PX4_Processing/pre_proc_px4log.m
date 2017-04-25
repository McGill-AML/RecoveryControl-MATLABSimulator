%% Pre-process px4log data

% remove repeat time / data points from .csv data and create TIME variable

TIME = TIME_StartTime - TIME_StartTime(2);
TIME = TIME/1000000;
count = 1;
for ii = 2:length(TIME)
    if TIME(ii) == TIME(ii-1) && IMU_AccX(ii) == IMU_AccX(ii-1) && IMU_GyroX(ii) == IMU_GyroX(ii-1)
        remove_index(count) = ii;
        count = count+1;
    end
end
IMU_MagZ(remove_index) = [];
IMU_MagX(remove_index) = [];
IMU_MagY(remove_index) = [];
IMU_GyroY(remove_index) = [];
IMU_GyroZ(remove_index) = [];
IMU_GyroX(remove_index) = [];
IMU_AccX(remove_index) = [];
IMU_AccY(remove_index) = [];
IMU_AccZ(remove_index) = [];
TIME(remove_index) = [];
ATT_qw(remove_index) = [];
ATT_qx(remove_index) = [];
ATT_qy(remove_index) = [];
ATT_qz(remove_index) = [];
ATT_Pitch(remove_index) = [];
ATT_Roll(remove_index) = [];
ATT_Yaw(remove_index) = [];
ATTC_Thrust(remove_index) = []; 

%% 
% 
% LPOS_X(remove_index) = []; 
% LPOS_Y(remove_index) = []; 
% LPOS_Z(remove_index) = []; 


BATT_Curr(remove_index) = [];

%% change the cell atttitude data to double - set empty cells to 0