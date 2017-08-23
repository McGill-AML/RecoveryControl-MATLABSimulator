% script to import data
% try 38
 fileNo = 60;

if fileNo < 10;
    px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\March 8th Indoors Zero Reference Acceleration\px4logs\csv\q0',num2str(fileNo),'.csv']);
else
    px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\March 8th Indoors Zero Reference Acceleration\px4logs\csv\q',num2str(fileNo),'.csv']);
end
% px4Data = importdata(['D:\Dropbox\Masters\Navi\Adrian_data\CompassMot\battery_rev.csv']);

% px4Data = importdata(['D:\Dropbox\Masters\Navi\Adrian_data\May5th\px4log-2017-05-11\08_04_38.csv']);


% if fileNo < 10;
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\April 7th Indoors - Same as Field Trial Conditions\csv\s0',num2str(fileNo),'.csv']);
% else
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\April 7th Indoors - Same as Field Trial Conditions\csv\s',num2str(fileNo),'.csv']);
% end
% % 
% if fileNo < 10;
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\May 16th EKF attitude estimate\csv\a0',num2str(fileNo),'.csv']);
% else
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\May 16th EKF attitude estimate\csv\a',num2str(fileNo),'.csv']);
% end

% if fileNo < 10;
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\June 1st AHINF attitude estimate\csv\b0',num2str(fileNo),'.csv']);
% else
%     px4Data = importdata(['D:\Dropbox\Spiri Collision Recovery\Experiments\Spring 2017\June 1st AHINF attitude estimate\csv\b',num2str(fileNo),'.csv']);
% end

% if fileNo < 10;
%     px4Data = importdata(['D:\Dropbox\Navi_crash_data\VII\px4logs\CSVs\crash_g0',num2str(fileNo),'.csv']);
% else
%     px4Data = importdata(['D:\Dropbox\Navi_crash_data\VII\px4logs\CSVs\crash_g',num2str(fileNo),'.csv']);
% end


% remove only data I want from the .csv file
for ii = 1:length(px4Data.colheaders)

    if strncmpi('IMU',px4Data.colheaders(ii),3)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('ATT_q',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('VISN_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('MOCP_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('BATT_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('TIME_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('ATTC_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
    elseif strncmpi('IRST_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
        
    elseif strncmpi('LPOS_',px4Data.colheaders(ii),5)
        evalc([px4Data.colheaders{ii} ' = px4Data.data(:,ii)']);
    end
    
end

% set time to be actual time
TIME = TIME_StartTime - TIME_StartTime(2);
TIME = TIME/1000000;

clearvars remove_index
% find repeated log entries. Not sure why there are repeats
count = 1;
for ii = 2:length(TIME)
    if TIME(ii) == TIME(ii-1) && IMU_AccX(ii) == IMU_AccX(ii-1) && IMU_GyroX(ii) == IMU_GyroX(ii-1)
        remove_index(count) = ii;
        count = count+1;
    end
end

% remove repeated log entries
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
% MOCP_QuatW(remove_index) = [];
% MOCP_QuatX(remove_index) = [];
% MOCP_QuatY(remove_index) = [];
% MOCP_QuatZ(remove_index) = [];
% MOCP_X(remove_index) = [];
% MOCP_Y(remove_index) = [];
% MOCP_Z(remove_index) = [];
% VISN_QuatW(remove_index) = [];
% VISN_QuatX(remove_index) = [];
% VISN_QuatY(remove_index) = [];
% VISN_QuatZ(remove_index) = [];
% VISN_X(remove_index) = [];
% VISN_Y(remove_index) = [];
% VISN_Z(remove_index) = [];
% BATT_Curr(remove_index) = [];
% ATTC_Thrust(remove_index) = [];
% IRST_RS(remove_index) = [];


    


%clear excess data
clear px4Data

%load motorcompensation variable
load('.\Sensors\Mag_compensation_feb_2017');
load('.\PX4_Processing\qRunIndices.mat');