%% compass motor calibration based on Arducopter compassmot.cpp
% initialize
% mag_field_0 = mean([IMU_MagX(1:20)'; IMU_MagY(1:20)'; IMU_MagZ(1:20)'], 2);
% mot_comp = [0;0;0];

%% this is how the ardu copter code does it. maybe. I think this is how they do it, but it doesn't work here so I'm not sure.
% they use data as it comes in so there's definitely an easier way to do it
% post process, but this is how they live update the values.... I think?
% no, this a low pass filter on the compass data, so that the data is
% better, then they perform the computation. 

% for ii = 21:length(BATT_Curr)
%     
%     if (ATTC_Thrust(ii) <=  .003)
%         mag_field_0 = 0.99*mag_field_0 + 0.01*[IMU_MagX(ii); IMU_MagY(ii); IMU_MagZ(ii)];
%         
%     elseif BATT_Curr(ii) >= 3
%         mot_comp_now(:,ii) = ([IMU_MagX(ii); IMU_MagY(ii); IMU_MagZ(ii)] - mag_field_0)/BATT_Curr(ii);
%         
%         mot_comp = mot_comp*0.99 + mot_comp_now(:,ii)*0.01;
%     end
%     
% end
% 
% perc_int = norm(mot_comp)*max(BATT_Curr)/max(ATTC_Thrust)/230*100;

%% easier post process way
mag_f_0_ind = [];
mot_comp_ind = [];

%finding indices when its likely affected by battery current
for ii = 1:length(BATT_Curr)
    
	if (ATTC_Thrust(ii) <=  .003)
       mag_f_0_ind = [mag_f_0_ind; ii];
        
    elseif BATT_Curr(ii) >= 3
       mot_comp_ind = [mot_comp_ind; ii];
        
    end
    
end



%% init stuff
mag_field_0 = sum([IMU_MagX(mag_f_0_ind)'; IMU_MagY(mag_f_0_ind)'; IMU_MagZ(mag_f_0_ind)'],2)/length(mag_f_0_ind);
batt_I_0 = sum([BATT_Curr(mag_f_0_ind(4:end))])/length(mag_f_0_ind);
mot_comp = zeros(3,1);

% %% averaging method of mot comp:
% 
% for ii = 1:length(mot_comp_ind)
%     mot_comp = mot_comp + ([IMU_MagX(mot_comp_ind(ii))'; IMU_MagY(mot_comp_ind(ii))'; IMU_MagZ(mot_comp_ind(ii))'] - mag_field_0)/(BATT_Curr(mot_comp_ind(ii)));%-batt_I_0);
% end
% 
% mot_comp = mot_comp/length(mot_comp_ind);
% 
% %max value method of motcomp:
% [max_batt_I, max_batt_ind] = max(BATT_Curr);
% 
% mot_comp = ([IMU_MagX(max_batt_ind)'; IMU_MagY(max_batt_ind)'; IMU_MagZ(max_batt_ind)'] - mag_field_0)/(BATT_Curr(max_batt_ind));%-batt_I_0);

%% filter the data first?
mag_filt = filter( 0.01, [1, -.99]   ,[IMU_MagX, IMU_MagY, IMU_MagZ]);

%THIS WAS THE ONE THAT WAS USED!!!
%averaging method of motcomp: 

for ii = 1:length(mot_comp_ind)
    mot_comp = mot_comp + (mag_filt(mot_comp_ind(ii),:)' - mag_field_0)/(BATT_Curr(mot_comp_ind(ii))-batt_I_0);
end

mot_comp = mot_comp/length(mot_comp_ind);
% 
% %max value method of motcomp:
% [max_batt_I, max_batt_ind] = max(BATT_Curr);
% 
% mot_comp = (mag_filt(max_batt_ind,:)'- mag_field_0)/(BATT_Curr(max_batt_ind)-batt_I_0);

%% correct using mot comp

for ii = 1:length(IMU_MagX)
    mag_corrected(:,ii) = [IMU_MagX(ii); IMU_MagY(ii); IMU_MagZ(ii)] - mot_comp*BATT_Curr(ii);
end

% figure;plot(TIME,mag_filt);grid on
figure;plot(TIME, mag_corrected);grid on
title('Corrected Magnetometer Data'); 
xlabel('Time'); ylabel('Mag Field [Gauss]');

% figure;plot(TIME, bsxfun(@minus,[IMU_MagX, IMU_MagY, IMU_MagZ],mag_field_0'));grid on