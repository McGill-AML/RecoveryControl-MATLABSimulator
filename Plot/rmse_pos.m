function [rmse] = rmse_pos(Plot,sensParams,crash_end)
%function uses Plot data to compute RMSE because Hist data is in cell
%format so it'd need to be changed anyways

%finds RMSE for 3 different times: while not crashing, crash til recovery, and
% total RMSE

crash_end = floor(crash_end); %make sure its an integer

for ii = 1:length(Plot.crash)
    if Plot.crash(ii) == 1
        crash_start = ii;
        break
    end
end

if ~exist('crash_start','var');
    crash_start = crash_end-1;
end

% rmse during a crash
rmse.crash.EKF_pos = mean(sqrt((Plot.posns(:,crash_start:crash_end) - Plot.EKF_pos(:,crash_start:crash_end)).^2),2);
rmse.crash.SPKF_full_pos =  mean(sqrt((Plot.posns(:,crash_start:crash_end) - Plot.SPKF_full_pos(:,crash_start:crash_end)).^2),2);


%rmse when not crashing
no_crash_ind = [1:crash_start,crash_end:length(Plot.quaternions)];


rmse.not_crash.EKF_pos = mean(sqrt((Plot.posns(:,no_crash_ind) - Plot.EKF_pos(:,no_crash_ind)).^2),2);
rmse.not_crash.SPKF_full_pos =  mean(sqrt((Plot.posns(:,no_crash_ind) - Plot.SPKF_full_pos(:,no_crash_ind)).^2),2);



%total rmse full sim
rmse.total.EKF_pos = mean(sqrt((Plot.posns - Plot.EKF_pos).^2),2);
rmse.total.SPKF_full_pos =  mean(sqrt((Plot.posns - Plot.SPKF_full_pos).^2),2);


% record ICs for run
