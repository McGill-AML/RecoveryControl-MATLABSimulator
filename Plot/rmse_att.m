function [rmse] = rmse_att(Plot,sensParams,crash_end)
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
rmse.crash.SPKF_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.SPKF_quat(:,crash_start:crash_end)).^2),2);
rmse.crash.ASPKF_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.ASPKF_quat(:,crash_start:crash_end)).^2),2);
rmse.crash.EKF_att_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.EKF_att_quat(:,crash_start:crash_end)).^2),2);
rmse.crash.HINF_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.HINF_quat(:,crash_start:crash_end)).^2),2);
rmse.crash.COMP_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.COMP_quat(:,crash_start:crash_end)).^2),2);
rmse.crash.SPKF_full_quat = mean(sqrt((Plot.quaternions(:,crash_start:crash_end)-Plot.SPKF_full_quat(:,crash_start:crash_end)).^2),2);

rmse.crash.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,crash_start:crash_end)).^2),2);

%rmse when not crashing
no_crash_ind = [1:crash_start,crash_end:length(Plot.quaternions)];

rmse.not_crash.SPKF_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.SPKF_quat(:,no_crash_ind)).^2),2);
rmse.not_crash.ASPKF_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.ASPKF_quat(:,no_crash_ind)).^2),2);
rmse.not_crash.EKF_att_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.EKF_att_quat(:,no_crash_ind)).^2),2);
rmse.not_crash.HINF_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.HINF_quat(:,no_crash_ind)).^2),2);
rmse.not_crash.COMP_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.COMP_quat(:,no_crash_ind)).^2),2);
rmse.not_crash.SPKF_full_quat = mean(sqrt((Plot.quaternions(:,no_crash_ind)-Plot.SPKF_full_quat(:,no_crash_ind)).^2),2);

rmse.not_crash.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,no_crash_ind)).^2),2);

%total rmse full sim
rmse.total.SPKF_quat = mean(sqrt((Plot.quaternions-Plot.SPKF_quat).^2),2);
rmse.total.ASPKF_quat = mean(sqrt((Plot.quaternions-Plot.ASPKF_quat).^2),2);
rmse.total.EKF_att_quat = mean(sqrt((Plot.quaternions-Plot.EKF_att_quat).^2),2);
rmse.total.HINF_quat = mean(sqrt((Plot.quaternions-Plot.HINF_quat).^2),2);
rmse.total.COMP_quat = mean(sqrt((Plot.quaternions-Plot.COMP_quat).^2),2);
rmse.total.SPKF_full_quat = mean(sqrt((Plot.quaternions-Plot.SPKF_full_quat).^2),2);

rmse.total.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias).^2),2);
rmse.total.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias).^2),2);
rmse.total.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias).^2),2);
rmse.total.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias).^2),2);
rmse.total.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias).^2),2);
rmse.total.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias).^2),2);

% record ICs for run
