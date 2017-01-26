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

%% rmse during a crash
rmse.crash.SPKF_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.SPKF_quat(:,crash_start:crash_end));
rmse.crash.ASPKF_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.ASPKF_quat(:,crash_start:crash_end));
rmse.crash.EKF_att_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.EKF_att_quat(:,crash_start:crash_end));
rmse.crash.HINF_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.HINF_quat(:,crash_start:crash_end));
rmse.crash.COMP_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.COMP_quat(:,crash_start:crash_end));
% rmse.crash.SPKF_full_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.SPKF_full_quat(:,crash_start:crash_end));
rmse.crash.SPKF_full_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), zeros(size(Plot.quaternions(:,crash_start:crash_end)))); %temp to keep indexing the same
rmse.crash.ASPKF_opt_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.ASPKF_opt_quat(:,crash_start:crash_end));
rmse.crash.AHINF_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.AHINF_quat(:,crash_start:crash_end));

rmse.crash.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,crash_start:crash_end)).^2),2);
% rmse.crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias(:,crash_start:crash_end)))).^2),2); %temp to keep indexing the same
rmse.crash.ASPKF_opt_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,crash_start:crash_end)).^2),2);
rmse.crash.AHINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,crash_start:crash_end)).^2),2);


rmse.crash.SPKF_norm_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.SPKF_norm_quat(:,crash_start:crash_end));
rmse.crash.SPKF_norm_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,crash_start:crash_end)).^2),2);

rmse.crash.SRSPKF_quat = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.SRSPKF_quat(:,crash_start:crash_end));
rmse.crash.SRSPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,crash_start:crash_end)).^2),2);

    % rmse for non normalized during crash
rmse.crash.SPKF_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.SPKF_quat_noN(:,crash_start:crash_end));
rmse.crash.ASPKF_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.ASPKF_quat_noN(:,crash_start:crash_end));
rmse.crash.ASPKF_opt_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.ASPKF_opt_quat_noN(:,crash_start:crash_end));
rmse.crash.EKF_att_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.EKF_att_quat_noN(:,crash_start:crash_end));
rmse.crash.HINF_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.HINF_quat_noN(:,crash_start:crash_end));
rmse.crash.AHINF_quat_noN = quat_error(Plot.quaternions(:,crash_start:crash_end), Plot.AHINF_quat_noN(:,crash_start:crash_end));

rmse.crash.SPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias_noN(:,crash_start:crash_end)).^2),2);
rmse.crash.ASPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias_noN(:,crash_start:crash_end)).^2),2);
rmse.crash.ASPKF_opt_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias_noN(:,crash_start:crash_end)).^2),2);
rmse.crash.EKF_att_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias_noN(:,crash_start:crash_end)).^2),2);
rmse.crash.HINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias_noN(:,crash_start:crash_end)).^2),2);
rmse.crash.AHINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias_noN(:,crash_start:crash_end)).^2),2);



%% rmse when not crashing
no_crash_ind = [1:crash_start,crash_end:length(Plot.quaternions)];

rmse.not_crash.SPKF_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.SPKF_quat(:,no_crash_ind));
rmse.not_crash.ASPKF_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.ASPKF_quat(:,no_crash_ind));
rmse.not_crash.EKF_att_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.EKF_att_quat(:,no_crash_ind));
rmse.not_crash.HINF_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.HINF_quat(:,no_crash_ind));
rmse.not_crash.COMP_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.COMP_quat(:,no_crash_ind));
% rmse.not_crash.SPKF_full_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.SPKF_full_quat(:,no_crash_ind));
rmse.not_crash.SPKF_full_quat = quat_error(Plot.quaternions(:,no_crash_ind), zeros(size(Plot.quaternions(:,no_crash_ind)))); %temp to keep indexing the same
rmse.not_crash.ASPKF_opt_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.ASPKF_opt_quat(:,no_crash_ind));
rmse.not_crash.AHINF_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.AHINF_quat(:,no_crash_ind));

rmse.not_crash.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,no_crash_ind)).^2),2);
% rmse.not_crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias(:,no_crash_ind)))).^2),2); %temp to keep indexing the same
rmse.not_crash.ASPKF_opt_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,no_crash_ind)).^2),2);
rmse.not_crash.AHINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,no_crash_ind)).^2),2);


rmse.not_crash.SPKF_norm_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.SPKF_norm_quat(:,no_crash_ind));
rmse.not_crash.SPKF_norm_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,no_crash_ind)).^2),2);

rmse.not_crash.SRSPKF_quat = quat_error(Plot.quaternions(:,no_crash_ind), Plot.SRSPKF_quat(:,no_crash_ind));
rmse.not_crash.SRSPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,no_crash_ind)).^2),2);

    % rmse for non normalized during not crash
rmse.not_crash.SPKF_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.SPKF_quat_noN(:,no_crash_ind));
rmse.not_crash.ASPKF_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.ASPKF_quat_noN(:,no_crash_ind));
rmse.not_crash.ASPKF_opt_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.ASPKF_opt_quat_noN(:,no_crash_ind));
rmse.not_crash.EKF_att_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.EKF_att_quat_noN(:,no_crash_ind));
rmse.not_crash.HINF_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.HINF_quat_noN(:,no_crash_ind));
rmse.not_crash.AHINF_quat_noN = quat_error(Plot.quaternions(:,no_crash_ind), Plot.AHINF_quat_noN(:,no_crash_ind));

rmse.not_crash.SPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias_noN(:,no_crash_ind)).^2),2);
rmse.not_crash.ASPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias_noN(:,no_crash_ind)).^2),2);
rmse.not_crash.ASPKF_opt_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias_noN(:,no_crash_ind)).^2),2);
rmse.not_crash.EKF_att_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias_noN(:,no_crash_ind)).^2),2);
rmse.not_crash.HINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias_noN(:,no_crash_ind)).^2),2);
rmse.not_crash.AHINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias_noN(:,no_crash_ind)).^2),2);


%% total rmse full sim
rmse.total.SPKF_quat = quat_error(Plot.quaternions, Plot.SPKF_quat);
rmse.total.ASPKF_quat = quat_error(Plot.quaternions, Plot.ASPKF_quat);
rmse.total.EKF_att_quat = quat_error(Plot.quaternions, Plot.EKF_att_quat);
rmse.total.HINF_quat = quat_error(Plot.quaternions, Plot.HINF_quat);
rmse.total.COMP_quat = quat_error(Plot.quaternions, Plot.COMP_quat);
% rmse.total.SPKF_full_quat = quat_error(Plot.quaternions, Plot.SPKF_full_quat);
rmse.total.SPKF_full_quat = quat_error(Plot.quaternions, zeros(size(Plot.quaternions))); %temp to keep indexing the same
rmse.total.ASPKF_opt_quat = quat_error(Plot.quaternions, Plot.ASPKF_opt_quat);
rmse.total.AHINF_quat = quat_error(Plot.quaternions, Plot.AHINF_quat);

rmse.total.SPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias).^2),2);
rmse.total.ASPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias).^2),2);
rmse.total.EKF_att_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias).^2),2);
rmse.total.HINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias).^2),2);
rmse.total.COMP_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias).^2),2);
% rmse.total.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias).^2),2);
rmse.total.SPKF_full_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias))).^2),2); %temp to keep indexing the same
rmse.total.ASPKF_opt_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias).^2),2);
rmse.total.AHINF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias).^2),2);


rmse.total.SPKF_norm_quat = quat_error(Plot.quaternions, Plot.SPKF_norm_quat);
rmse.total.SPKF_norm_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias).^2),2);

rmse.total.SRSPKF_quat = quat_error(Plot.quaternions, Plot.SRSPKF_quat);
rmse.total.SRSPKF_gyr_bias = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias).^2),2);

rmse.total.SPKF_quat_noN = quat_error(Plot.quaternions, Plot.SPKF_quat_noN);
rmse.total.ASPKF_quat_noN = quat_error(Plot.quaternions, Plot.ASPKF_quat_noN);
rmse.total.ASPKF_opt_quat_noN = quat_error(Plot.quaternions, Plot.ASPKF_opt_quat_noN);
rmse.total.EKF_att_quat_noN = quat_error(Plot.quaternions, Plot.EKF_att_quat_noN);
rmse.total.HINF_quat_noN = quat_error(Plot.quaternions, Plot.HINF_quat_noN);
rmse.total.AHINF_quat_noN = quat_error(Plot.quaternions, Plot.AHINF_quat_noN);

rmse.total.SPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias_noN).^2),2);
rmse.total.ASPKF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias_noN).^2),2);
rmse.total.ASPKF_opt_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias_noN).^2),2);
rmse.total.EKF_att_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias_noN).^2),2);
rmse.total.HINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias_noN).^2),2);
rmse.total.AHINF_gyr_bias_noN = mean(sqrt(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias_noN).^2),2);

% record ICs for run
