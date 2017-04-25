function [rmse] = rmse_att(Plot,sensParams, rmse, crash_end, useExtData, loop_no, fileNo, crashIndex, viconDrop, vicDelay)
%function uses Plot data to compute RMSE because Hist data is in cell
%format so it'd need to be changed anyways


%finds RMSE for 4 different times: pre crashing, crash til recovery, post crash and
% total RMSE


if useExtData == 0
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
    useQuat = zeros(size(Plot.quaternions));
    for ii = 1:length(Plot.quaternions)
    useQuat(:,ii) = [-Plot.quaternions(1,ii);Plot.quaternions(2:4,ii)];
    end
else
   crash_start = crashIndex(1,fileNo) - vicDelay;
   crash_end = crashIndex(2,fileNo) - vicDelay;
   
    useQuat = Plot.quaternions;
    
    useViconDrop = viconDrop{fileNo} - vicDelay; %shift vicon to match with estimator data. 
    temp = 1;
    if ~isempty(useViconDrop)
        while useViconDrop(temp) <= 0 %ensure vicondrop indices are positive.
            useViconDrop(temp) = 1;
            temp = temp +1;
        end
    end
end
%% rmse during a crash
if useExtData == 0
    crash_ind = crash_start:crash_end;
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    crash_ind = 1:length(Plot.quaternions)- vicDelay;
    crash_ind([1:crash_start-1,crash_end+1:length(Plot.quaternions)- vicDelay]) = 0;
    for ii = 1:length(useViconDrop)/2
        crash_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    crash_ind(crash_ind == 0) = [];
end
    
    
rmse.crash.SPKF_quat(:,loop_no) = quat_error(useQuat(:,crash_ind), Plot.SPKF_quat(:,crash_ind));
rmse.crash.ASPKF_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.ASPKF_quat(:,crash_ind));
rmse.crash.EKF_att_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.EKF_att_quat(:,crash_ind));
rmse.crash.HINF_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.HINF_quat(:,crash_ind));
rmse.crash.COMP_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.COMP_quat(:,crash_ind));
% rmse.crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.SPKF_full_quat(:,crash_ind));
rmse.crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), zeros(size(useQuat(:,crash_ind)))); %temp to keep indexing the same
rmse.crash.ASPKF_opt_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.ASPKF_opt_quat(:,crash_ind));
rmse.crash.AHINF_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.AHINF_quat(:,crash_ind));

rmse.crash.SPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.ASPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.EKF_att_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.HINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.COMP_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,crash_ind)).^2,2));
% rmse.crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias(:,crash_ind)))).^2,2)); %temp to keep indexing the same
rmse.crash.ASPKF_opt_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,crash_ind)).^2,2));
rmse.crash.AHINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,crash_ind)).^2,2));


% rmse.crash.SPKF_norm_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.SPKF_norm_quat(:,crash_ind));
% rmse.crash.SPKF_norm_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,crash_ind)).^2,2));

rmse.crash.SRSPKF_quat(:,loop_no)= quat_error(useQuat(:,crash_ind), Plot.SRSPKF_quat(:,crash_ind));
rmse.crash.SRSPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,crash_ind)).^2,2));

    


%% rmse pre crashing
if useExtData == 0
    pre_crash_ind = 1:crash_start;
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    pre_crash_ind = 1:length(Plot.quaternions)- vicDelay;
    pre_crash_ind(crash_start+1:length(Plot.quaternions)- vicDelay) = 0;
    for ii = 1:length(useViconDrop)/2
        pre_crash_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    pre_crash_ind(pre_crash_ind == 0) = [];
end
    
rmse.pre_crash.SPKF_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.SPKF_quat(:,pre_crash_ind));
rmse.pre_crash.ASPKF_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.ASPKF_quat(:,pre_crash_ind));
rmse.pre_crash.EKF_att_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.EKF_att_quat(:,pre_crash_ind));
rmse.pre_crash.HINF_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.HINF_quat(:,pre_crash_ind));
rmse.pre_crash.COMP_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.COMP_quat(:,pre_crash_ind));
% rmse.pre_crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,no_crash_ind), Plot.SPKF_full_quat(:,no_crash_ind));
rmse.pre_crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), zeros(size(useQuat(:,pre_crash_ind)))); %temp to keep indexing the same
rmse.pre_crash.ASPKF_opt_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.ASPKF_opt_quat(:,pre_crash_ind));
rmse.pre_crash.AHINF_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.AHINF_quat(:,pre_crash_ind));

rmse.pre_crash.SPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,pre_crash_ind)).^2,2));
rmse.pre_crash.ASPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,pre_crash_ind)).^2,2));
rmse.pre_crash.EKF_att_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,pre_crash_ind)).^2,2));
rmse.pre_crash.HINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,pre_crash_ind)).^2,2));
rmse.pre_crash.COMP_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,pre_crash_ind)).^2,2));
% rmse.pre_crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,no_crash_ind)).^2,2));
rmse.pre_crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias(:,pre_crash_ind)))).^2,2)); %temp to keep indexing the same
rmse.pre_crash.ASPKF_opt_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,pre_crash_ind)).^2,2));
rmse.pre_crash.AHINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,pre_crash_ind)).^2,2));


% rmse.pre_crash.SPKF_norm_quat(:,loop_no)= quat_error(useQuat(:,no_crash_ind), Plot.SPKF_norm_quat(:,no_crash_ind));
% rmse.pre_crash.SPKF_norm_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,no_crash_ind)).^2,2));

rmse.pre_crash.SRSPKF_quat(:,loop_no)= quat_error(useQuat(:,pre_crash_ind), Plot.SRSPKF_quat(:,pre_crash_ind));
rmse.pre_crash.SRSPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,pre_crash_ind)).^2,2));

  
%% rmse post crashing
if useExtData == 0
    post_crash_ind = crash_end:length(useQuat);
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    post_crash_ind = 1:length(useQuat)- vicDelay;
    post_crash_ind(1:crash_end-1) = 0;
    for ii = 1:length(useViconDrop)/2
        post_crash_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    post_crash_ind(post_crash_ind == 0) = [];
end

rmse.post_crash.SPKF_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.SPKF_quat(:,post_crash_ind));
rmse.post_crash.ASPKF_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.ASPKF_quat(:,post_crash_ind));
rmse.post_crash.EKF_att_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.EKF_att_quat(:,post_crash_ind));
rmse.post_crash.HINF_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.HINF_quat(:,post_crash_ind));
rmse.post_crash.COMP_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.COMP_quat(:,post_crash_ind));
% rmse.post_crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,no_crash_ind), Plot.SPKF_full_quat(:,no_crash_ind));
rmse.post_crash.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), zeros(size(useQuat(:,post_crash_ind)))); %temp to keep indexing the same
rmse.post_crash.ASPKF_opt_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.ASPKF_opt_quat(:,post_crash_ind));
rmse.post_crash.AHINF_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.AHINF_quat(:,post_crash_ind));

rmse.post_crash.SPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,post_crash_ind)).^2,2));
rmse.post_crash.ASPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,post_crash_ind)).^2,2));
rmse.post_crash.EKF_att_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,post_crash_ind)).^2,2));
rmse.post_crash.HINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,post_crash_ind)).^2,2));
rmse.post_crash.COMP_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,post_crash_ind)).^2,2));
% rmse.post_crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias(:,no_crash_ind)).^2,2));
rmse.post_crash.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias(:,post_crash_ind)))).^2,2)); %temp to keep indexing the same
rmse.post_crash.ASPKF_opt_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,post_crash_ind)).^2,2));
rmse.post_crash.AHINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,post_crash_ind)).^2,2));


% rmse.post_crash.SPKF_norm_quat(:,loop_no)= quat_error(useQuat(:,no_crash_ind), Plot.SPKF_norm_quat(:,no_crash_ind));
% rmse.post_crash.SPKF_norm_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,no_crash_ind)).^2,2));

rmse.post_crash.SRSPKF_quat(:,loop_no)= quat_error(useQuat(:,post_crash_ind), Plot.SRSPKF_quat(:,post_crash_ind));
rmse.post_crash.SRSPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,post_crash_ind)).^2,2));

  


%% total rmse full sim
if useExtData == 0
    total_ind = 1:length(useQuat);
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    total_ind = 1:length(useQuat)- vicDelay;
    
    for ii = 1:length(useViconDrop)/2
        total_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    total_ind(total_ind == 0) = [];
end

    
rmse.total.SPKF_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.SPKF_quat(:,total_ind));
rmse.total.ASPKF_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.ASPKF_quat(:,total_ind));
rmse.total.EKF_att_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.EKF_att_quat(:,total_ind));
rmse.total.HINF_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.HINF_quat(:,total_ind));
rmse.total.COMP_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.COMP_quat(:,total_ind));
% rmse.total.SPKF_full_quat(:,loop_no)= quat_error(useQuat, Plot.SPKF_full_quat(:,total_ind));
rmse.total.SPKF_full_quat(:,loop_no)= quat_error(useQuat(:,total_ind), zeros(size(useQuat))); %temp to keep indexing the same
rmse.total.ASPKF_opt_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.ASPKF_opt_quat(:,total_ind));
rmse.total.AHINF_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.AHINF_quat(:,total_ind));

rmse.total.SPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_gyr_bias(:,total_ind)).^2,2));
rmse.total.ASPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_gyr_bias(:,total_ind)).^2,2));
rmse.total.EKF_att_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.EKF_att_gyr_bias(:,total_ind)).^2,2));
rmse.total.HINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.HINF_gyr_bias(:,total_ind)).^2,2));
rmse.total.COMP_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.COMP_gyr_bias(:,total_ind)).^2,2));
% rmse.total.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_full_gyr_bias).^2,2));
rmse.total.SPKF_full_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, zeros(size(Plot.ASPKF_opt_gyr_bias))).^2,2)); %temp to keep indexing the same
rmse.total.ASPKF_opt_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.ASPKF_opt_gyr_bias(:,total_ind)).^2,2));
rmse.total.AHINF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.AHINF_gyr_bias(:,total_ind)).^2,2));

% 
% rmse.total.SPKF_norm_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.SPKF_norm_quat(:,total_ind));
% rmse.total.SPKF_norm_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SPKF_norm_gyr_bias(:,total_ind)).^2,2));

rmse.total.SRSPKF_quat(:,loop_no)= quat_error(useQuat(:,total_ind), Plot.SRSPKF_quat(:,total_ind));
rmse.total.SRSPKF_gyr_bias(:,loop_no)= sqrt(mean(bsxfun(@minus,sensParams.bias.gyr, Plot.SRSPKF_gyr_bias(:,total_ind)).^2,2));


% record ICs for run
