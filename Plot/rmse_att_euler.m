function [PlotRMSEeul] = rmse_att_euler(Plot, PlotRMSEeul,crash_end, useExtData,loop_no, fileNo,  crashIndex, viconDrop, vicDelay )
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
   AttEuler = zeros(size(Plot.eulerAngles));
   for ii = 1:length(Plot.quaternions)
       [a, b, c] = quat2angle([-Plot.quaternions(1,ii);Plot.quaternions(2:4,ii)], 'zyx');
       AttEuler(:,ii) = [a;b;c];
   end
    
else
   crash_start = crashIndex(1,fileNo) - vicDelay;
   crash_end = crashIndex(2,fileNo) - vicDelay;
   
    AttEuler = Plot.eulerAngles;
    useViconDrop = viconDrop{fileNo} - vicDelay;
    temp = 1;
    if ~isempty(useViconDrop)
        while useViconDrop(temp) <= 0 %ensure vicondrop indices are positive.
            useViconDrop(temp) = 1;
            temp = temp +1;
        end
    end
end

%% rmse of euler angles during a crash
%set indices to use
if useExtData == 0
    crash_ind = crash_start:crash_end;
else
    % remove vicon drop indices for exp data
    % take entire indices, then set ones you dont want to zero, set zero = []
    crash_ind = 1:length(Plot.quaternions)- vicDelay;
    crash_ind([1:crash_start-1,crash_end+1:length(Plot.quaternions)- vicDelay]) = 0;
    for ii = 1:length(useViconDrop)/2
        crash_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    crash_ind(crash_ind == 0) = [];
end

temp_eul = zeros(3,size(Plot.quaternions(:,crash_ind),2));
for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,crash_ind(ii))) ;
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)), [a;b;c]);
end
PlotRMSEeul.crash.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] =  quat2angle(Plot.AHINF_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_ind),2)
     [a, b, c] =  quat2angle(Plot.SRSPKF_quat(:,crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.crash.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


%% rmse pre crash

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
    
temp_eul = zeros(3,length(pre_crash_ind));
for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.AHINF_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(pre_crash_ind)
     [a, b, c] = quat2angle(Plot.SRSPKF_quat(:,pre_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,pre_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.pre_crash.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));

%% rmse post crash

if useExtData == 0
    post_crash_ind = crash_end:length(Plot.quaternions);
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    post_crash_ind = 1:length(Plot.quaternions)- vicDelay;
    post_crash_ind(1:crash_end-1) = 0;
    for ii = 1:length(useViconDrop)/2
        post_crash_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    post_crash_ind(post_crash_ind == 0) = [];
end
temp_eul = zeros(3,length(post_crash_ind));
for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.AHINF_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(post_crash_ind)
     [a, b, c] = quat2angle(Plot.SRSPKF_quat(:,post_crash_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,post_crash_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.post_crash.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


%% total rmse full sim
if useExtData == 0
    total_ind = 1:length(Plot.quaternions);
else
    % remove vicon drop indices
    % take entire indices, then set ones you dont want to zero, set zero = []
    total_ind = 1:length(Plot.quaternions)- vicDelay;
    
    for ii = 1:length(useViconDrop)/2
        total_ind(useViconDrop(ii*2-1):useViconDrop(ii*2)) = 0;
    end
    total_ind(total_ind == 0) = [];
end


temp_eul = zeros(3,length(total_ind));
for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,total_ind(ii)));
    temp_eul(:,ii) =   calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.AHINF_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(total_ind)
     [a, b, c] = quat2angle(Plot.SRSPKF_quat(:,total_ind(ii)));
    temp_eul(:,ii) = calcEulErr(AttEuler(:,total_ind(ii)),  [a;b;c]);
end
PlotRMSEeul.total.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));

% record ICs for run
