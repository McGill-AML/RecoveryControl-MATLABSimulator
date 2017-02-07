function [PlotRMSEeul] = rmse_att_euler(Plot,sensParams,crash_end, PlotRMSEeul,loop_no)
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

%% rmse of euler angles during a crash
temp_eul = zeros(3,size(Plot.quaternions(:,crash_start:crash_end),2));
for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,crash_start+ii-1), 'xyz') ;
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) - [a;b;c];
end
PlotRMSEeul.crash.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] =  quat2angle(Plot.AHINF_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) - [a;b;c];
end
PlotRMSEeul.crash.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) -  [a;b;c];
end
PlotRMSEeul.crash.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:size(Plot.quaternions(:,crash_start:crash_end),2)
     [a, b, c] =  quat2angle(Plot.SRSPKF_quat(:,crash_start+ii-1), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,crash_start+ii-1) - [a;b;c];
end
PlotRMSEeul.crash.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


%% rmse when not crashing
no_crash_ind = [1:crash_start,crash_end:length(Plot.quaternions)];

temp_eul = zeros(3,length(no_crash_ind));
for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) - [a;b;c];
end
PlotRMSEeul.not_crash.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) - [a;b;c];
end
PlotRMSEeul.not_crash.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.AHINF_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.SPKF_norm_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(no_crash_ind)
     [a, b, c] = quat2angle(Plot.SRSPKF_quat(:,no_crash_ind(ii)), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,no_crash_ind(ii)) -  [a;b;c];
end
PlotRMSEeul.not_crash.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));





%% total rmse full sim
temp_eul = zeros(3,length(Plot.eulerAngles));
for ii = 1:length(Plot.eulerAngles)
    [a, b, c] = quat2angle(Plot.SPKF_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c] ;
end
PlotRMSEeul.total.SPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] =  quat2angle(Plot.ASPKF_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c];
end
PlotRMSEeul.total.ASPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] = quat2angle(Plot.EKF_att_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) -  [a;b;c];
end
PlotRMSEeul.total.EKF_att_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] = quat2angle(Plot.HINF_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c];
end
PlotRMSEeul.total.HINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] = quat2angle(Plot.COMP_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) -  [a;b;c];
end
PlotRMSEeul.total.COMP_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] = quat2angle(Plot.ASPKF_opt_quat(:,ii), 'xyz') ;
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c];
end
PlotRMSEeul.total.ASPKF_opt_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] =  quat2angle(Plot.AHINF_quat(:,ii), 'xyz') ;
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) -[a;b;c];
end
PlotRMSEeul.total.AHINF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] =  quat2angle(Plot.SPKF_norm_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c];
end
PlotRMSEeul.total.SPKF_norm_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));


for ii = 1:length(Plot.eulerAngles)
     [a, b, c] =  quat2angle(Plot.SRSPKF_quat(:,ii), 'xyz');
    temp_eul(:,ii) = Plot.eulerAngles(:,ii) - [a;b;c];
end
PlotRMSEeul.total.SRSPKF_eul(:,loop_no) = sqrt(mean(temp_eul.^2,2));



% record ICs for run
