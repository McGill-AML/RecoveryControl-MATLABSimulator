% create histograms for crash or not crash data

% rpy_data = zeros(7,3);   % indices for good crash data (:,[1:39, 41:57, 60:64,66:74])
% rpy_cov = zeros(8,6);
% rpy_data = zeros(4,4);
% rpy_cov = zeros(4,3);

% rpy_data(1,:) = mean(rmseEUL.total.SPKF_norm_eul,2)'; 
% rpy_data(1,1) = mean(rmse.total.COMP_quat,2)';
% rpy_data(2,1) = mean(rmse.total.EKF_att_quat,2)';
% rpy_data(3,1) = mean(rmse.total.HINF_quat,2)';
% rpy_data(4,1) = mean(rmse.total.SPKF_quat,2)';
% 
% rpy_data(1,2:4) = mean(rmseEUL.total.COMP_eul,2)';
% rpy_data(2,2:4) = mean(rmseEUL.total.EKF_att_eul,2)';
% rpy_data(3,2:4) = mean(rmseEUL.total.HINF_eul,2)';
% rpy_data(4,2:4) = mean(rmseEUL.total.SPKF_eul,2)';

% rpy_cov(1,:) = (diag(cov(rmseEUL.total.COMP_eul'))').^.5;
% rpy_cov(2,:) = (diag(cov(rmseEUL.total.EKF_att_eul'))').^.5;
% rpy_cov(3,:) = (diag(cov(rmseEUL.total.HINF_eul'))').^.5;
% rpy_cov(4,:) = (diag(cov(rmseEUL.total.AHINF_eul'))').^.5;

% rpy_data(1,:) = diag(cov(rmseEUL.crash.SPKF_norm_eul'))'; 
% rpy_data(2,:) = diag(cov(rmseEUL.crash.COMP_eul'))';
% rpy_data(3,:) = diag(cov(rmseEUL.crash.EKF_att_eul'))';
% rpy_data(4,:) = diag(cov(rmseEUL.crash.HINF_eul'))';
% rpy_data(5,:) = diag(cov(rmseEUL.crash.AHINF_eul'))';
% rpy_data(6,:) = diag(cov(rmseEUL.crash.SPKF_eul'))';
% rpy_data(7,:) = diag(cov(rmseEUL.crash.ASPKF_eul'))';
% rpy_data(8,:) = diag(cov(rmseEUL.crash.ASPKF_opt_eul'))';

% rpy_data(1,:) = mean(rmseEUL.total.COMP_eul,2)';
% rpy_data(2,:) = mean(rmseEUL.total.EKF_att_eul,2)';
% rpy_data(3,:) = mean(rmseEUL.total.HINF_eul,2)';
% rpy_data(4,:) = mean(rmseEUL.total.AHINF_eul,2)';
% rpy_data(5,:) = mean(rmseEUL.total.SPKF_eul,2)';
% rpy_data(6,:) = mean(rmseEUL.total.ASPKF_eul,2)';
% rpy_data(7,:) = mean(rmseEUL.total.ASPKF_opt_eul,2)';

%% sim collision data
% tempc = mean(rmseEUL.crash.SPKF_norm_eul,2)'; 
% tempt =  mean(rmseEUL.total.SPKF_norm_eul,2)';
% rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.COMP_eul,2)'; 
% tempt =  mean(rmseEUL.total.COMP_eul,2)';
% rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.EKF_att_eul,2)'; 
% tempc(1) = 1.84; tempc(2) = 0.88; tempc(3) = 0.76;
% tempt =  mean(rmseEUL.total.EKF_att_eul,2)';
% tempt(1) = 2.11; tempt(2) = 0.60; tempt(3) = 0.37;
% rpy_data(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.HINF_eul,2)'; 
% tempt =  mean(rmseEUL.total.HINF_eul,2)';
% rpy_data(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.AHINF_eul,2)'; 
% tempt =  mean(rmseEUL.total.AHINF_eul,2)';
% rpy_data(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.SPKF_eul,2)'; 
% tempt =  mean(rmseEUL.total.SPKF_eul,2)';
% rpy_data(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.ASPKF_eul,2)'; 
% tempc(1) = 1.37; tempc(2) = 0.68; tempc(3) = 0.66;
% tempt =  mean(rmseEUL.total.ASPKF_eul,2)';
% tempt(1) = 1.91; tempt(2) = 0.51; tempt(3) = 0.33;
% rpy_data(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = mean(rmseEUL.crash.ASPKF_opt_eul,2)'; 
% tempt =  mean(rmseEUL.total.ASPKF_opt_eul,2)';
% rpy_data(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];

% tempc = mean(rmse.crash.COMP_quat,2)'; 
% tempt =  mean(rmse.total.COMP_quat,2)';
% rpy_data(1,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.EKF_att_quat,2)'; 
% tempc(1) = 2.23;
% tempt =  mean(rmse.total.EKF_att_quat,2)';
% tempt(1) = 2.22;
% rpy_data(2,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.HINF_quat,2)'; 
% tempt =  mean(rmse.total.HINF_quat,2)';
% rpy_data(3,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.AHINF_quat,2)'; 
% tempt =  mean(rmse.total.AHINF_quat,2)';
% rpy_data(4,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.SPKF_quat,2)'; 
% tempt =  mean(rmse.total.SPKF_quat,2)';
% rpy_data(5,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.ASPKF_quat,2)'; 
% tempc(1) = 1.69;
% tempt =  mean(rmse.total.ASPKF_quat,2)';
% tempt(1) = 1.99;
% rpy_data(6,:) = [tempt(1), tempc(1), 0];
% tempc = mean(rmse.crash.ASPKF_opt_quat,2)'; 
% tempt =  mean(rmse.total.ASPKF_opt_quat,2)';
% rpy_data(7,:) = [tempt(1), tempc(1), 0];


% rpy_data(1,3) = mean(rmse.total.COMP_quat,2)';
% rpy_data(2,3) = mean(rmse.total.EKF_att_quat,2)';
% rpy_data(3,3) = mean(rmse.total.HINF_quat,2)';
% rpy_data(4,3) = mean(rmse.total.AHINF_quat,2)';
% rpy_data(5,3) = mean(rmse.total.SPKF_quat,2)';
% rpy_data(6,3) = mean(rmse.total.ASPKF_quat,2)';
% rpy_data(7,3) = mean(rmse.total.ASPKF_opt_quat,2)';


% tempc = (diag(cov(rmseEUL.crash.SPKF_norm_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.SPKF_norm_eul'))').^.5;
% rpy_cov(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.COMP_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.COMP_eul'))').^.5;
% rpy_cov(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.EKF_att_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.EKF_att_eul'))').^.5;
% rpy_cov(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.HINF_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.HINF_eul'))').^.5;
% rpy_cov(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.AHINF_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.AHINF_eul'))').^.5;
% rpy_cov(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.SPKF_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.SPKF_eul'))').^.5;
% rpy_cov(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.ASPKF_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.ASPKF_eul'))').^.5;
% rpy_cov(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmseEUL.crash.ASPKF_opt_eul'))').^.5; 
% tempt =  (diag(cov(rmseEUL.total.ASPKF_opt_eul'))').^.5;
% rpy_cov(8,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];

%% exp collision data
%this data is only the indices with vicon drops under 100 timesteps approx
%.80 seconds.  dataset has 51 entries
% use_files = [1:3, 6, 9:13, 15:22, 23, 26, 29:33, 35:36, 38, 40:49, 52:54, 56, 61,62, 64:67, 72:74, 76];
% use_files = maxDropLength([1:13,15:67, 71:78]) < 100;
% use_files([40, 58:59 65]) = 0;
%this data is only indices with crashes less than 0.4 seconds
% use_files = maxDropLength < 10000;
% use_files([14, 41, 59, 60, 66, 68:70]) = 0;

tempc = mean(rmseEUL.crash.SPKF_norm_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.SPKF_norm_eul(:, use_files),2)';
rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.COMP_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.COMP_eul(:, use_files),2)';
rpy_data(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.EKF_att_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.EKF_att_eul(:, use_files),2)';
rpy_data(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.HINF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.HINF_eul(:, use_files),2)';
rpy_data(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.AHINF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.AHINF_eul(:, use_files),2)';
rpy_data(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.SPKF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.SPKF_eul(:, use_files),2)';
rpy_data(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.ASPKF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.ASPKF_eul(:, use_files),2)';
rpy_data(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.ASPKF_opt_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.total.ASPKF_opt_eul(:, use_files),2)';
rpy_data(8,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];


% %% exp post collision data
% %this data is only the indices with vicon drops under 100 timesteps approx
% %.80 seconds.  dataset has 51 entries
% % use_files = [1:3, 6, 9:13, 15:22, 23, 26, 29:33, 35:36, 38, 40:49, 52:54, 56, 61,62, 64:67, 72:74, 76];
% use_files = maxDropLength < 1000;
% use_files([14, 67:70]) = 0;
% %this data is only indices with crashes less than 0.4 seconds
% % use_files = maxDropLength < 100;
% % use_files([14, 41, 59, 60, 66, 68:70]) = 0;
% 
tempc = mean(rmseEUL.post_crash.SPKF_norm_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.SPKF_norm_eul(:, use_files),2)';
rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.COMP_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.COMP_eul(:, use_files),2)';
rpy_data(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.EKF_att_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.EKF_att_eul(:, use_files),2)';
rpy_data(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.HINF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.HINF_eul(:, use_files),2)';
rpy_data(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.AHINF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.AHINF_eul(:, use_files),2)';
rpy_data(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.SPKF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.SPKF_eul(:, use_files),2)';
rpy_data(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.ASPKF_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.ASPKF_eul(:, use_files),2)';
rpy_data(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.post_crash.ASPKF_opt_eul(:, use_files),2)'; 
tempt =  mean(rmseEUL.pre_crash.ASPKF_opt_eul(:, use_files),2)';
rpy_data(8,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% 

figure;

% clrmp = [1 0 1; 0.99 0 0.99; 0 1 0; 0 0.99 0.01; 0 0 1; 0.01 0 0.99];
% clrmp = [0 0 0; 1 0 1;  0 1 0; 0 0 1];

clrmp = [1 0 1; 0 0 0; 0 0 0.01];

colormap(clrmp);
b_data = bar(rpy_data); 

% clearvars ctr ydt
% for ii = 1:size(rpy_data,2)
%     ctr(ii,:) = bsxfun(@plus, b_data(1).XData, [b_data(ii).XOffset]');
%     ydt(ii,:) = b_data(ii).YData;
% end

% hold on;
% errorbar(ctr', ydt', rpy_cov, '.r','LineWidth', 2);


grid on;

set(gca,'FontSize',16);
% set(gca,'XTickLabel', {'PX4','Comp','MEKF','H_\infty','AH_\infty','UKF','AUKF_1','AUKF_2'});
set(gca,'XTickLabel', {'Comp','MEKF','H_\infty','AH_\infty','UKF','AUKF_1','AUKF_2'});
% set(gca,'XTickLabel', {'Comp','MEKF','H_\infty','UKF'});


% legend('Yaw - total','Yaw - crash','Pitch - total','Pitch - crash','Roll - total','Roll - crash');
% legend('Yaw - pre crash','Yaw - post crash','Pitch - pre crash','Pitch - post crash','Roll - pre crash','Roll - post crash');
% legend('Total', 'Yaw','Pitch','Roll');
legend('Scenario 1', 'Scenario 3 - entire run', 'Scenario 3 - during crash');
ylabel('Total Attitude Error [\circ]');
xlabel('Estimation Algorithms');


% im_hatch = applyhatch_pluscolor(gcf,'k/k/k/',1,[0 0 0 0 0 0],clrmp);
% im_hatch = applyhatch_pluscolor(gcf,'kk/',1,[0 0 0],clrmp);


