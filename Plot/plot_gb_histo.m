% create histograms for crash or not crash data

rpy_data = zeros(7,6);   % indices for good crash data (:,[1:39, 41:57, 60:64,66:74])
% rpy_cov = zeros(7,6);
% rpy_data = zeros(4,3);
% rpy_cov = zeros(4,4);

% rpy_data(1,:) = mean(rmse.total.SPKF_norm_gyr_bias,2)'; 
% rpy_data(1,1) = mean(rmse.total.COMP_gyr_bias,2)';
% rpy_data(2,1) = mean(rmse.total.EKF_att_gyr_bias,2)';
% rpy_data(3,1) = mean(rmse.total.HINF_gyr_bias,2)';
% rpy_data(4,1) = mean(rmse.total.SPKF_gyr_bias,2)';

% rpy_data(1,:) = mean(rmse.total.COMP_gyr_bias,2)'*180/pi;
% rpy_data(2,:) = mean(rmse.total.EKF_att_gyr_bias,2)'*180/pi;
% rpy_data(3,:) = mean(rmse.total.HINF_gyr_bias,2)'*180/pi;
% rpy_data(4,:) = mean(rmse.total.SPKF_gyr_bias,2)'*180/pi;

% rpy_cov(1,:) = (diag(cov(rmse.total.COMP_gyr_bias'))').^.5;
% rpy_cov(2,:) = (diag(cov(rmse.total.EKF_att_gyr_bias'))').^.5;
% rpy_cov(3,:) = (diag(cov(rmse.total.HINF_gyr_bias'))').^.5;
% rpy_cov(4,:) = (diag(cov(rmse.total.AHINF_gyr_bias'))').^.5;

% rpy_data(1,:) = diag(cov(rmse.crash.SPKF_norm_gyr_bias'))'; 
% rpy_data(2,:) = diag(cov(rmse.crash.COMP_gyr_bias'))';
% rpy_data(3,:) = diag(cov(rmse.crash.EKF_att_gyr_bias'))';
% rpy_data(4,:) = diag(cov(rmse.crash.HINF_gyr_bias'))';
% rpy_data(5,:) = diag(cov(rmse.crash.AHINF_gyr_bias'))';
% rpy_data(6,:) = diag(cov(rmse.crash.SPKF_gyr_bias'))';
% rpy_data(7,:) = diag(cov(rmse.crash.ASPKF_gyr_bias'))';
% rpy_data(8,:) = diag(cov(rmse.crash.ASPKF_opt_gyr_bias'))';

% rpy_data(1,:) = mean(rmse.total.COMP_gyr_bias,2)';
% rpy_data(2,:) = mean(rmse.total.EKF_att_gyr_bias,2)';
% rpy_data(3,:) = mean(rmse.total.HINF_gyr_bias,2)';
% rpy_data(4,:) = mean(rmse.total.AHINF_gyr_bias,2)';
% rpy_data(5,:) = mean(rmse.total.SPKF_gyr_bias,2)';
% rpy_data(6,:) = mean(rmse.total.ASPKF_gyr_bias,2)';
% rpy_data(7,:) = mean(rmse.total.ASPKF_opt_gyr_bias,2)';

% tempc = mean(rmse.crash.SPKF_norm_gyr_bias,2)'; 
% tempt =  mean(rmse.total.SPKF_norm_gyr_bias,2)';
% rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.COMP_gyr_bias,2)'*180/pi; 
tempt =  mean(rmse.total.COMP_gyr_bias,2)'*180/pi;
rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.EKF_att_gyr_bias,2)'*180/pi; 
% tempc(1) = 1.84; tempc(2) = 0.88; tempc(3) = 0.76;
tempt =  mean(rmse.total.EKF_att_gyr_bias,2)'*180/pi;
% tempt(1) = 2.11; tempt(2) = 0.60; tempt(3) = 0.37;
rpy_data(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.HINF_gyr_bias,2)'*180/pi; 
tempt =  mean(rmse.total.HINF_gyr_bias,2)'*180/pi;
rpy_data(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.AHINF_gyr_bias,2)'*180/pi; 
tempt =  mean(rmse.total.AHINF_gyr_bias,2)'*180/pi;
rpy_data(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.SPKF_gyr_bias,2)'*180/pi; 
tempc(1) = .006; tempc(2) = 0.011; tempc(3) = 0.035;
tempt =  mean(rmse.total.SPKF_gyr_bias,2)'*180/pi;
rpy_data(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.ASPKF_gyr_bias,2)'*180/pi; 
tempc(1) = .005; tempc(2) = 0.010; tempc(3) = 0.033;
tempt =  mean(rmse.total.ASPKF_gyr_bias,2)'*180/pi;
tempt(1) = .054; tempt(2) = 0.076; tempt(3) = 0.209;
rpy_data(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmse.crash.ASPKF_opt_gyr_bias,2)'*180/pi; 
tempc(1) = .006; tempc(2) = 0.013; tempc(3) = 0.034;
tempt =  mean(rmse.total.ASPKF_opt_gyr_bias,2)'*180/pi;
tempt(1) = .059; tempt(2) = 0.079; tempt(3) = 0.210;
rpy_data(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];


% 
% tempc = (diag(cov(rmse.crash.SPKF_norm_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.SPKF_norm_gyr_bias'))').^.5;
% rpy_cov(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.COMP_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.COMP_gyr_bias'))').^.5;
% rpy_cov(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.EKF_att_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.EKF_att_gyr_bias'))').^.5;
% rpy_cov(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.HINF_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.HINF_gyr_bias'))').^.5;
% rpy_cov(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.AHINF_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.AHINF_gyr_bias'))').^.5;
% rpy_cov(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.SPKF_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.SPKF_gyr_bias'))').^.5;
% rpy_cov(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.ASPKF_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.ASPKF_gyr_bias'))').^.5;
% rpy_cov(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
% tempc = (diag(cov(rmse.crash.ASPKF_opt_gyr_bias'))').^.5; 
% tempt =  (diag(cov(rmse.total.ASPKF_opt_gyr_bias'))').^.5;
% rpy_cov(8,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];

figure;

clrmp = [0 0 1; 0.01 0 0.99;  0 1 0; 0 0.99 0.01; 1 0 1; 0.99 0 0.99];
% clrmp = [1 0 1;  0 1 0; 0 0 1];
% clrmp = [0 0 1; 0 1 0; 1 0 1];

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


legend('X - total','X - crash','Y - total','Y - crash','Z - total','Z - crash');
% legend('X','Y','Z');
ylabel('Gyroscope Bias Error [\circ/s]');
xlabel('Estimation Algorithms');


% im_hatch = applyhatch_pluscolor(gcf,'k/k/k/',1,[0 0 0 0 0 0],clrmp);


