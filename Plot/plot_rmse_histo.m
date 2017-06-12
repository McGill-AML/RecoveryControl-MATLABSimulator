% create histograms for crash or not crash data

rpy_data = zeros(8,6);   % indices for good crash data (:,[1:39, 41:57, 60:64,66:74])

% rpy_data(1,:) = mean(rmseEUL.crash.SPKF_norm_eul,2)'; 
% rpy_data(2,:) = mean(rmseEUL.crash.COMP_eul,2)';
% rpy_data(3,:) = mean(rmseEUL.crash.EKF_att_eul,2)';
% rpy_data(4,:) = mean(rmseEUL.crash.HINF_eul,2)';
% rpy_data(5,:) = mean(rmseEUL.crash.AHINF_eul,2)';
% rpy_data(6,:) = mean(rmseEUL.crash.SPKF_eul,2)';
% rpy_data(7,:) = mean(rmseEUL.crash.ASPKF_eul,2)';
% rpy_data(8,:) = mean(rmseEUL.crash.ASPKF_opt_eul,2)';

% rpy_data(1,:) = cov(rmseEUL.crash.SPKF_norm_eul,2)'; 
% rpy_data(2,:) = cov(rmseEUL.crash.COMP_eul,2)';
% rpy_data(3,:) = cov(rmseEUL.crash.EKF_att_eul,2)';
% rpy_data(4,:) = cov(rmseEUL.crash.HINF_eul,2)';
% rpy_data(5,:) = cov(rmseEUL.crash.AHINF_eul,2)';
% rpy_data(6,:) = cov(rmseEUL.crash.SPKF_eul,2)';
% rpy_data(7,:) = cov(rmseEUL.crash.ASPKF_eul,2)';
% rpy_data(8,:) = cov(rmseEUL.crash.ASPKF_opt_eul,2)';

tempc = mean(rmseEUL.crash.SPKF_norm_eul,2)'; 
tempt =  mean(rmseEUL.total.SPKF_norm_eul,2)';
rpy_data(1,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.COMP_eul,2)'; 
tempt =  mean(rmseEUL.total.COMP_eul,2)';
rpy_data(2,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.EKF_att_eul,2)'; 
tempt =  mean(rmseEUL.total.EKF_att_eul,2)';
rpy_data(3,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.HINF_eul,2)'; 
tempt =  mean(rmseEUL.total.HINF_eul,2)';
rpy_data(4,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.AHINF_eul,2)'; 
tempt =  mean(rmseEUL.total.AHINF_eul,2)';
rpy_data(5,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.SPKF_eul,2)'; 
tempt =  mean(rmseEUL.total.SPKF_eul,2)';
rpy_data(6,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.ASPKF_eul,2)'; 
tempt =  mean(rmseEUL.total.ASPKF_eul,2)';
rpy_data(7,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];
tempc = mean(rmseEUL.crash.ASPKF_opt_eul,2)'; 
tempt =  mean(rmseEUL.total.ASPKF_opt_eul,2)';
rpy_data(8,:) = [tempt(1), tempc(1), tempt(2), tempc(2), tempt(3), tempc(3)];

figure;
clrmp = [1 0 0; 0.99 0.01 0; 0 1 0; 0 0.99 0.01; 0 0 1; 0.01 0 0.99];
colormap(clrmp);
b_data = bar(rpy_data); grid on



set(gca,'FontSize',16);
set(gca,'XTickLabel', {'PX4','Comp','MEKF','H_\infty','AH_\infty','UKF','AUKF_1','AUKF_2'});


legend('Yaw - total','Yaw - crash','Pitch - total','Pitch - crash','Roll - total','Roll - crash');
% legend('Yaw','Pitch','Roll');
ylabel('Attitude Error [\circ]');
xlabel('Estimation Algorithms');


im_hatch = applyhatch_pluscolor(gcf,'k/k/k/',1,[0 0 0 0 0 0],clrmp);


