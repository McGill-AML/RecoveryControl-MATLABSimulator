function PlotRMSE = RMSE_to_plot(rmse)

temp = struct2cell(rmse);
tempcrash = struct2cell([temp{1,:}]);
PlotRMSE.crash.SPKF_quat = [tempcrash{1,:}];
PlotRMSE.crash.ASPKF_quat = [tempcrash{2,:}];
PlotRMSE.crash.EKF_att_quat = [tempcrash{3,:}];
PlotRMSE.crash.HINF_quat = [tempcrash{4,:}];
PlotRMSE.crash.COMP_quat = [tempcrash{5,:}];
PlotRMSE.crash.SPKF_full_quat = [tempcrash{6,:}];

PlotRMSE.crash.SPKF_gyr_bias = [tempcrash{7,:}];
PlotRMSE.crash.ASPKF_gyr_bias = [tempcrash{8,:}];
PlotRMSE.crash.EKF_att_gyr_bias = [tempcrash{9,:}];
PlotRMSE.crash.HINF_gyr_bias = [tempcrash{10,:}];
PlotRMSE.crash.COMP_gyr_bias = [tempcrash{11,:}];
PlotRMSE.crash.SPKF_full_gyr_bias = [tempcrash{12,:}];


tempcrash = struct2cell([temp{2,:}]);
PlotRMSE.not_crash.SPKF_quat = [tempcrash{1,:}];
PlotRMSE.not_crash.ASPKF_quat = [tempcrash{2,:}];
PlotRMSE.not_crash.EKF_att_quat = [tempcrash{3,:}];
PlotRMSE.not_crash.HINF_quat = [tempcrash{4,:}];
PlotRMSE.not_crash.COMP_quat = [tempcrash{5,:}];
PlotRMSE.not_crash.SPKF_full_quat = [tempcrash{6,:}];

PlotRMSE.not_crash.SPKF_gyr_bias = [tempcrash{7,:}];
PlotRMSE.not_crash.ASPKF_gyr_bias = [tempcrash{8,:}];
PlotRMSE.not_crash.EKF_att_gyr_bias = [tempcrash{9,:}];
PlotRMSE.not_crash.HINF_gyr_bias = [tempcrash{10,:}];
PlotRMSE.not_crash.COMP_gyr_bias = [tempcrash{11,:}];
PlotRMSE.not_crash.SPKF_full_gyr_bias = [tempcrash{12,:}];

tempcrash = struct2cell([temp{3,:}]);
PlotRMSE.total.SPKF_quat = [tempcrash{1,:}];
PlotRMSE.total.ASPKF_quat = [tempcrash{2,:}];
PlotRMSE.total.EKF_att_quat = [tempcrash{3,:}];
PlotRMSE.total.HINF_quat = [tempcrash{4,:}];
PlotRMSE.total.COMP_quat = [tempcrash{5,:}];
PlotRMSE.total.SPKF_full_quat = [tempcrash{6,:}];

PlotRMSE.total.SPKF_gyr_bias = [tempcrash{7,:}];
PlotRMSE.total.ASPKF_gyr_bias = [tempcrash{8,:}];
PlotRMSE.total.EKF_att_gyr_bias = [tempcrash{9,:}];
PlotRMSE.total.HINF_gyr_bias = [tempcrash{10,:}];
PlotRMSE.total.COMP_gyr_bias = [tempcrash{11,:}];
PlotRMSE.total.SPKF_full_gyr_bias = [tempcrash{12,:}];