function [ Plot ] = hist2plot( Hist )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Plot.times = Hist.times;

temp = struct2cell(Hist.propStates);
Plot.propRpms = [temp{1,:}];
Plot.propRpmDerivs = [temp{2,:}];

temp = struct2cell(Hist.poses);
Plot.posns = [temp{1,:}];
Plot.eulerAngles = [temp{3,:}];
Plot.quaternions = [temp{2,:}];
Plot.quaternionDerivs = Hist.stateDerivs(10:13,:);

temp = struct2cell(Hist.twists);
Plot.angVels = [temp{4,:}];
Plot.linVels = [temp{1,:}];

Plot.posnDerivs = Hist.stateDerivs(7:9,:);
Plot.bodyAccs = Hist.stateDerivs(1:3,:);

temp = struct2cell(Hist.contacts);
% Plot.normalForces = [temp{5,:}];
Plot.normalForces = reshape([temp{6,:}],[4,size(temp,2)]);
temp2 = [temp{8,:}];
Plot.slidingDirectionWorlds_bump1 =  temp2(:,1:4:end)';
Plot.slidingDirectionWorlds_bump2 =  temp2(:,2:4:end)';
Plot.slidingDirectionWorlds_bump3 =  temp2(:,3:4:end)';
Plot.slidingDirectionWorlds_bump4 =  temp2(:,4:4:end)';

temp2 = [temp{9,:}];
Plot.tangentialForceWorlds_bump1 = temp2(:,1:4:end)';
Plot.tangentialForceWorlds_bump2 = temp2(:,2:4:end)';
Plot.tangentialForceWorlds_bump3 = temp2(:,3:4:end)';
Plot.tangentialForceWorlds_bump4 = temp2(:,4:4:end)';

temp2 = [temp{7,:}];
Plot.contactPtVelocityWorlds_bump1 = temp2(:,1:4:end)';
Plot.contactPtVelocityWorlds_bump2 = temp2(:,2:4:end)';
Plot.contactPtVelocityWorlds_bump3 = temp2(:,3:4:end)';
Plot.contactPtVelocityWorlds_bump4 = temp2(:,4:4:end)';

temp2 = [temp{3,:}];
temp3 = struct2cell(temp2');
temp4 = [temp3{2,:}];
Plot.contactPointWorlds_bump1 = temp4(:,1:4:end)';
Plot.contactPointWorlds_bump2 = temp4(:,2:4:end)';
Plot.contactPointWorlds_bump3 = temp4(:,3:4:end)';
Plot.contactPointWorlds_bump4 = temp4(:,4:4:end)';
temp4 = [temp3{3,:}];
Plot.contactPointBodys_bump1 = temp4(:,1:4:end)';
Plot.contactPointBodys_bump2 = temp4(:,2:4:end)';
Plot.contactPointBodys_bump3 = temp4(:,3:4:end)';
Plot.contactPointBodys_bump4 = temp4(:,4:4:end)';

Plot.defls = reshape([temp{4,:}],[4,size(temp,2)])';

Plot.muSlidings = [temp{11,:}];

temp = struct2cell(Hist.controls);
Plot.errEulers = [temp{5,:}];
Plot.desEulers = [temp{13,:}];
Plot.desYawDerivs = [temp{14,:}];
Plot.controlAccDes = [temp{3,:}];

Plot.recoveryStage = [temp{21,:}];
Plot.accelRef = [temp{22,:}];
Plot.errQuat = [temp{4,:}];

temp2 = [temp{2,:}]; %Control.twist
temp3 = struct2cell(temp2');
Plot.controlAngVels = [temp3{4,:}];

%sensor stuff
temp = struct2cell(Hist.sensors);
Plot.accelerometers = [temp{1,:}];
Plot.gyros = [temp{2,:}];
Plot.mag = [temp{3,:}];
Plot.gps = [temp{4,:}];
Plot.baro = [temp{5,:}];

temp = struct2cell(Hist.crash);
Plot.crash = [temp{4,:}];
Plot.time_since_cont = [temp{5,:}];

temp = struct2cell(Hist.EKF);
temp2 = struct2cell([temp{1,:}]);
Plot.EKF_pos = [temp2{1,:}];
Plot.EKF_vel = [temp2{2,:}];
Plot.EKF_acc_bias = [temp2{3,:}];
tempP = [temp{2,:}];
for ii = 1:9
    Plot.EKF_P_hat(ii,:) = tempP(ii,ii:9:end);
end

temp = struct2cell(Hist.AEKF);
temp2 = struct2cell([temp{1,:}]);
Plot.AEKF_pos = [temp2{1,:}];
Plot.AEKF_vel = [temp2{2,:}];
Plot.AEKF_acc_bias = [temp2{3,:}];
Plot.AEKF_G_k = [temp{8,:}];
for ii = 1:length([temp{9,:}])
    Plot.AEKF_innov(ii) = sum(temp{9,ii});
end
tempP = [temp{2,:}];
for ii = 1:9
    Plot.AEKF_P_hat(ii,:) = tempP(ii,ii:9:end);
end

temp = struct2cell(Hist.SPKF);
temp2 = struct2cell([temp{2,:}]);
Plot.SPKF_quat = [temp2{1,:}];
Plot.SPKF_omega = [temp2{2,:}];
Plot.SPKF_gyr_bias = [temp2{3,:}];
% tempP = [temp{2,:}];
% for ii = 1:6
%     Plot.SPKF_P_hat(ii,:) = tempP(ii,ii:6:end);
% end

temp = struct2cell(Hist.ASPKF);
temp2 = struct2cell([temp{2,:}]);
Plot.ASPKF_quat = [temp2{1,:}];
Plot.ASPKF_omega = [temp2{2,:}];
Plot.ASPKF_gyr_bias = [temp2{3,:}];
Plot.ASPKF_G_k = [temp{8,:}];
for ii = 1:length([temp{9,:}])
    Plot.ASPKF_innov(ii) = sum(temp{9,ii});
end
% tempP = [temp{2,:}];
% for ii = 1:6
%     Plot.ASPKF_P_hat(ii,:) = tempP(ii,ii:6:end);
% end

% %% Simulate accelerometer data
% % Reference: "Small Unmanned Aircraft: Theory and Practice" - Beard &
% % McLain, 2012 - 7.1 Accelerometers
% global g
% Plot.accelerometers = zeros(3,numel(Plot.times));
% Plot.worldAccs = zeros(3,numel(Plot.times));
% for iData = 1:numel(Plot.times)    
%     rotMat = quat2rotmat(Plot.quaternions(:,iData));
%     Plot.accelerometers(:,iData) = invar2rotmat('x',pi)*(rotMat*[0;0;g] + Plot.bodyAccs(:,iData) + cross(Plot.angVels(:,iData),Plot.linVels(:,iData)))/g;
%     Plot.worldAccs(:,iData) = rotMat'*Plot.bodyAccs(:,iData);
% end

% %% Add noise to accelerometer data
% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_02_25/MATLAB_processed_data/sensor_covs.mat');
% numSamples = size(Hist.times,1);
% numStates = 3;
% sigma = C_accel; %Covariance matrix
% R = chol(sigma);
% noise = randn(numSamples,numStates)*R;
% Plot.accelerometersNoisy = Plot.accelerometers + noise';
% 
% %% Simulate gyroscope data
% Plot.gyros = sign(rad2deg(Plot.angVels)).*min(abs(rad2deg(Plot.angVels)),240);
% Plot.gyros(3,:) = -Plot.gyros(3,:);
% %% Add noise to gyroscope data
% numSamples = size(Hist.times,1);
% numStates = 3;
% sigma = C_gyro; %Covariance matrix
% R = chol(sigma);
% noise = randn(numSamples,numStates)*R;
% Plot.gyrosNoisy = Plot.gyros + noise';


end

