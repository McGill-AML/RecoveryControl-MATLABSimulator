
%% Run Batch Sims
Batch = [];

% for iPitch = -30:10:30 % pitchImpact in degrees
%     for iVel = 0.5:0.5:3 %m/s
%         for iYaw = 0:45:45 %deg
            for iBatch = 1:500
            iPitch = 60*rand-30; %-30 to 30 deg
            iVel = 2.5*rand +0.5; %0.5 to 3 m/s
            iYaw = 45*rand; %0 to 45 deg
    %         videoFileName = strcat('BatchSim_Yaw45_Vel',num2str(iVel),'_Pitch',num2str(iPitch),'.avi');
    %         [CrashData.Hist, CrashData.Plot,CrashData.ImpactParams,CrashData.timeImpact,...
    %             CrashData.accelMagMax,CrashData.accelMagHorizMax,CrashData.accelDir_atPeak,...
    %             CrashData.angVels_atPeak,CrashData.angVels_avg] = startsimBatch_prescribeVelocity(iVel,iPitch);
            [CrashData.Hist, CrashData.Plot,CrashData.ImpactParams,CrashData.timeImpact] = startsimBatch_prescribeVelocity(iVel,iPitch,iYaw);
    %         animate(1,CrashData.Hist,'I-10',CrashData.ImpactParams,CrashData.timeImpact,videoFileName);
            CrashData.pitch_atImpact = iPitch; %in degrees;
            CrashData.vel_atImpact = iVel;
            CrashData.yaw_atImpact = iYaw;
            Batch = [Batch;CrashData];
            close all;
            end
%         end
%     end
% end
        
% %% Batch Sims to Excel
% numFields = 13;
% copy2excel = zeros(numel(Batch),numFields);
% for iSim = 1:numel(Batch)
%     copy2excel(iSim,1) = Batch(iSim).pitch_atImpact;
%     copy2excel(iSim,2) = Batch(iSim).vel_atImpact;
%     copy2excel(iSim,3) = Batch(iSim).accelMagMax;
%     copy2excel(iSim,4) = Batch(iSim).accelMagHorizMax;
%     copy2excel(iSim,5) = Batch(iSim).accelDir_atPeak(1);
%     copy2excel(iSim,6) = Batch(iSim).accelDir_atPeak(2);
%     copy2excel(iSim,7) = Batch(iSim).accelDir_atPeak(3);
%     copy2excel(iSim,8) = Batch(iSim).angVels_atPeak(1);
%     copy2excel(iSim,9) = Batch(iSim).angVels_atPeak(2);
%     copy2excel(iSim,10) = Batch(iSim).angVels_atPeak(3);
%     copy2excel(iSim,11) = Batch(iSim).angVels_avg(1);
%     copy2excel(iSim,12) = Batch(iSim).angVels_avg(2);
%     copy2excel(iSim,13) = Batch(iSim).angVels_avg(3);
% end

%% table





        
        