
%% Run Batch Sims
Batch = [];

for iPitch = -30:5:10 % pitchImpact in degrees
    for iVel = 0.5:0.5:2 %m/s
        videoFileName = strcat('BatchSim_Vel',num2str(iVel),'_Pitch',num2str(iPitch),'.avi');
        [CrashData.Hist, CrashData.Plot,CrashData.ImpactParams,CrashData.timeImpact,...
            CrashData.accelMagMax,CrashData.accelMagHorizMax,CrashData.accelDir_atPeak,...
            CrashData.angVels_atPeak,CrashData.angVels_avg] = startsimBatch_prescribeVelocity(iVel,iPitch);
        animate(1,CrashData.Hist,'I-10',CrashData.ImpactParams,CrashData.timeImpact,videoFileName);
        CrashData.pitch_atImpact = iPitch; %in degrees;
        CrashData.vel_atImpact = iVel;
        Batch = [Batch;CrashData];
        close all;
    end
end
        
%% Batch Sims to Excel
numFields = 13;
copy2excel = zeros(numel(Batch),numFields);
for iSim = 1:numel(Batch)
    copy2excel(iSim,1) = Batch(iSim).pitch_atImpact;
    copy2excel(iSim,2) = Batch(iSim).vel_atImpact;
    copy2excel(iSim,3) = Batch(iSim).accelMagMax;
    copy2excel(iSim,4) = Batch(iSim).accelMagHorizMax;
    copy2excel(iSim,5) = Batch(iSim).accelDir_atPeak(1);
    copy2excel(iSim,6) = Batch(iSim).accelDir_atPeak(2);
    copy2excel(iSim,7) = Batch(iSim).accelDir_atPeak(3);
    copy2excel(iSim,8) = Batch(iSim).angVels_atPeak(1);
    copy2excel(iSim,9) = Batch(iSim).angVels_atPeak(2);
    copy2excel(iSim,10) = Batch(iSim).angVels_atPeak(3);
    copy2excel(iSim,11) = Batch(iSim).angVels_avg(1);
    copy2excel(iSim,12) = Batch(iSim).angVels_avg(2);
    copy2excel(iSim,13) = Batch(iSim).angVels_avg(3);
end

%% table





        
        