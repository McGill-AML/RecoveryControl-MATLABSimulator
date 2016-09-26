Batch = [];

for iBatch = 1:1000
    disp(iBatch)
    rollImpact = 30*rand-15; %-15 to 15 deg
    pitchImpact = 90*rand-45; %-45 to 45 deg
    VxImpact = 2*rand+0.5; %0.5 to 2.5 m/s
    yawImpact = 90*rand-45; %-45 to 45 deg
    [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(VxImpact, rollImpact, pitchImpact, yawImpact);
    
    CrashData.roll_atImpact = rollImpact;
    CrashData.pitch_atImpact = pitchImpact; %in degrees;
    CrashData.vel_atImpact = VxImpact ;
    CrashData.yaw_atImpact = yawImpact;
    Batch = [Batch;CrashData];
end

save('/home/thread/fmchui/Spiri/Matlab-Simulator/Analysis/Fuzzy Logic Process/BatchSimData-14(montecarlo1000).mat')

%%

numBatch = 1000;
angle2normal_fromIMU_array = zeros(numBatch,1);
angle2normal_fromCM_array = zeros(numBatch,1);
angle2normal_corrected_array = zeros(numBatch,1);
angle2normal_corrected_array2 = zeros(numBatch,1);

plotcount = numBatch;

% figure()
subplot(1,2,2)
quiver(0,0,-1,0,0.5,'k','LineWidth',2)
hold on

for iBatch = 1:numBatch
    
    if sum(Batch(iBatch).ImpactIdentification.wallNormalWorld) ~= 0
        if abs(Batch(iBatch).pitch_atImpact) >= 51
%             figure;
%             Plot = Batch(iBatch).Plot;
%             plot(Plot.times,Plot.accelerometers);
%             hold on
%             plot(Plot.times,colnorm(Plot.accelerometers(1:2,:)),'m--')
%             plot([Batch(iBatch).timeImpact Batch(iBatch).timeImpact],ylim(),'k--')
%             plot(xlim(),[0.75 0.75],'k--');
%             title(num2str(iBatch));
            plotcount = plotcount - 1;
        
        else
        
            wallNormal_fromIMU = Batch(iBatch).ImpactIdentification.wallNormalWorld;
            wallNormal_fromCM = Batch(iBatch).ImpactIdentification.wallNormalWorldFromCM;    
            wallNormal_corrected = Batch(iBatch).ImpactIdentification.wallNormalWorldCorrected;
            wallNormal_corrected2 = Batch(iBatch).ImpactIdentification.wallNormalWorldCorrected2;


            angle2normal_fromIMU = atand(wallNormal_fromIMU(2)/wallNormal_fromIMU(1));
            angle2normal_fromCM = atand(wallNormal_fromCM(2)/wallNormal_fromCM(1));
            angle2normal_corrected = atand(wallNormal_corrected(2)/wallNormal_corrected(1));
            angle2normal_corrected2 = atand(wallNormal_corrected2(2)/wallNormal_corrected2(1));

            angle2normal_fromIMU_array(iBatch) = angle2normal_fromIMU;
            angle2normal_fromCM_array(iBatch) = angle2normal_fromCM;
            angle2normal_corrected_array(iBatch) = angle2normal_corrected;
            angle2normal_corrected_array2(iBatch) = angle2normal_corrected2;
            
%             quiver(0,0,wallNormal_fromIMU(1),wallNormal_fromIMU(2),0.25)
%             quiver(0,0, wallNormal_corrected2(1), wallNormal_corrected2(2),0.25)

        end
    end

end

%%
figure()
subplot(3,1,1)
hist(angle2normal_fromIMU_array,-90:5:90);
title('IMU located at RL Position');

subplot(3,1,2)
hist(angle2normal_fromCM_array,-90:5:90);
title('IMU located at CM Position');

% subplot(4,1,3)
% hist(angle2normal_corrected_array,-90:5:90);
% title('IMU located at RL Position, Accelerometer Corrected with True Angular Acceleration')

subplot(3,1,3)
hist(angle2normal_corrected_array2,-90:5:90);
title('IMU located at RL Position, Corrected');
% title('IMU located at RL Position, Accelerometer Corrected with Estimated Angular Acceleration')

suptitle({'Histogram of Angle Between Estimated and Real Wall Normal Vectors (deg)','1000 Monte Carlo Sims'})
