%sim_MonteCarlo.m Script to perform quadrotor Monte Carlo simulation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Script to perform quadrotor Monte Carlo simulation.
%                Initial collision conditions are randomized between certain
%                ranges. Data is saved to the variable "Batch". Keep note
%                of the default data values, which are unchanged if the 
%                quadrotor does not actually collide with the wall.
%                (e.g., horizLoss = 9999, xVelRecovered = 9999, etc.)
%-------------------------------------------------------------------------%

Batch = [];
numTrials = 1000;

for iBatch = 1:numTrials 
    disp(iBatch)
    rollImpact = 30*rand-15; %-15 to 15 deg
    pitchImpact = 90*rand-45; %-45 to 45 deg
    VxImpact = 2*rand+0.5; %0.5 to 2.5 m/s
    yawImpact = 90*rand-45; %-45 to 45 deg
    [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(VxImpact, rollImpact, pitchImpact, yawImpact);
    
    recoveryIdxs = [0;0;0]; %timeImpact, RS1 done, RS2 done
    recoveryTime = 9999;
    horizLoss = 9999;
    heightLoss = 9999;
    xVelRecovered = 9999;
    recoverySuccessful = 0;
    
    recoveryIdxs(1) = vlookup(CrashData.Plot.times,CrashData.timeImpact);
    if ~isempty(find(CrashData.Plot.recoveryStage == 1,1))
        allIdxs = find(CrashData.Plot.recoveryStage == 1);
        recoveryIdxs(2) = allIdxs(end);
    end
    if ~isempty(find(CrashData.Plot.recoveryStage == 3,1)) %successful recovery
        allIdxs = find(CrashData.Plot.recoveryStage == 2);
        recoveryIdxs(3) = allIdxs(end);
        recoveryTime = CrashData.Plot.times(recoveryIdxs(3)) - CrashData.timeImpact;
        horizLoss = sqrt(sum((CrashData.Plot.posns(1:2,recoveryIdxs(3)) - CrashData.Plot.posns(1:2,recoveryIdxs(1))).^2));
        heightLoss = CrashData.Plot.posns(3,recoveryIdxs(3)) - CrashData.Plot.posns(3,recoveryIdxs(1));
        xVelRecovered =  CrashData.Plot.posnDerivs(1,recoveryIdxs(3));
        recoverySuccessful = 1;
    end
        
    CrashData.roll_atImpact = rollImpact;
    CrashData.pitch_atImpact = pitchImpact; %in degrees;
    CrashData.vel_atImpact = VxImpact ;
    CrashData.yaw_atImpact = yawImpact;
    CrashData.recoveryIdxs = recoveryIdxs;
    CrashData.recoveryTime = recoveryTime;
    CrashData.horizLoss = horizLoss;
    CrashData.heightLoss = heightLoss;
    CrashData.xVelRecovered = xVelRecovered;
    CrashData.recoverySuccessful = recoverySuccessful;
    Batch = [Batch;CrashData];
end

save('MonteCarlo.mat')

