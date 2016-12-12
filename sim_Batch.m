%sim_Batch.m Script to perform batch quadrotor simulation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Script to perform batch quadrotor simulation.
%                Initial collision conditions are prescribed at different
%                intervals for each variable. However, prescribed initial
%                conditions will likely not be perfectly achieved, as the
%                simulation is dynamic. For validation, using sim_MonteCarlo.m
%                is probably better. 
%-------------------------------------------------------------------------%

Batch = [];
iBatch = 0;
crash_array = [];

for vImpact = -2:0.05:3
    for inclinationImpact = -30:5:30
        for yawImpact = 0:45:45
            if abs(yawImpact) == 45
                angle = (inclinationImpact - 0.0042477)/1.3836686;
                rollImpact = -angle;
                pitchImpact = -angle;
            elseif yawImpact == 0
                rollImpact = 0;
                pitchImpact = -inclinationImpact;
            end   
            
            iBatch = iBatch + 1;
            disp(iBatch)

            [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(vImpact, rollImpact, pitchImpact, yawImpact,iBatch);

            CrashData.roll_atImpact = rollImpact;
            CrashData.pitch_atImpact = pitchImpact; %in degrees;
            CrashData.vel_atImpact = vImpact ;
            CrashData.yaw_atImpact = yawImpact;
            Batch = [Batch;CrashData];
            
            crash_array(iBatch) = 0;
            if CrashData.Plot.posns(3,end) <=0
                if CrashData.Plot.times(end) - CrashData.timeImpact <= 0.9
                    crash_array(iBatch) = 1;
                end
            end
                    
        end
    end
end

save('Batch.mat')
