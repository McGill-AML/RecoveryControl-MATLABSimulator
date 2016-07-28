function Monte = updatemontecarlo(k, IC, Hist, Monte, FuzzyInfo, ImpactInfo, xVelocity)

    Monte.trial = [Monte.trial; k]; % record trial numbers
    Monte.impactOccured = [Monte.impactOccured; ImpactInfo.firstImpactOccured];
    Monte.impactDetected = [Monte.impactDetected; ImpactInfo.firstImpactDetected];
    Monte.IC = [Monte.IC; IC]; % record initial conditions for all trials
    Monte.xVelocity = [Monte.xVelocity; xVelocity];
    
    if ~Monte.impactOccured | ~Monte.impactDetected
        % either didn't crash or didn't detect it, give all zeros
        Monte.recovery = [Monte.recovery; [0 0 0 0]];
        Monte.heightLoss = [Monte.heightLoss; [0 0 0]];
        Monte.horizLoss = [Monte.horizLoss; [0 0 0]];
        Monte.fuzzyInput = [Monte.fuzzyInput; [0 0 0 0]];
        Monte.fuzzyOutput = [Monte.fuzzyOutput; 0];
        Monte.accelRef = [Monte.accelRef; [0 0 0]];
    else % impact occured and was detected, so control was attempted
        % record times at which each recovery stage was reached 
        % as well as time index
        recovered = [0 0 0 0];
        recoveryIndex = [0 0 0 0];
        
        for i = 1:length(Hist.times)
            switch Hist.controls(i).recoveryStage
                case 0
                case 1
                    if Hist.controls(i-1).recoveryStage < 1
                        recovered(1) = Hist.times(i);
                        recoveryIndex(1) = i;
                    end
                case 2
                    if Hist.controls(i-1).recoveryStage < 2
                        recovered(2) = Hist.times(i);
                        recoveryIndex(2) = i;
                    end
                case 3
                    if Hist.controls(i-1).recoveryStage < 3
                        recovered(3) = Hist.times(i);
                        recoveryIndex(3) = i;
                    end
                case 4
                    if Hist.controls(i-1).recoveryStage < 4
                        recovered(4) = Hist.times(i);
                        recoveryIndex(4) = i;
                    end
                otherwise 
                    error('Monte Carlo recovery times extraction failed!');
            end  
        end
        
        % Record the recovery times
        Monte.recovery = [Monte.recovery; recovered];
        
        % Record how much height and horizontal distance was lost after
        % recovery stages two, three and four were reached
        heightLoss = [0 0 0]; 
        horizLoss = [0 0 0];
        for i = 2:4 % after stage 2, 3 and 4.
            if recoveryIndex(i) ~= 0
                heightLoss(i-1) = Hist.poses(recoveryIndex(i)).posn(3) - Hist.poses(recoveryIndex(1)).posn(3);
                horizLoss(i-1) = sqrt((Hist.poses(recoveryIndex(i)).posn(1)-Hist.poses(recoveryIndex(1)).posn(1))^2 + ...
                                      (Hist.poses(recoveryIndex(i)).posn(2)-Hist.poses(recoveryIndex(1)).posn(2))^2);
            end
        end
        % elements 1 2 and 3 correspond to height/horizontal losses after reaching
        % stages 2 3 and 4
        Monte.heightLoss = [Monte.heightLoss; heightLoss];
        Monte.horizLoss  = [Monte.horizLoss;   horizLoss];
        
        % Record fuzzy inputs.
        fuzzyInputs = [FuzzyInfo.InputArray(1).value FuzzyInfo.InputArray(2).value ...
                       FuzzyInfo.InputArray(3).value FuzzyInfo.InputArray(4).value];
        Monte.fuzzyInput = [Monte.fuzzyInput; fuzzyInputs];
        
        % Record fuzzy output number.
        Monte.fuzzyOutput = [Monte.fuzzyOutput; FuzzyInfo(end).output]; 
      
        Monte.finalHorizVel = [Monte.finalHorizVel; ...
            sqrt(Hist.twists(end).posnDeriv(1)^2 + Hist.twists(end).posnDeriv(2)^2)];
            
    end
end
