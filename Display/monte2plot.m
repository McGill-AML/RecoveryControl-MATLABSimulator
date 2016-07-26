function Plot = monte2plot(Monte)
    Plot.trial = Monte.trial;
    % ICs
    temp = struct2cell(Monte.IC);
    % positions
    Plot.initPositions = [temp{1,:}];
    % body rates
    Plot.initBodyrates = [temp{2,:}];
    % incoming angles
    Plot.initAngles = [temp{3,:}];
    % speed
    Plot.initSpeeds = [temp{4,:}];   
   
    %% Performance measures
    recoveryStageSwitchTimes = Monte.recovery;
    
    num_iter = length(Monte.trial);
    % Extract number of trials that recovered 
    Plot.fractionReachedStageOne   = sum(recoveryStageSwitchTimes(:,1) > 0)/num_iter;
    Plot.fractionReachedStageTwo   = sum(recoveryStageSwitchTimes(:,2) > 0)/num_iter;
    Plot.fractionReachedStageThree = sum(recoveryStageSwitchTimes(:,3) > 0)/num_iter;
    Plot.fractionReachedStageFour  = sum(recoveryStageSwitchTimes(:,4) > 0)/num_iter;
 
    % get spectrum of recovery times
    Plot.timeUntilStageTwo   = recoveryStageSwitchTimes(:,2);
    Plot.timeUntilStageThree = recoveryStageSwitchTimes(:,3);
 
    Plot.heightLoss = Monte.heightLoss;
    Plot.horizLoss = Monte.horizLoss;
    
end
