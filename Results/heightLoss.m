% load('iterate_no_recovery.mat');
%%
close all
hold on
grid on
numPitch = 46;
numOffset = 71;
maxHeightLoss = 0;
counter = 0
heightLosses = [];
weirdTrials = [];
for iPitch = 1:numPitch
    for iOffset = 1:numOffset
        trial = 71*(iPitch-1)+iOffset;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
        pitch = cell2mat(Batch(trial,2));
        success = cell2mat(Batch(trial,3));
        normal = cell2mat(Batch(trial,4));
        times = cell2mat(Batch(trial,5));
        positions = cell2mat(Batch(trial,6));
        deflections = cell2mat(Batch(trial,7));
        recoveryStages = cell2mat(Batch(trial,8));
        states = cell2mat(Batch(trial,9));
        normalForces = cell2mat(Batch(trial,10));
        timeImpact = cell2mat(Batch(trial,11));
        
        newHeightLoss = (2.0 - positions(3,end));

        if (newHeightLoss < -10)
            weirdTrials = [weirdTrials; offset pitch newHeightLoss];
        end
        heightLosses = [heightLosses; newHeightLoss];
        
%         color = [abs(normal(1)) abs(normal(2)) 0.0];
%         scatter(normal(1),normal(2),'MarkerFaceColor',[(offset+1)/2 0 0],'MarkerEdgeColor',[(offset+1)/2 0 0]);
%         scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor',color,'MarkerEdgeColor',color);
        
    end
end
histogram(heightLosses, 100);
counter
% legend([h2 h1],'Successful Recovery','Failed recovery');
xlabel('Pitch (degrees)');
ylabel('Offset (normalized)');