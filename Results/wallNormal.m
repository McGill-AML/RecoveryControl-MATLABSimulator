% load('iterate_no_recovery.mat');
close all
hold on
grid on
numPitch = 46;
numOffset = 71;
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
        
        color = [0.0 abs(normal(2)) 0.0];
%         scatter(normal(1),normal(2),'MarkerFaceColor',[(offset+1)/2 0 0],'MarkerEdgeColor',[(offset+1)/2 0 0]);
        scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor',color,'MarkerEdgeColor',color);
        
    end
end
% legend([h2 h1],'Successful Recovery','Failed recovery');
xlabel('Pitch (degrees)');
ylabel('Offset (normalized)');