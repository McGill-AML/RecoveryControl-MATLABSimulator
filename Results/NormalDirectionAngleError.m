%% Angle Error - Estimated and Expected normal
% close all
figure();
hold on
grid on
numPitch = 46;
numOffset = 71;
heightLoss=[];
trial=0;
failures = 0;
for iPitch = 1%1:numPitch
    THETA = [];
    OFFSET = [];
    for iOffset = 1:numOffset
        trial = trial+1;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
        pitch = cell2mat(Batch(trial,2));
        success = cell2mat(Batch(trial,3));
        estimatedNormal = cell2mat(Batch(trial,4));
        expectedNormal = cell2mat(Batch(trial,5));
        positions = cell2mat(Batch(trial,7));  
        heightLoss =[heightLoss, positions(3,1)-min(positions(3,:))];
        
        Theta = acos(dot(estimatedNormal, expectedNormal));
        Theta = rad2deg(Theta);
        THETA = [THETA;Theta];
        OFFSET = [OFFSET;offset];
        
    end
end
plot(OFFSET,THETA);
xlim([-1.1,1.1]);
ylim([0,50]);
title('Angle Deviation between Estimated and Expected Normal');
ylabel('Angle Deviation from Expected Normal (degrees)');
xlabel('Horizontal Offset (meters)');