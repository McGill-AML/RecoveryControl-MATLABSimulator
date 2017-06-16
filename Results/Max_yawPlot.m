close all
hold on
grid on
numPitch = 46;
numOffset = 71;
% xlim([-1 0]);
% ylim([-1 1]);
% pbaspect([1 2 1]);
% map=[];
SET=[];
for iPitch = 16%:numPitch     % poleNormal plot for 15 degree Pitch
    for iOffset = 1:numOffset
        trial = numOffset*(iPitch-1)+iOffset;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
%         pitch = cell2mat(Batch(trial,2));
%         success = cell2mat(Batch(trial,3));
%         normal = cell2mat(Batch(trial,4));
%         times = cell2mat(Batch(trial,5));
%         positions = cell2mat(Batch(trial,6));
%         deflections = cell2mat(Batch(trial,7));
%         recoveryStages = cell2mat(Batch(trial,8));
        states = cell2mat(Batch(trial,10));
%         normalForces = cell2mat(Batch(trial,10));
        timeImpact = cell2mat(Batch(trial,11));
        Impacts = cell2mat(Batch(trial,14));
        maxYaw=max(abs(states(6,1:210)));
        if Impacts == 1
            h1 = scatter(offset,maxYaw,'MarkerEdgeColor','b');
        elseif (Impacts == 2)
            h2 = scatter(offset,maxYaw,'MarkerEdgeColor','r');
        else
            h3 = scatter(offset,maxYaw,'MarkerEdgeColor',[0.4 1 0]);
        end
        
%         SET=[SET; [offset,maxYaw]];
        
%         scatter(offset,maxYaw,'bo');
%         scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor',color,'MarkerEdgeColor',color);
        
    end
end
% colormap(map);
% caxis([-1 1]);
% plot(SET(:,1),SET(:,2));
legend([h1 h2 h3],'1 Impact','2 Impacts','3 or more Impacts');
title('Max Yaw Rate vs Offset -15 Degrees Pitch (Without Recovery Controller)');
xlabel('Horizontal Offset (meters)');
ylabel('Maximum Yaw Rate (rad/sec)');
