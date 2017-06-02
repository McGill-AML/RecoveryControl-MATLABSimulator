% load('june_2_with_recovery.mat');
close all
% hold on
% grid on
numPitch = 46;
numOffset = 36;
for iPitch = 45%1:numPitch
    for iOffset = 19%1:numOffset
        trial = numOffset*(iPitch-1)+iOffset
        disp(trial)
        offset = cell2mat(Batch(trial,1))
        pitch = cell2mat(Batch(trial,2))
        success = cell2mat(Batch(trial,3));
        normal = cell2mat(Batch(trial,4));
        times = cell2mat(Batch(trial,5));
        positions = cell2mat(Batch(trial,6));
%         deflections = cell2mat(Batch(trial,7));
%         recoveryStages = cell2mat(Batch(trial,8));
%         states = cell2mat(Batch(trial,9));
%         normalForces = cell2mat(Batch(trial,10));
%         timeImpact = cell2mat(Batch(trial,11));
        
%         if positions(3,end) > 1.0
%             h1 = scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor','b','MarkerEdgeColor','b');
%         else
%             h2 = scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor','r','MarkerEdgeColor','r');
%         end
%         color = [abs(normal(1)) abs(normal(2)) 0.0];
% %         scatter(normal(1),normal(2),'MarkerFaceColor',[(offset+1)/2 0 0],'MarkerEdgeColor',[(offset+1)/2 0 0]);
%         
    end
end
% end
% title('With Recovery Control')
% legend([h1 h2],'Lost less than 1 meter height','Lost less than 1 meter height');
% xlabel('Pitch (degrees)');
% ylabel('Offset (normalized)');