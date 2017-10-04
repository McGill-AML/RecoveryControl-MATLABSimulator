%% 3D plot for pole normal Components alongwith pitch variation
% % close all
% figure();
% hold on
% grid on
% numPitch = 46;
% numOffset = 71;
% xlim([-1 0]);
% ylim([-1 1]);
% zlim([3 6]);
% pbaspect([1 2 1]);
% map=[];
% 
% for iPitch = 1%:8:numPitch     
%     for iOffset = 1:71
%         for i=4:5
%             trial = 71*(iPitch-1)+iOffset;
%             disp(trial);
%             offset = cell2mat(Batch(trial,1));
%             pitch = cell2mat(Batch(trial,2));
%     %         success = cell2mat(Batch(trial,3));
%             normal = cell2mat(Batch(trial,i));
%     %         times = cell2mat(Batch(trial,5));
%     %         positions = cell2mat(Batch(trial,6));
%     %         deflections = cell2mat(Batch(trial,7));
%     %         recoveryStages = cell2mat(Batch(trial,8));
%             states = cell2mat(Batch(trial,9));
%             normalForces = cell2mat(Batch(trial,10));
%             timeImpact = cell2mat(Batch(trial,11));
% 
%             color = [abs((offset+1)/2) 0 0];
%             if iPitch==1
%                 map=[map;color];
%             end
%             if i==4
%                 scatter3(normal(1),normal(2),i,'MarkerFaceColor',color,'MarkerEdgeColor',color);
%             else
%                 scatter3(normal(1),normal(2),i,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 1 0]);
%             end
%             
%     %         scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor',color,'MarkerEdgeColor',color);
%         end
%     end
% end
% % legend([h2 h1],'Successful Recovery','Failed recovery');
% 
% for iOffset = 1:numOffset     % poleNormal plot for 15 degree Pitch
%     NORMAL1=[];
%     NORMAL2=[];
%     PITCH=[];
%     for iPitch = 1%:8:numPitch
%         for i=4:5
%             trial = 71*(iPitch-1)+iOffset;
%             disp(trial);
%             offset = cell2mat(Batch(trial,1));
%             pitch = cell2mat(Batch(trial,2));
%             normal = cell2mat(Batch(trial,i));
%             NORMAL1=[NORMAL1, normal(1)];
%             NORMAL2=[NORMAL2, normal(2)];
%             PITCH=[PITCH, i];
%         end
%     end
%     plot3(NORMAL1,NORMAL2,PITCH,'b-');
% end
% colormap(map);
% caxis([-1 1]);
% % colorbar;
% title('Pole Normal Components otained(color)vs Pole Normal Components Predicted(green) WithoutFriction');
% xlabel('Pole Normal X-Component');
% ylabel('Pole Normal Y-Component');
% zlabel('Pitch');

%% For 2D plot of Pole Components vs Offset
% close all
figure();
hold on
grid on
numPitch = 46;
numOffset = 71;
xlim([-1 0]);
ylim([-1 1]);
pbaspect([1 2 1]);
map=[];
offsetanomally=[];
for iPitch = 15%:numPitch     % poleNormal plot for 15 degree Pitch
    for iOffset = 1:71
        trial = 71*(iPitch-1)+iOffset;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
        pitch = cell2mat(Batch(trial,2));
        normal = cell2mat(Batch(trial,4));
        color = [abs((offset+1)/2) 0 0];
       
        map=[map;color];
        scatter(normal(1),normal(2),'MarkerFaceColor',color,'MarkerEdgeColor',color);
%         if(normal(2)>0)
%              offsetanomally=[offsetanomally;offset];
%         end
%         scatter(iPitch-1,-1+2*((iOffset-1)/(numOffset-1)),'MarkerFaceColor',color,'MarkerEdgeColor',color);
        
    end
end
colormap(map);
caxis([-1 1]);
title('Pole Normal Components With Varying Horizontal Offset(Pitch = 15^o)');
xlabel('Pole Normal X-Component');
ylabel('Pole Normal Y-Component');
% zlabel('Pitch');
colormap(map);
colorbar;