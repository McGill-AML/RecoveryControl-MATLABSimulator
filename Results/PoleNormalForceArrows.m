%% Scaled down force magnitude Arrows plotted on the pole
% hold on;
% global poleRadius
% numPitch = 46;
% numOffset = 71;
% height = 3.0;
% axis('equal');
% axis([-.2,.2,-.2,.2]);
% grid on;
% n = 100;
% [X,Y,Z] = cylinder(poleRadius,n);
% Z = [zeros(1,n+1); height*ones(1,n+1)];
% polePoints = [X;Y;Z];
% surf(X,Y,Z);
% for iPitch = 1%:numPitch     % poleNormal plot for 15 degree Pitch
%     for iOffset = 1:numOffset
%         trial = 71*(iPitch-1)+iOffset;
%         disp(trial);
%         offset = cell2mat(Batch(trial,1));
% %         pitch = cell2mat(Batch(trial,2));
% %         success = cell2mat(Batch(trial,3));
%         normal = cell2mat(Batch(trial,4));
%         contactPoint = cell2mat(Batch(trial,5))*poleRadius;
% %     	positions = cell2mat(Batch(trial,6));
% %     	deflections = cell2mat(Batch(trial,7));
% %     	recoveryStages = cell2mat(Batch(trial,8));
% %         states = cell2mat(Batch(trial,9));
%        normalForces = cell2mat(Batch(trial,12));
%        ImpactBumper = cell2mat(Batch(trial,13));
% %        ImpactBumperFirst = min([find(ImpactBumper(:,1),1),find(ImpactBumper(:,2),1),find(ImpactBumper(:,3),1),find(ImpactBumper(:,4),1)]);
%         hold on;
%         Force =normal*max([max(normalForces(1,:)),max(normalForces(2,:)),max(normalForces(3,:)),max(normalForces(4,:))]);
%         Force=Force./500;
%         quiver3(contactPoint(1),contactPoint(2),height/2,Force(1),Force(2),Force(3),'MaxHeadSize',0.6,'LineWidth',1);
%         
%     end
% end

%% Normal directions plotted on the pole using offset colour map
hold on;
global poleRadius
numPitch = 46;
numOffset = 71;
height = 3.0;
axis('equal');
axis([-.2,.11,-.2,.2]);
grid on;
n = 100;
[X,Y,Z] = cylinder(poleRadius,n);
Z = [zeros(1,n+1); height*ones(1,n+1)];
polePoints = [X;Y;Z];
surf(X,Y,Z);
map=[];
for iPitch = 1%:numPitch     % poleNormal plot for 15 degree Pitch
    for iOffset = 1:2:numOffset
        trial = 71*(iPitch-1)+iOffset;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
%         pitch = cell2mat(Batch(trial,2));
%         success = cell2mat(Batch(trial,3));
        normal = cell2mat(Batch(trial,4));
        contactPoint = cell2mat(Batch(trial,5))*poleRadius;
%     	positions = cell2mat(Batch(trial,6));
%     	deflections = cell2mat(Batch(trial,7));
%     	recoveryStages = cell2mat(Batch(trial,8));
%         states = cell2mat(Batch(trial,9));
       normalForces = cell2mat(Batch(trial,12));
       ImpactBumper = cell2mat(Batch(trial,13));
%        ImpactBumperFirst = min([find(ImpactBumper(:,1),1),find(ImpactBumper(:,2),1),find(ImpactBumper(:,3),1),find(ImpactBumper(:,4),1)]);
        hold on;
        color = [abs((offset+1)/2) 0 0];
        map=[map;color];
        Force =normal;%*max([max(normalForces(1,:)),max(normalForces(2,:)),max(normalForces(3,:)),max(normalForces(4,:))]);
        Force=Force./10;
        quiver3(contactPoint(1),contactPoint(2),height/2,Force(1),Force(2),Force(3),'MaxHeadSize',0.3,'LineWidth',1.0,'color',color);
        
    end
end
colormap(map);
caxis([-1 1]);
colormap(map);
colorbar;