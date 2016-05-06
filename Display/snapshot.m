function [ ] =snapshot( frame,Hist,sideview,ImpactParams,timeImpact)
 
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global BUMP_POSNS BUMP_RADIUS BUMP_ANGLE

%% Save inputs to arrays
t = Hist.times;
stateHist = Hist.states';

figure('Position',[962 25 960 949]);

%% Define body-fixed points and axes

% Body-fixed bumper centers
bumperCenter1 = BUMP_POSNS(:,1);
bumperCenter2 = BUMP_POSNS(:,2);
bumperCenter3 = BUMP_POSNS(:,3);
bumperCenter4 = BUMP_POSNS(:,4);

% Body-fixed bumper normal vectors
bumperNormalBody1 = invar2rotmat('Z',deg2rad(45))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
bumperNormalBody2 = invar2rotmat('Z',deg2rad(135))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
bumperNormalBody3 = invar2rotmat('Z',deg2rad(-135))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
bumperNormalBody4 = invar2rotmat('Z',deg2rad(-45))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];

% Body-fixed points of Spiri body (pentagon)
load('locations2');
clear p1 p2 p3 p4;

bodyPoint1 = [0.08;0.0115;0]-CoM;
bodyPoint2 = [0;0.0575;0]-CoM;
bodyPoint3 = [-0.1;0;0]-CoM;
bodyPoint4 = [0;-0.0575;0]-CoM;
bodyPoint5 = [0.08;-0.0115;0]-CoM;


% Body-fixed origin (CoM) and x,y,z axes
comBody = [0;0;0];
axisXBody = [0.1;0;0];
axisYBody = [0;0.1;0];
axisZBody = [0;0;0.1];

%%  Calculate axes ranges for plotting
% axisMin = min([min(stateHist(:,7))-0.4,min(stateHist(:,8))-0.4,min(stateHist(:,9))-0.4]);
% axisMax = max([max(stateHist(:,7))+0.4,max(stateHist(:,8))+0.4,max(stateHist(:,9))+0.4]);

axisWidth = 1;

% xAxisMin = mean([min(stateHist(:,7)),max(stateHist(:,7))])-axisWidth/2;
% xAxisMax = xAxisMin + axisWidth;
xAxisMin = 0.8; % 0.5750;
xAxisMax = xAxisMin + axisWidth;

yAxisMin = mean([min(stateHist(:,8)),max(stateHist(:,8))])-axisWidth/2;
yAxisMax = yAxisMin + axisWidth;

zAxisMin = mean([min(stateHist(:,9)),max(stateHist(:,9))])-axisWidth/2 + 0.2;
zAxisMax = zAxisMin + axisWidth;

%% Create world-frame wall points
impactYPosn = stateHist(vlookup(t,timeImpact),8);
impactZPosn = stateHist(vlookup(t,timeImpact),9);
% [wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,roundn(impactZPosn,-1)-axisWidth/2,roundn(impactYPosn,-1)+0.5,(axisWidth),1.8);
                                                  %bottom,center, height, width  
[wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,zAxisMin,mean([yAxisMin,yAxisMax]),axisWidth, axisWidth);
    
%% Plot Frame
%% Rotate body-fixed points to world-frame points
q = [stateHist(frame,10);stateHist(frame,11);stateHist(frame,12);stateHist(frame,13)];
q = q/norm(q);
rotMat = quat2rotmat(q);
translation = [stateHist(frame,7);stateHist(frame,8);stateHist(frame,9)];

bodyPointWorld1 = rotMat'*bodyPoint1 + translation;
bodyPointWorld2 = rotMat'*bodyPoint2 + translation;
bodyPointWorld3 = rotMat'*bodyPoint3 + translation;
bodyPointWorld4 = rotMat'*bodyPoint4 + translation;
bodyPointWorld5 = rotMat'*bodyPoint5 + translation;

bumperCenterWorld1 = rotMat'*bumperCenter1 + translation;
bumperCenterWorld2 = rotMat'*bumperCenter2 + translation;
bumperCenterWorld3 = rotMat'*bumperCenter3 + translation;
bumperCenterWorld4 = rotMat'*bumperCenter4 + translation;

comWorld = rotMat'*comBody + translation;
axisXWorld = rotMat'*axisXBody + translation;
axisYWorld = rotMat'*axisYBody + translation;
axisZWorld = rotMat'*axisZBody + translation;

bodyPts = [bodyPointWorld1 bodyPointWorld2 bodyPointWorld3 bodyPointWorld4 bodyPointWorld5 bodyPointWorld1];

%% Plot Spiri body points and CoM
plot3(bodyPts(1,:),bodyPts(2,:),bodyPts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
hold on;
plot3(translation(1),translation(2),translation(3),'rx','MarkerSize',8); %Centre of mass

%% Plot Spiri 2-d bumpers
bumperNormalWorld1 = rotMat'*bumperNormalBody1;
bumperNormalWorld2 = rotMat'*bumperNormalBody2;
bumperNormalWorld3 = rotMat'*bumperNormalBody3;
bumperNormalWorld4 = rotMat'*bumperNormalBody4;

plotCircle3D(bumperCenterWorld1,bumperNormalWorld1,BUMP_RADIUS);
plotCircle3D(bumperCenterWorld2,bumperNormalWorld2,BUMP_RADIUS);
plotCircle3D(bumperCenterWorld3,bumperNormalWorld3,BUMP_RADIUS);
plotCircle3D(bumperCenterWorld4,bumperNormalWorld4,BUMP_RADIUS);

%% Plot body-fixed axes
axisXWorldPts = [comWorld axisXWorld];
axisYWorldPts = [comWorld axisYWorld];
axisZWorldPts = [comWorld axisZWorld];   

plot3(axisXWorldPts(1,:),axisXWorldPts(2,:),axisXWorldPts(3,:),'r-','LineWidth',1);
plot3(axisYWorldPts(1,:),axisYWorldPts(2,:),axisYWorldPts(3,:),'g-','LineWidth',1);
plot3(axisZWorldPts(1,:),axisZWorldPts(2,:),axisZWorldPts(3,:),'b-','LineWidth',1);

%% Plot contact points
for iBumper = 1:4
   pointContactWorld = Hist.contacts(frame).point.contactWorld(:,iBumper);
   plot3(pointContactWorld(1),pointContactWorld(2),pointContactWorld(3),'mX','MarkerSize',10);
end

%% Plot wall
fill3(wallPts(1,:)',wallPts(2,:)',wallPts(3,:)','r','FaceAlpha',0.4);
%    plot3(wallLines(1,1:2)',wallLines(2,1:2)',wallLines(3,1:2)','r-');
%    plot3(wallLines(1,3:4)',wallLines(2,3:4)',wallLines(3,3:4)','r-');

%% Figure settings
axis([xAxisMin,xAxisMax,yAxisMin,yAxisMax,zAxisMin,zAxisMax]);
xlhand = xlabel('X(m)','Interpreter','LaTex');
ylhand = ylabel('Y(m)','Interpreter','LaTex');
zlhand = zlabel('Z(m)','Interpreter','LaTex');
set(xlhand,'fontsize',14);
set(ylhand,'fontsize',14);
set(zlhand,'fontsize',14);
% title(strcat('t = ',num2str(t(frame),'%.2f'),' s'));

if sideview == 'XZ'
    view([0 0]); %view XZ plane
elseif sideview == 'ZX'
   view(-180,0);       
elseif sideview == 'YZ'
    view(90, 0); %view YZ plane
elseif sideview =='ZY'
    view(-90,0);
elseif sideview == 'XY'
    view([0 0 1]);
elseif sideview == 'V1'
    view([-14.5,6]);
elseif sideview == 'V2'
    view([-19.5,28]);
elseif sideview == 'V3'
    view(-130,14);
elseif sideview == 'C3'
    view(-140,26);
elseif sideview == 'C5'
    view(-176,30);
elseif sideview == 'C6'
    view(-177,6);
elseif sideview == 'C7'
    view(-178,8);
elseif sideview == 'C9'
    view(-180,8);
elseif sideview == 'CT' %crash Ten
    view(-177,20);       
elseif sideview == 'VV'
    view([-180,26]);
end

grid on;
axis square;
drawnow;
hold off;



end

