function [ ] =snapshot( frame,Hist,sideview,ImpactParams,timeImpact)
%snapshot.m Displays single snapshot of simulation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: To display snapshot at a certain "time", use:
%                   snapshot(vlookup(Plot.times,time),Hist,...)
%                Example for snapshot of initial collision conditions:
%                   snapshot(vlookup(Plot.times,timeImpact),Hist,'ZX',ImpactParams,timeImpact);
%-------------------------------------------------------------------------%
global BUMP_POSNS BUMP_RADII BUMP_ANGLE

%% Save inputs to arrays
t = Hist.times;
stateHist = Hist.states';

fig = figure('Position',[962 25 960 949]);

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
axisMin = min([min(stateHist(:,7))-0.4,min(stateHist(:,8))-0.4,min(stateHist(:,9))-0.4]);
axisMax = max([max(stateHist(:,7))+0.4,max(stateHist(:,8))+0.4,max(stateHist(:,9))+0.4]);
% axisWidth = 1;
% 
% xAxisMin = mean([min(stateHist(:,7)),max(stateHist(:,7))])-axisWidth/2;
% xAxisMax = xAxisMin + axisWidth;
% % xAxisMin = 0.8; % 0.5750;
% % xAxisMax = xAxisMin + axisWidth;
% 
% yAxisMin = mean([min(stateHist(:,8)),max(stateHist(:,8))])-axisWidth/2;
% yAxisMax = yAxisMin + axisWidth;
% 
% zAxisMin = mean([min(stateHist(:,9)),max(stateHist(:,9))])-axisWidth/2 + 0.2;
% zAxisMax = zAxisMin + axisWidth;

%% Create world-frame wall points
impactYPosn = stateHist(vlookup(t,timeImpact),8);
impactZPosn = stateHist(vlookup(t,timeImpact),9);
% [wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,roundn(impactZPosn,-1)-axisWidth/2,roundn(impactYPosn,-1)+0.5,(axisWidth),1.8);
%                                                   bottom,center, height, width  
[wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,roundn(impactZPosn,-1)-(axisMax-axisMin)/2,roundn(impactYPosn,-1)+0.5,(axisMax-axisMin),2);
% [wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,0,0,1,1);

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

plotCircle3D(bumperCenterWorld1,bumperNormalWorld1,BUMP_RADII(1),2);
plotCircle3D(bumperCenterWorld2,bumperNormalWorld2,BUMP_RADII(2),2);
plotCircle3D(bumperCenterWorld3,bumperNormalWorld3,BUMP_RADII(3),2);
plotCircle3D(bumperCenterWorld4,bumperNormalWorld4,BUMP_RADII(4),2);

%% Plot body-fixed axes
axisXWorldPts = [comWorld axisXWorld];
axisYWorldPts = [comWorld axisYWorld];
axisZWorldPts = [comWorld axisZWorld];   

plot3(axisXWorldPts(1,:),axisXWorldPts(2,:),axisXWorldPts(3,:),'b-','LineWidth',1);
plot3(axisYWorldPts(1,:),axisYWorldPts(2,:),axisYWorldPts(3,:),'g-','LineWidth',1);
plot3(axisZWorldPts(1,:),axisZWorldPts(2,:),axisZWorldPts(3,:),'r-','LineWidth',1);

% %% Plot contact points
% for iBumper = 1:4
%    pointContactWorld = Hist.contacts(frame).point.contactWorld(:,iBumper);
%    plot3(pointContactWorld(1),pointContactWorld(2),pointContactWorld(3),'mX','MarkerSize',10);
% end

%% Plot wall
fill3(wallPts(1,:)',wallPts(2,:)',wallPts(3,:)','r','FaceAlpha',0.4,'LineWidth',2,'EdgeColor','r');
%    plot3(wallLines(1,1:2)',wallLines(2,1:2)',wallLines(3,1:2)','r-');
%    plot3(wallLines(1,3:4)',wallLines(2,3:4)',wallLines(3,3:4)','r-');

%% Figure settings
% axis([xAxisMin,xAxisMax,yAxisMin,yAxisMax,zAxisMin,zAxisMax]);
axis([axisMin,axisMax,axisMin,axisMax,axisMin,axisMax]);
xlhand = xlabel('X(m)','Interpreter','LaTex');
ylhand = ylabel('Y(m)','Interpreter','LaTex');
zlhand = zlabel('Z(m)','Interpreter','LaTex');
set(xlhand,'fontsize',14);
set(ylhand,'fontsize',14);
set(zlhand,'fontsize',14);

% title(strcat('t = ',num2str(t(frame),'%.2f'),' s'));
setsimulationview(sideview);

grid on;
axis square;
drawnow;
hold off;

end

