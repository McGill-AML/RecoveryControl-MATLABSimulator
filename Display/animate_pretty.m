function [ ] =animate_pretty( recordAnimation,Hist,sideview,ImpactParams,timeImpact,videoFileName )
 
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global BUMP_RADIUS BUMP_ANGLE BUMP_POSNS

%% Define recording parameters
if recordAnimation == 1
    recordRate = 50; %Hz
    writerObj = VideoWriter(videoFileName);
    writerObj.FrameRate = recordRate;
    open(writerObj);
end

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
axisMin = min([min(stateHist(:,7))-0.4,min(stateHist(:,8))-0.4,min(stateHist(:,9))-0.4]);
axisMax = max([max(stateHist(:,7))+0.4,max(stateHist(:,8))+0.4,max(stateHist(:,9))+0.4]);

%% Create world-frame wall points
impactYPosn = stateHist(vlookup(t,timeImpact),8);
impactZPosn = stateHist(vlookup(t,timeImpact),9);
% [wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,roundn(impactZPosn,-1)-(axisMax-axisMin)/2,roundn(impactYPosn,-1)+0.5,(axisMax-axisMin),2);
% [wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,0.4,0.4,0.9,0.9);
[wallPts, wallLines] = getwallpts(ImpactParams.wallLoc,ImpactParams.wallPlane,0,0,0.9,0.9);
                                                  %bottom,center, height, width  

%% Animate!
for iFrame = 1:1:154%173%size(t,1)
    %% Rotate body-fixed points to world-frame points
    q = [stateHist(iFrame,10);stateHist(iFrame,11);stateHist(iFrame,12);stateHist(iFrame,13)];
    q = q/norm(q);
    rotMat = quat2rotmat(q);
    translation = [stateHist(iFrame,7);stateHist(iFrame,8);stateHist(iFrame,9)];

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

    plotCircle3D(bumperCenterWorld1,bumperNormalWorld1,BUMP_RADIUS,2);
    plotCircle3D(bumperCenterWorld2,bumperNormalWorld2,BUMP_RADIUS,2);
    plotCircle3D(bumperCenterWorld3,bumperNormalWorld3,BUMP_RADIUS,1);
    plotCircle3D(bumperCenterWorld4,bumperNormalWorld4,BUMP_RADIUS,1);

    %% Plot body-fixed axes
    axisXWorldPts = [comWorld axisXWorld];
    axisYWorldPts = [comWorld axisYWorld];
    axisZWorldPts = [comWorld axisZWorld];   

    plot3(axisXWorldPts(1,:),axisXWorldPts(2,:),axisXWorldPts(3,:),'r-','LineWidth',1);
    plot3(axisYWorldPts(1,:),axisYWorldPts(2,:),axisYWorldPts(3,:),'g-','LineWidth',1);
    plot3(axisZWorldPts(1,:),axisZWorldPts(2,:),axisZWorldPts(3,:),'b-','LineWidth',1);

    %% Plot contact points
    for iBumper = 1:4
       pointContactWorld = Hist.contacts(iFrame).point.contactWorld(:,iBumper);
       plot3(pointContactWorld(1),pointContactWorld(2),pointContactWorld(3),'mX','MarkerSize',10);
    end

    %% Plot wall
    fill3(wallPts(1,:)',wallPts(2,:)',wallPts(3,:)','r','FaceAlpha',0.4);
    %    plot3(wallLines(1,1:2)',wallLines(2,1:2)',wallLines(3,1:2)','r-');
    %    plot3(wallLines(1,3:4)',wallLines(2,3:4)',wallLines(3,3:4)','r-');

    %% Figure settings
%     axis([axisMin,axisMax,axisMin,axisMax,axisMin,axisMax]);
%     %Crash I-06:
%     xlim([0.65 1.65]);
%     ylim([-0.2 0.8]);
%     zlim([0.35 1.35]);

    %Crash I-10:
    xlim([0.8 1.8]);
    ylim([-0.5 0.5]);
    zlim([0 1]);
    
    xlhand = xlabel('X(m)','Interpreter','LaTex');
    ylhand = ylabel('Y(m)','Interpreter','LaTex');
    zlhand = zlabel('Z(m)','Interpreter','LaTex');
    set(xlhand,'fontsize',14);
    set(ylhand,'fontsize',14);
    set(zlhand,'fontsize',14);
%     title(strcat('t = ',num2str(t(iFrame),'%.2f'),' s'));
    
    setsimulationview(sideview);
    
    grid on;
    axis square;

    drawnow;
%     pause(0.05);

    if recordAnimation == 1
        frame = getframe;
        writeVideo(writerObj,frame);
    end

    hold off;
end

if recordAnimation == 1
    close(writerObj);
end

end

