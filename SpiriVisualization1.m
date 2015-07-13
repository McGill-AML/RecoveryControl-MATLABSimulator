function [ ] =SpiriVisualization1( record,t,X,sideview,wall_loc,wall_plane, pint1_hist,pint2_hist,pc_w_hist )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global prop_loc Rbumper Cbumper

disprate = 30; %Hz
disprate_idx = round((size(t,1)/(t(end)-t(1)))/disprate);

figure('units','normalized','outerposition',[0 0 1 1])

%% Create body-fixed points of spherical bumper
[sx,sy,sz] = sphere;
sx = sx(9:13,:);
sy = sy(9:13,:);
sz = sz(9:13,:)+prop_loc(3,1);
sr = Rbumper;
sxR = zeros(size(sx));
syR = zeros(size(sy));
szR = zeros(size(sz));

%% Create body-fixed points of Spiri body
p1 = [0.08;0.0115;0];
p2 = [0;0.0575;0];
p3 = [-0.1;0;0];
p4 = [0;-0.0575;0];
p5 = [0.08;-0.0115;0];

%% Create body-fixed centers of 4 bumpers + virtual bumper
c1 = [0.2*cos(pi/4);0.2*cos(pi/4);prop_loc(3,1)];
c2 = [-0.2*cos(pi/4);0.2*cos(pi/4);prop_loc(3,2)];
c3 = [-0.2*cos(pi/4);-0.2*cos(pi/4);prop_loc(3,3)];
c4 = [0.2*cos(pi/4);-0.2*cos(pi/4);prop_loc(3,4)];
cR = Cbumper;

%% Create body-fixed axes
po = [0;0;0];
px = [0.3;0;0];
py = [0;0.3;0];
pz = [0;0;0.3];

%%  Calculate axes ranges for plotting
axis_min = min([min(X(:,7))-0.4,min(X(:,8))-0.4,min(X(:,9))-0.4]);
axis_max = max([max(X(:,7))+0.4,max(X(:,8))+0.4,max(X(:,9))+0.4]);

%% Create world-frame wall points
[wall_pts, wall_ln] = WallPts(wall_loc,wall_plane,10,15);

if record == 1
    writerObj = VideoWriter('simulation_nosat.avi');
    writerObj.FrameRate = disprate;
    open(writerObj);
end

for i = 1:disprate_idx:size(t,1)
% for i = 313    
   
   %% Rotate body-fixed points to world-frame points
   q = [X(i,10);X(i,11);X(i,12);X(i,13)];
   q = q/norm(q);
   R = quatRotMat(q);
   T = [X(i,7);X(i,8);X(i,9)];
   
   p1_p = R'*p1 + T;
   p2_p = R'*p2 + T;
   p3_p = R'*p3 + T;
   p4_p = R'*p4 + T;
   p5_p = R'*p5 + T;
   
   c1_p = R'*c1 + T;
   c2_p = R'*c2 + T;
   c3_p = R'*c3 + T;
   c4_p = R'*c4 + T;
   cR_p = R'*cR + T;
   
   po_p = R'*po + T;
   px_p = R'*px + T;
   py_p = R'*py + T;
   pz_p = R'*pz + T;
   
   pts = [p1_p p2_p p3_p p4_p p5_p p1_p];
   
   %% Plot Spiri body points
   plot3(pts(1,:),pts(2,:),pts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
   hold on;
   plot3(T(1),T(2),T(3),'rx','MarkerSize',8); %Centre of mass
   
   %% Plot Spiri 2-d bumpers 
   normal = cross(p1_p-p2_p,p2_p-p3_p);
   plotCircle3D(c1_p,normal,0.11);
   plotCircle3D(c2_p,normal,0.11);
   plotCircle3D(c3_p,normal,0.11);
   plotCircle3D(c4_p,normal,0.11);
   plotCircle3D(cR_p,normal,Rbumper); %Virtual bumper
   
   %% Plot Spiri spherical bumper
   for j = 1:size(sx,1)
       for k = 1:size(sx,2)
           sxR(j,k) = R(:,1)'*[sx(j,k);sy(j,k);sz(j,k)];
           syR(j,k) = R(:,2)'*[sx(j,k);sy(j,k);sz(j,k)];
           szR(j,k) = R(:,3)'*[sx(j,k);sy(j,k);sz(j,k)];       
       end
   end     
%    surf(sxR*sr+T(1),syR*sr+T(2),szR*sr+T(3),'FaceColor','y','FaceAlpha',0.2,'EdgeAlpha',0.5);

    %% Plot body-fixed axes
    xpts = [po_p px_p];
    ypts = [po_p py_p];
    zpts = [po_p pz_p];   

    plot3(xpts(1,:),xpts(2,:),xpts(3,:),'r-','LineWidth',2);
    plot3(ypts(1,:),ypts(2,:),ypts(3,:),'g-','LineWidth',2);
    plot3(zpts(1,:),zpts(2,:),zpts(3,:),'b-','LineWidth',2);

    %% Plot contact points
%    plot3(pt1_hist(1,i),pt1_hist(2,i),pt1_hist(3,i),'mX');
%    plot3(pt2_hist(1,i),pt2_hist(2,i),pt2_hist(3,i),'mX');
   plot3(pc_w_hist(1,i),pc_w_hist(2,i),pc_w_hist(3,i),'mX','MarkerSize',10);

   %% Plot wall
   fill3(wall_pts(1,:)',wall_pts(2,:)',wall_pts(3,:)','r','FaceAlpha',0.4);
   plot3(wall_ln(1,1:2)',wall_ln(2,1:2)',wall_ln(3,1:2)','r-');
   plot3(wall_ln(1,3:4)',wall_ln(2,3:4)',wall_ln(3,3:4)','r-');

   %% Figure settings
   axis([axis_min,axis_max,axis_min,axis_max,axis_min,axis_max]);
%    axis([3,5,-1,1,3.5,5.5]);
%  axis([-20,20,-20,20,0,80]);
   xlabel('x^W');
   ylabel('y^W');
   zlabel('z^W');
   title(strcat('t = ',num2str(t(i),'%.2f'),' s'));
   
   if sideview == 'XZ'
        view([0 0]); %view XZ plane
   elseif sideview == 'YZ'
        view(90, 0); %view YZ plane
   end
   
   grid on;
   axis square;
   drawnow;
   
   if record == 1
    frame = getframe;
    writeVideo(writerObj,frame);
   end
   
   hold off;
end

if record == 1
    close(writerObj);
end
end

