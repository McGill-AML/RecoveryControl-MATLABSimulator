function [ ] =SpiriVisualization1( t,X,sideview,wall_loc,wall_plane, pt1_hist,pt2_hist,Pc_w_hist )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global prop_loc Rb

disprate = 30; %Hz
disprate_idx = round((size(t,1)/(t(end)-t(1)))/disprate);

figure('units','normalized','outerposition',[0 0 1 1])
% figure()
[sx,sy,sz] = sphere;
sx = sx(9:13,:);
sy = sy(9:13,:);
sz = sz(9:13,:)+prop_loc(3,1);
sr = Rb;
sxR = zeros(size(sx));
syR = zeros(size(sy));
szR = zeros(size(sz));

p1 = [0.08;0.0115;0];
p2 = [0;0.0575;0];
p3 = [-0.1;0;0];
p4 = [0;-0.0575;0];
p5 = [0.08;-0.0115;0];
c1 = [0.2*cos(pi/4);0.2*cos(pi/4);prop_loc(3,1)];
c2 = [-0.2*cos(pi/4);0.2*cos(pi/4);prop_loc(3,2)];
c3 = [-0.2*cos(pi/4);-0.2*cos(pi/4);prop_loc(3,3)];
c4 = [0.2*cos(pi/4);-0.2*cos(pi/4);prop_loc(3,4)];
cR = [0;0;prop_loc(3,1)];
po = [0;0;0];
px = [0.5;0;0];
py = [0;0.5;0];
pz = [0;0;0.5];


% writerObj = VideoWriter('simulation.avi');
% writerObj.FrameRate = 30;
% open(writerObj);
axis_min = min([min(X(:,7))-0.4,min(X(:,8))-0.4,min(X(:,9))-0.4]);
axis_max = max([max(X(:,7))+0.4,max(X(:,8))+0.4,max(X(:,9))+0.4]);




[wall_pts, wall_ln] = WallPts(wall_loc,wall_plane,10,15);

for i = 1:disprate_idx:size(t,1)
% for i = 265
    
    
% for i = 91
%     if i>1
%         break;
%     end
    %figure()
%    R = RotMat('X',X(i,14))*RotMat('Y',-X(i,15))*RotMat('Z',X(i,16));
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
   plot3(pts(1,:),pts(2,:),pts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
   hold on;
   plot3(T(1),T(2),T(3),'rx','MarkerSize',8);
%    plot3(Pc(1,i),Pc(2,i),Pc(3,i),'rx','MarkerSize',8);
%    plot3(Pc_wall(1,i),Pc_wall(2,i),Pc_wall(3,i),'bx','MarkerSize',10);
   normal = cross(p1_p-p2_p,p2_p-p3_p);
   plotCircle3D(c1_p,normal,0.11);
   plotCircle3D(c2_p,normal,0.11);
   plotCircle3D(c3_p,normal,0.11);
   plotCircle3D(c4_p,normal,0.11);
   plotCircle3D(cR_p,normal,Rb);
   
   
   for j = 1:size(sx,1)
       for k = 1:size(sx,2)
           sxR(j,k) = R(:,1)'*[sx(j,k);sy(j,k);sz(j,k)];
           syR(j,k) = R(:,2)'*[sx(j,k);sy(j,k);sz(j,k)];
           szR(j,k) = R(:,3)'*[sx(j,k);sy(j,k);sz(j,k)];       
       end
   end  
   
   surf(sxR*sr+T(1),syR*sr+T(2),szR*sr+T(3),'FaceColor','y','FaceAlpha',0.2,'EdgeAlpha',0.5);
%    fill3([4 4 4 4]',[-3 8 8 -3]',[-3 -3 8 8]','r','FaceAlpha',0.4);
   
   xpts = [po_p px_p];
   ypts = [po_p py_p];
   zpts = [po_p pz_p];
   
%    disp('XY check:');
%    disp(round((px_p-po_p)'*(py_p-po_p),2));
%    
%     disp('YZ check:');
%    disp(round((pz_p-po_p)'*(py_p-po_p),2));
%    
%     disp('ZX check:');
%    disp(round((px_p-po_p)'*(pz_p-po_p),2));
   
   plot3(xpts(1,:),xpts(2,:),xpts(3,:),'r-','LineWidth',2);
   plot3(ypts(1,:),ypts(2,:),ypts(3,:),'g-','LineWidth',2);
   plot3(zpts(1,:),zpts(2,:),zpts(3,:),'b-','LineWidth',2);
   
%    plot3(pt1_hist(1,i),pt1_hist(2,i),pt1_hist(3,i),'mX');
%    plot3(pt2_hist(1,i),pt2_hist(2,i),pt2_hist(3,i),'mX');
   plot3(Pc_w_hist(1,i),Pc_w_hist(2,i),Pc_w_hist(3,i),'mX','MarkerSize',10);
%    fill3([w1_x;w2_x;w3_x;w4_x],[w1_y;w2_y;w3_y;w4_y],[w1_z;w2_z;w3_z;w4_z],'r','FaceAlpha',0.4);
   fill3(wall_pts(1,:)',wall_pts(2,:)',wall_pts(3,:)','r','FaceAlpha',0.4);
   plot3(wall_ln(1,1:2)',wall_ln(2,1:2)',wall_ln(3,1:2)','r-');
   plot3(wall_ln(1,3:4)',wall_ln(2,3:4)',wall_ln(3,3:4)','r-');

   %    axis([min(X(:,7))-0.4,max(X(:,7))+0.4,min(X(:,8))-0.4,max(X(:,8))+0.4,min(X(:,9))-0.4,max(X(:,9))+0.4]);
   axis([axis_min,axis_max,axis_min,axis_max,axis_min,axis_max]);
%    axis([3,5,-1,1,3.5,5.5]);
%  axis([-20,20,-20,20,0,80]);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   title(strcat('t = ',num2str(t(i),'%.2f'),' s'));
   if sideview == 'XZ'
    view([0 0]); %view XZ plane
   elseif sideview == 'YZ'
    view(90, 0); %view YZ plane
   end
    grid on;
   axis square;
   drawnow;
%    frame = getframe;
%    writeVideo(writerObj,frame);
   hold off;
end
% close(writerObj);
end

