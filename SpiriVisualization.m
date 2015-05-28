function [ ] =SpiriVisualization( t,X )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
figure('units','normalized','outerposition',[0 0 1 1])
% figure()
% [sx,sy,sz] = sphere;
% sr = 0.11 + 0.2;

p1 = [0.08;0.0115;0];
p2 = [0;0.0575;0];
p3 = [-0.1;0;0];
p4 = [0;-0.0575;0];
p5 = [0.08;-0.0115;0];
c1 = [0.2*cos(pi/4);0.2*cos(pi/4);0];
c2 = [-0.2*cos(pi/4);0.2*cos(pi/4);0];
c3 = [-0.2*cos(pi/4);-0.2*cos(pi/4);0];
c4 = [0.2*cos(pi/4);-0.2*cos(pi/4);0];
po = [0;0;0];
px = [1;0;0];
py = [0;1;0];
pz = [0;0;-1];

writerObj = VideoWriter('badcontrol.avi');
writerObj.FrameRate = 30;
open(writerObj);
axis_min = min([min(X(:,7))-0.4,min(X(:,8))-0.4,-max(X(:,9))-0.4]);
axis_max = max([max(X(:,7))+0.4,max(X(:,8))+0.4,-min(X(:,9))+0.4]);

for i = 1:size(t,1)
%     if i>1
%         break;
%     end
    %figure()
%    R = RotMat('X',X(i,14))*RotMat('Y',X(i,15))*RotMat('Z',X(i,16));
   q = [X(i,10);X(i,11);X(i,12);X(i,13)];
   q = q/norm(q);
   R = quatRotMat(q);
   T = [X(i,7);X(i,8);-X(i,9)];
   p1_p = R'*p1 + T;
   p2_p = R'*p2 + T;
   p3_p = R'*p3 + T;
   p4_p = R'*p4 + T;
   p5_p = R'*p5 + T;
   
   c1_p = R'*c1 + T;
   c2_p = R'*c2 + T;
   c3_p = R'*c3 + T;
   c4_p = R'*c4 + T;
   
   po_p = R'*po + T;
   px_p = R'*px + T;
   py_p = R'*py + T;
   pz_p = R'*pz + T;
   
   pts = [p1_p p2_p p3_p p4_p p5_p p1_p];
   plot3(pts(1,:),pts(2,:),pts(3,:),'Color',[154 215 227]/255,'LineWidth',2);
   hold on;
   plot3(T(1),T(2),T(3),'rx','MarkerSize',8);
   normal = cross(p1_p-p2_p,p2_p-p3_p);
   plotCircle3D(c1_p,normal,0.11);
   plotCircle3D(c2_p,normal,0.11);
   plotCircle3D(c3_p,normal,0.11);
   plotCircle3D(c4_p,normal,0.11);   
%    surf(sx*sr+T(1),sy*sr+T(2),sz*sr+T(3),'FaceColor','y','FaceAlpha',0.2,'EdgeAlpha',0.2);
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
   
   
%    axis([min(X(:,7))-0.4,max(X(:,7))+0.4,min(X(:,8))-0.4,max(X(:,8))+0.4,-max(X(:,9))-0.4,-min(X(:,9))+0.4]);
   axis([axis_min,axis_max,axis_min,axis_max,axis_min,axis_max]);
%  axis([-20,20,-20,20,0,80]);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   title(strcat('t = ',num2str(t(i),'%.2f'),' s'));
   grid on;
   drawnow;
%    frame = getframe;
%    writeVideo(writerObj,frame);
   hold off;
end
close(writerObj);
end

