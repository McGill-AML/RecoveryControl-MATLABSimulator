function plotCircle3D(center,normal,radius,linewidthvalue)
%plotCircle3D.m Plot 3D circle
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Can change linewidthvalue through function input, but
%                line colour must be changed manually below 
%-------------------------------------------------------------------------%

theta=0:0.01:2*pi;
v=null(normal');
points=repmat(center,1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
plot3(points(1,:),points(2,:),points(3,:),'k-','Linewidth',linewidthvalue);

end