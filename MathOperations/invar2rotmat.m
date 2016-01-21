function [ rotMat ] = invar2rotmat( rotationAxis, rotationAngle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% if ~strcmp(class(angle),'sym')
%     error('Input angle must be symbolic');
% end

if rotationAxis == 'x' || rotationAxis == 'X'
    rotMat = [1 0 0;0 cos(rotationAngle) sin(rotationAngle);0 -sin(rotationAngle) cos(rotationAngle)];
elseif rotationAxis == 'y' || rotationAxis == 'Y'
    rotMat = [cos(rotationAngle) 0 -sin(rotationAngle);0 1 0;sin(rotationAngle) 0 cos(rotationAngle)];
elseif rotationAxis == 'z' || rotationAxis == 'Z'
    rotMat = [cos(rotationAngle) sin(rotationAngle) 0;-sin(rotationAngle) cos(rotationAngle) 0;0 0 1];
else
    error('Input axis must be x, y, or z');
end

end

