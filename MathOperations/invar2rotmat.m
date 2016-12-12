function [ rotMat ] = invar2rotmat( rotationAxis, rotationAngle )
%invar2rotmat.m Converts rotation representation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Converts rotation representation from rotation axis +
%   rotation angle around that axis to a rotation matrix
%-------------------------------------------------------------------------%
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
