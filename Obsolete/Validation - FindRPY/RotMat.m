function [ R ] = RotMat( axis, angle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% if ~strcmp(class(angle),'sym')
%     error('Input angle must be symbolic');
% end

if axis == 'x' || axis == 'X'
    R = [1 0 0;0 cos(angle) sin(angle);0 -sin(angle) cos(angle)];
elseif axis == 'y' || axis == 'Y'
    R = [cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
elseif axis == 'z' || axis == 'Z'
    R = [cos(angle) sin(angle) 0;-sin(angle) cos(angle) 0;0 0 1];
else
    error('Input axis must be x, y, or z');
end

end

