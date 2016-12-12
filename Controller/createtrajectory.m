function [ ref_posn, ref_head ] = createtrajectory( posn,heading,t,timeStep )
%createtrajectory.m Creates array of trajectory positions and headings
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

ref_x = posn(2,1);
ref_y = posn(2,2);
ref_z = posn(2,3);
ref_head = heading(2);

for i = 2:size(t)

    trajectorySteps = (t(i)-t(i-1))/timeStep;
%     deltax = (posn(i,1) - posn(i-1,1))/trajectorySteps;
%     deltay = (posn(i,2) - posn(i-1,2))/trajectorySteps;
%     deltaz = (posn(i,3) - posn(i-1,3))/trajectorySteps;
%     deltahead = (heading(i) - heading(i-1))/trajectorySteps;
    
    ref_x = [ref_x;posn(i,1)*ones(trajectorySteps,1)];

    ref_y = [ref_y;posn(i,2)*ones(trajectorySteps,1)];

    ref_z = [ref_z;posn(i,3)*ones(trajectorySteps,1)];

    ref_head = [ref_head;heading(i)*ones(trajectorySteps,1)];

    
end

ref_posn = [ref_x ref_y ref_z];

end


