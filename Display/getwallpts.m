function [ wallPts, wallLines ] = getwallpts(wallLoc,wallPlane,wallBottom,wallMiddle,wallHeight,wallWidth)
%getwallpts.m 
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%       Returns points needed to to display wall using fill3, and points to plot
%       lines representing middle of wall
%-------------------------------------------------------------------------%

%Returns points needed to to display wall using fill3, and points to plot
%lines representing middle of wall

wallPts = zeros(3,4);
wallLines = zeros(3,4);

wallPts(3,1) = wallBottom + wallHeight;
wallPts(3,2) = wallBottom + wallHeight;
wallPts(3,3) = wallBottom;
wallPts(3,4) = wallBottom;

wallLines(3,1) = wallBottom + wallHeight;
wallLines(3,2) = wallBottom;
wallLines(3,3) = wallBottom + wallHeight/2;
wallLines(3,4) = wallBottom + wallHeight/2;
    
if sum(wallPlane == 'XZ')==2 || sum(wallPlane == 'ZX')== 2
    wallPts(2,1) = wallLoc;
    wallPts(2,2) = wallLoc;
    wallPts(2,3) = wallLoc;
    wallPts(2,4) = wallLoc;
    
    wallPts(1,1) = wallMiddle - wallWidth/2;
    wallPts(1,2) = wallMiddle + wallWidth/2;
    wallPts(1,3) = wallMiddle + wallWidth/2;
    wallPts(1,4) = wallMiddle - wallWidth/2;
    
    wallLines(1,1) = wallMiddle;
    wallLines(1,2) = wallMiddle;
    wallLines(1,3) = wallMiddle - wallWidth/2;
    wallLines(1,4) = wallMiddle + wallWidth/2;
    
    wallLines(2,1) = wallLoc;
    wallLines(2,2) = wallLoc;
    wallLines(2,3) = wallLoc;
    wallLines(2,4) = wallLoc;
    
elseif (sum(wallPlane == 'YZ')==2 || sum(wallPlane == 'ZY')==2)
    wallPts(1,1) = wallLoc;
    wallPts(1,2) = wallLoc;
    wallPts(1,3) = wallLoc;
    wallPts(1,4) = wallLoc;
    
    wallPts(2,1) = wallMiddle - wallWidth/2;
    wallPts(2,2) = wallMiddle + wallWidth/2;
    wallPts(2,3) = wallMiddle + wallWidth/2;
    wallPts(2,4) = wallMiddle - wallWidth/2;
    
    wallLines(1,1) = wallLoc;
    wallLines(1,2) = wallLoc;
    wallLines(1,3) = wallLoc;
    wallLines(1,4) = wallLoc;
    
    wallLines(2,1) = wallMiddle;
    wallLines(2,2) = wallMiddle;
    wallLines(2,3) = wallMiddle - wallWidth/2;
    wallLines(2,4) = wallMiddle + wallWidth/2;
    
else
    error('Invalid vertical wall_plane');
end

end

