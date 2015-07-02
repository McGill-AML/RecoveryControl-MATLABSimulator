function [ wall_pts, wall_ln ] = WallPts(wall_loc,wall_plane,height,width)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    w1_z = height;
    w2_z = height;
    w3_z = -1;
    w4_z = -1;
    
    pt1_z = height;
    pt2_z = -1;
    pt3_z = height/2;
    pt4_z = height/2;
    
if sum(wall_plane == 'XZ')==2 || sum(wall_plane == 'ZX')== 2
    w1_y = wall_loc;
    w2_y = wall_loc;
    w3_y = wall_loc;
    w4_y = wall_loc;
    
    w1_x = -width/2;
    w2_x = width/2;
    w3_x = width/2;
    w4_x = -width/2;
    
    pt1_x = 0;
    pt2_x = 0;
    pt3_x = -width/2;
    pt4_x = width/2;
    
    pt1_y = wall_loc;
    pt2_y = wall_loc;
    pt3_y = wall_loc;
    pt4_y = wall_loc;
    
elseif (sum(wall_plane == 'YZ')==2 || sum(wall_plane == 'ZY')==2)
    w1_x = wall_loc;
    w2_x = wall_loc;
    w3_x = wall_loc;
    w4_x = wall_loc;
    
    w1_y = -width/2;
    w2_y = width/2;
    w3_y = width/2;
    w4_y = -width/2;
    
    pt1_x = wall_loc;
    pt2_x = wall_loc;
    pt3_x = wall_loc;
    pt4_x = wall_loc;
    
    pt1_y = 0;
    pt2_y = 0;
    pt3_y = -width/2;
    pt4_y = width/2;
    
else
    error('Invalid vertical wall_plane');
end

wall_pts = [w1_x w2_x w3_x w4_x;w1_y w2_y w3_y w4_y;w1_z w2_z w3_z w4_z];
wall_ln = [pt1_x pt2_x pt3_x pt4_x;pt1_y pt2_y pt3_y pt4_y;pt1_z pt2_z pt3_z pt4_z];

end

