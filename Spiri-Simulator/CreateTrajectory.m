function [ ref_posn, ref_head ] = CreateTrajectory( pos,head,t,dt )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

ref_x = pos(2,1);
ref_y = pos(2,2);
ref_z = pos(2,3);
ref_head = head(2);

for i = 2:size(t)

    steps = (t(i)-t(i-1))/dt;
    deltax = (pos(i,1) - pos(i-1,1))/steps;
    deltay = (pos(i,2) - pos(i-1,2))/steps;
    deltaz = (pos(i,3) - pos(i-1,3))/steps;
    deltahead = (head(i) - head(i-1))/steps;
    
%     ref_x = [ref_x;[pos(i-1,1)+deltax:deltax:pos(i,1)]'];
%     ref_y = [ref_y;[pos(i-1,2)+deltay:deltay:pos(i,2)]'];
%     ref_z = [ref_z;[pos(i-1,3)+deltaz:deltaz:pos(i,3)]'];
%     ref_head = [ref_head;[head(i-1)+deltahead:deltahead:head(i)]'];
%     
%     if deltax == 0
%         ref_x = [ref_x;pos(i,1)*ones(steps,1)];
%     end
%     
%     if deltay == 0
%         ref_y = [ref_y;pos(i,2)*ones(steps,1)];
%     end
%     
%     if deltaz == 0
%         ref_z = [ref_z;pos(i,3)*ones(steps,1)];
%     end
%     
%     if deltahead == 0
%         ref_head = [ref_head;head(i)*ones(steps,1)];
%     end


    ref_x = [ref_x;pos(i,1)*ones(steps,1)];

    ref_y = [ref_y;pos(i,2)*ones(steps,1)];

    ref_z = [ref_z;pos(i,3)*ones(steps,1)];

    ref_head = [ref_head;head(i)*ones(steps,1)];

    
end
ref_posn = [ref_x ref_y ref_z];

