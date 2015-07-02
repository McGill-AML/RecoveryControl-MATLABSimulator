function [ pB_contact,pW_wall,vi_contact,ti_contact,numContacts,flag_c ] = DetectContact( i,X, r_ribbon, numContacts )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global prop_loc

dist_contact = 0.5*1e-3;

q = [X(10);X(11);X(12);X(13)];
q = q/norm(q);
R = quatRotMat(q);
T = [X(7);X(8);-X(9)];

ang = 0:0.01:2*pi; 
xB_ribbon=r_ribbon*cos(ang);
yB_ribbon=r_ribbon*sin(ang);
zB_ribbon= prop_loc(3,1)*ones(size(ang));

pW_ribbon = R'*[xB_ribbon;yB_ribbon;zB_ribbon] + repmat(T,size(ang));
pW_dist = (repmat(4,size(ang))-pW_ribbon(1,:));
[min_dist, min_idx] = min(pW_dist);
% disp('Minimum Dist');
% disp(min_dist);

if min_dist > dist_contact
    flag_c = 0;
%     disp('False Contact Detect');
    pB_contact = [];
    pW_wall = [];
    vi_contact = [];
    ti_contact = [];
else
    flag_c = 1;
    pB_contact = [xB_ribbon(min_idx);yB_ribbon(min_idx);zB_ribbon(min_idx)];
    pW_wall = [4;pW_ribbon(2,min_idx);pW_ribbon(3,min_idx)];
    vi_contact = sqrt(sum(X(1:3).^2));
    ti_contact = i;

    numContacts = numContacts + 1;
end

% vB_normal = R*[-1;0;0];            
% %             vB_normal = CM - pB_contact;
% vB_normal = vB_normal/norm(vB_normal);

end

