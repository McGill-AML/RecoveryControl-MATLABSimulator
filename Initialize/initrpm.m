function [rpm,u] = initrpm(rotMat, worldLinAcc)

global m g u2RpmMat

u1 = m*(rotMat(3,3)*g + rotMat(3,:)*worldLinAcc);
% u1 = m*(rotMat(3,3)*g);

u = [u1;0;0;0];
disp(u1)

%Propeller RPM Control Signal
temp = u2RpmMat*u;
rpmsquare = temp.*(temp>0);
rpm = sqrt(rpmsquare);
rpm = [-rpm(1);rpm(2);-rpm(3);rpm(4)]; %in RPM
    
end