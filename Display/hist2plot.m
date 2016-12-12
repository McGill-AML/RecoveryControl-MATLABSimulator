function [ Plot ] = hist2plot( Hist )
%hist2plot.m Returns plottable arrays 
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: If you append new fields to existing structs in
%   intialization (e.g. initcontact.m), they must be added to the bottom of
%   the file. If you don't, it will mess up the conversions in this
%   function.
%-------------------------------------------------------------------------%

Plot.times = Hist.times;

temp = struct2cell(Hist.propStates);
Plot.propRpms = [temp{1,:}];
Plot.propRpmDerivs = [temp{2,:}];

temp = struct2cell(Hist.poses);
Plot.posns = [temp{1,:}];
Plot.eulerAngles = [temp{3,:}];
Plot.quaternions = [temp{2,:}];
Plot.quaternionDerivs = Hist.stateDerivs(10:13,:);

temp = struct2cell(Hist.twists);
Plot.angVels = [temp{4,:}];
Plot.linVels = [temp{1,:}];
Plot.eulerAngleRates = [temp{5,:}];


Plot.posnDerivs = Hist.stateDerivs(7:9,:);
Plot.bodyAccs = Hist.stateDerivs(1:3,:);
Plot.angAccs = Hist.stateDerivs(4:6,:);

temp = struct2cell(Hist.contacts);
Plot.normalForces = reshape([temp{6,:}],[4,size(temp,2)]);
temp2 = [temp{8,:}];
Plot.slidingDirectionWorlds_bump1 =  temp2(:,1:4:end)';
Plot.slidingDirectionWorlds_bump2 =  temp2(:,2:4:end)';
Plot.slidingDirectionWorlds_bump3 =  temp2(:,3:4:end)';
Plot.slidingDirectionWorlds_bump4 =  temp2(:,4:4:end)';

temp2 = [temp{9,:}];
Plot.tangentialForceWorlds_bump1 = temp2(:,1:4:end)';
Plot.tangentialForceWorlds_bump2 = temp2(:,2:4:end)';
Plot.tangentialForceWorlds_bump3 = temp2(:,3:4:end)';
Plot.tangentialForceWorlds_bump4 = temp2(:,4:4:end)';

temp2 = [temp{7,:}];
Plot.contactPtVelocityWorlds_bump1 = temp2(:,1:4:end)';
Plot.contactPtVelocityWorlds_bump2 = temp2(:,2:4:end)';
Plot.contactPtVelocityWorlds_bump3 = temp2(:,3:4:end)';
Plot.contactPtVelocityWorlds_bump4 = temp2(:,4:4:end)';

temp2 = [temp{3,:}];
temp3 = struct2cell(temp2');
temp4 = [temp3{2,:}];
Plot.contactPointWorlds_bump1 = temp4(:,1:4:end)';
Plot.contactPointWorlds_bump2 = temp4(:,2:4:end)';
Plot.contactPointWorlds_bump3 = temp4(:,3:4:end)';
Plot.contactPointWorlds_bump4 = temp4(:,4:4:end)';
temp4 = [temp3{3,:}];
Plot.contactPointBodys_bump1 = temp4(:,1:4:end)';
Plot.contactPointBodys_bump2 = temp4(:,2:4:end)';
Plot.contactPointBodys_bump3 = temp4(:,3:4:end)';
Plot.contactPointBodys_bump4 = temp4(:,4:4:end)';

Plot.defls = reshape([temp{4,:}],[4,size(temp,2)])';

Plot.muSlidings = [temp{11,:}];

temp = struct2cell(Hist.controls);
Plot.errEulers = [temp{5,:}];
Plot.desEulers = [temp{15,:}];
Plot.desYawDerivs = [temp{14,:}];
Plot.controlAccDes = [temp{3,:}];
Plot.controlUs = [temp{17,:}]; 
Plot.errAngVels = [temp{24,:}];

Plot.recoveryStage = [temp{21,:}];
Plot.accelRef = [temp{22,:}];
Plot.errQuat = [temp{4,:}];

temp2 = [temp{2,:}]; %Control.twist
temp3 = struct2cell(temp2');
Plot.controlAngVels = [temp3{4,:}];


temp = struct2cell(Hist.sensors);
Plot.accelerometers = [temp{1,:}];
Plot.gyros = [temp{2,:}];
Plot.CMaccelerometers = [temp{3,:}];

Plot.worldAcc = zeros(3,numel(Hist.times));
Plot.worldAcc2 = zeros(3,numel(Hist.times));

for iSim = 1:numel(Hist.times)
    rotMat = quat2rotmat(Plot.quaternions(:,iSim));
    bodyAcc = Hist.stateDerivs(1:3,iSim);
    angVel = Hist.states(4:6,iSim);
    linVel = Hist.states(1:3,iSim);
    Plot.worldAcc(:,iSim) = rotMat'*(bodyAcc+cross(angVel,linVel));
    Plot.worldAcc2(:,iSim) = rotMat'*bodyAcc;
end


end

