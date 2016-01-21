function [ Plot ] = hist2plot( Hist )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

temp = struct2cell(Hist.propStates);
Plot.propRpms = [temp{1,:}];

temp = struct2cell(Hist.poses);
Plot.posns = [temp{1,:}];

temp = struct2cell(Hist.twists);
Plot.angVels = [temp{3,:}];
Plot.linVels = [temp{1,:}];

Plot.posnDerivs = Hist.stateDerivs(7:9,:);
Plot.bodyAccs = Hist.stateDerivs(1:3,:);
end

