function [ outputArray ] = getDataArray( totalArray,startIdx,numData )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

outputArray = [];
for iData = 1:numData
    outputArray = [outputArray,totalArray(startIdx+iData-1)];
end


end

