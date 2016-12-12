function [ outputArray ] = getDataArray( totalArray,startIdx,numData )

outputArray = [];
for iData = 1:numData
    outputArray = [outputArray,totalArray(startIdx+iData-1)];
end


end

