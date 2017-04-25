function cleanData = cleanVicon(dirtyData)
%this function is just used to flip the vicon data so that it's smooth,
%cause the vicon solver seems to flip between the two possible quaternion
%solutions
cleanData = zeros(size(dirtyData));
cleanData(:,1) = dirtyData(:,1);

for ii = 2:length(dirtyData)
    if sum(-dirtyData(:,ii) <= cleanData(:,ii-1)+ .1)==4 && sum(-dirtyData(:,ii) >= cleanData(:,ii-1)-.1) == 4
        cleanData(:,ii) = -dirtyData(:,ii);
    else
        cleanData(:,ii) = dirtyData(:,ii);
    end
end