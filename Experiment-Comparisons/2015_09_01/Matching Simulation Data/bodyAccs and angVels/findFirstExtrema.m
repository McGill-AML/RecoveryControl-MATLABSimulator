function [ pk, loc ] = findFirstExtrema( t,y,timeAfter )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

[ymax, imax, ymin, imin] = extrema(y);
A = [imax imin; ymax ymin];
[~,sortedIdx] = sort(A(1,:));
sortedA = A(:,sortedIdx);
afterLocs = find(sortedA(1,:)>vlookup(t,timeAfter));

if numel(afterLocs)>0
    loc = sortedA(1,afterLocs(1));
    pk = sortedA(2,afterLocs(1));
else
    loc = 1;
    pk = 0;
end


end

