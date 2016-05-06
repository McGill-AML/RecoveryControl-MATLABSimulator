Plot = hist2plot(Hist);
temp = struct2cell(Hist.contacts);
temp2 = [temp{2,:}];

contactPtPosnWorld_bump1 = zeros(numel(Plot.times),3);
contactPtPosnWorld_bump2 = zeros(numel(Plot.times),3);
contactPtPosnWorld_bump3 = zeros(numel(Plot.times),3);
contactPtPosnWorld_bump4 = zeros(numel(Plot.times),3);

for i = 1:numel(Plot.times)
    
    if sum(sum(temp2(i).contactWorld(:,:))) < 800
        contactPtPosnWorld_bump1(i,:) = temp2(i).contactWorld(:,1)';
        contactPtPosnWorld_bump1(i,:) = temp2(i).contactWorld(:,2)';
        contactPtPosnWorld_bump1(i,:) = temp2(i).contactWorld(:,2)';
        contactPtPosnWorld_bump1(i,:) = temp2(i).contactWorld(:,4)';   
    end
end