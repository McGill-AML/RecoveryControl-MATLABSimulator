% 
close all
h = zeros(2,1);
for trial = 1:1000
    numBumpersImpacted = 0;
    for bumper = 1:4   % check how many bumpers impacted
        if(sum(Batch(trial).Plot.defls(:,bumper))~=0)
            numBumpersImpacted = numBumpersImpacted + 1;
        end
    end
    if numBumpersImpacted == 0
        scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'g');
    elseif numBumpersImpacted == 1
        h1 = scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'b');
    elseif numBumpersImpacted >= 2
        h2 = scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'r');
    end
    axis([-1 1 0 14])
    grid on
    hold on
end
legend([h1 h2], 'single impact','multiple impacts');
