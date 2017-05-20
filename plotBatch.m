
close all
for trial = 1:100
    numBumpersImpacted = 0;
    for bumper = 1:4   % check how many bumpers impacted
        if(sum(Batch(trial).Plot.defls(:,bumper))~=0)
            numBumpersImpacted = numBumpersImpacted + 1;
        end
    end
    if numBumpersImpacted == 0
        scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'g');
    elseif numBumpersImpacted == 1
        scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'b');
    elseif numBumpersImpacted >= 2
        scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))),'r');
    end
%     legend('zero collision','single collision','double collision');
    grid on
    hold on
end