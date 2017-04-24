
close all
for trial = 1:1000
    if trial ~= 979
        if Batch(trial).contact 
        scatter(Batch(trial).offset,max(abs(Batch(trial).Plot.eulerAngleRates(3,:))));
    end
    grid on
    hold on
end