
close all
for trial = 1:100
        plot(Batch(trial).Plot.times, Batch(trial).Plot.eulerAngleRates(3,:));
    grid on
    hold on
%     plot(Batch(trial).Plot.times, Batch(trial).Plot.recoveryStage);

end