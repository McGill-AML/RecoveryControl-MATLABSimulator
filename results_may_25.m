load('iteration_no_recovery.mat');
%%
load('iteration_with_recovery.mat');

%%
close all
hold on
grid on
numPitch = 46;
numOffset = 71;
for iPitch = 1:numPitch
    for iOffset = 1:numOffset
        trial = 71*(iPitch-1)+iOffset;
        disp(trial);
        if (length(Batch(trial).Plot.times) < 401)
            h1 = scatter(iPitch-1,-1+2*((iOffset-1)/numOffset),'MarkerFaceColor','r','MarkerEdgeColor','r');
        else
            h2 = scatter(iPitch-1,-1+2*((iOffset-1)/numOffset),'MarkerFaceColor','b','MarkerEdgeColor','b');
        end
    end
end
legend([h2 h1],'Successful Recovery','Failed recovery');
xlabel('Pitch (degrees)');
ylabel('Offset (normalized)');