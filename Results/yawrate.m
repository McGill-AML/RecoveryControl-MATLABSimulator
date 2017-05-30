close all
hold on
grid on
for trial = 1:117
    if(Batch(trial).ImpactInfo.isStable == 1)
        h1 = scatter(5*(Batch(trial).i-1)-30, 0.5+0.25*(Batch(trial).j-1),'b');
    else
        h2 = scatter(5*(Batch(trial).i-1)-30, 0.5+0.25*(Batch(trial).j-1),'r');
    end
    xlabel('Pitch angle ($^\circ$)','Interpreter','LaTex','FontSize',20);
    ylabel('Incoming velocity (m/s)','Interpreter','LaTex','FontSize',20);
    title('Repeat of Fionas Figure 3-3','Interpreter','LaTex','FontSize',20);
    axis([-35 35 0 2.75]);
end
l = legend([h1 h2], 'did not crash in two seconds','did crash in two seconds');
set(l,'Interpreter','LaTex','FontSize',20);