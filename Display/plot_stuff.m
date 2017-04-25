close all
Plot.stateDeriv = Hist.stateDerivs(1:13,:)';
for i = 1:101
    Plot.normStateDeriv(i,:) = norm(Plot.stateDeriv(i,:));
end
plot(Plot.times, Hist.stateDerivs(4:6,:))
hold on

% plot(Plot.times, Plot.slidingVelocityWorlds_bump1);
% % plot(Plot.times, Plot.contactPtVelocityWorlds_bump1);
% plot(Plot.times, Plot.slidingDirectionWorlds_bump1);
% plot(Plot.times, Plot.normalForces)
% plot(Plot.times,100*tDelay)
% legend('stateDeriv','delay')
grid on
