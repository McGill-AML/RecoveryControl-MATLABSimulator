Plot = hist2plot(Hist);
crashString = 'Crash 11';



fig = figure();
plot(Hist.times,Plot.bodyAccs(1,:),'b','LineWidth',2);
hold on;
plot(Hist.times,Plot.bodyAccs(2,:),'r--','LineWidth',2);
plot(Hist.times,Plot.bodyAccs(3,:),'m-.','LineWidth',2);
grid on

[ uDerivPk, uDerivLoc ] = findFirstExtrema( Hist.times,Plot.bodyAccs(1,:),timeImpact+0.00 );
[ vDerivPk, vDerivLoc ] = findFirstExtrema( Hist.times,Plot.bodyAccs(2,:),timeImpact+0.00 );
[ wDerivPk, wDerivLoc ] = findFirstExtrema( Hist.times,Plot.bodyAccs(3,:),timeImpact+0.00 );

leg = legend('$\dot{u}$','$\dot{v}$','$\dot{w}$','Location','eastoutside');
set(leg,'Interpreter','latex')
plot(Hist.times(uDerivLoc),uDerivPk,'bo',Hist.times(vDerivLoc),vDerivPk,'ro',Hist.times(wDerivLoc),wDerivPk,'mo');

ylims = get(gca,'ylim');
xlims = get(gca,'xlim');

plot([timeImpact timeImpact],ylims,'k');
xlim([0.4 xlims(2)]);

ylabel('Body Accelerations (m/s^2)');
xlabel('Time (s)');
title(strcat(crashString,' - Body Accelerations'));
export_fig crash11_bodyAccs -transparent
saveas(fig,'crash11_bodyAccs.fig')

fig = figure();
plot(Hist.times,Plot.angVels(1,:),'b','LineWidth',2);
hold on;
plot(Hist.times,Plot.angVels(2,:),'r--','LineWidth',2);
plot(Hist.times,Plot.angVels(3,:),'m-.','LineWidth',2);
grid on

[ pPk, pLoc ] = findFirstExtrema( Hist.times,Plot.angVels(1,:),timeImpact+0.00 );
[ qPk, qLoc ] = findFirstExtrema( Hist.times,Plot.angVels(2,:),timeImpact+0.00 );
[ rPk, rLoc ] = findFirstExtrema( Hist.times,Plot.angVels(3,:),timeImpact+0.00 );

legend('p','q','r','Location','eastoutside');

plot(Hist.times(pLoc),pPk,'bo',Hist.times(qLoc),qPk,'ro',Hist.times(rLoc),rPk,'mo');

ylims = get(gca,'ylim');
xlims = get(gca,'xlim');

plot([timeImpact timeImpact],ylims,'k');
xlim([0.4 xlims(2)]);

ylabel('Body Angular Rates (rad/s)');
xlabel('Time (s)');
title(strcat(crashString,' - Body Angular Rates'));
export_fig crash11_bodyAngVels -transparent
saveas(fig,'crash11_bodyAngVels.fig')

copy2excel = abs([ uDerivPk, vDerivPk, wDerivPk, pPk, qPk, rPk]);






