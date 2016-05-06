col=hsv(5);
crashName = 'Grazing Maneuv.';

%% Plot Contact Forces
figure()
for iBumper = 1:4
    subplot(3,4,iBumper); %normal force
    hold on;
    for iCoeff = 0:4
%         timeVar = strcat('Plot_mu',num2str(iCoeff),'.times');
%         forceVar = strcat('Plot_mu',num2str(iCoeff),'.normalForces(',num2str(iBumper),',:)');
        eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
        eval(sprintf('forceVar = Plot_mu%d.normalForces(%d,:);',iCoeff,iBumper));                 
        plot(timeVar,forceVar,'color',col(iCoeff+1,:));              
    end
    grid on;
    title(strcat('Bumper ',num2str(iBumper)));
    ylabel('F_n (N)');
    xlabel('Time (s)')';
%     legend('\mu = 0.0','\mu = 0.1','\mu = 0.2','\mu = 0.3','\mu = 0.4');
    ylims = ylim();
    plot([0.5 0.5],ylims,'k-');
    
    
    subplot(3,4,iBumper+4); %tangential force
    hold on;
    for iCoeff = 0:4
        eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
        eval(sprintf('forceVar_y = Plot_mu%d.tangentialForceWorlds_bump%d(:,2);',iCoeff,iBumper));                 
        plot(timeVar,forceVar_y,'color',col(iCoeff+1,:),'LineStyle','-');
    end
    grid on;
    ylabel('F_{t,y} (N)');
    xlabel('Time (s)')';
%     legend('\mu = 0.0','\mu = 0.1','\mu = 0.2','\mu = 0.3','\mu = 0.4');
    ylims = ylim();
    plot([0.5 0.5],ylims,'k-');
    
    subplot(3,4,iBumper+8); %tangential force
    hold on;
    for iCoeff = 0:4
        eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
        eval(sprintf('forceVar_z = Plot_mu%d.tangentialForceWorlds_bump%d(:,3);',iCoeff,iBumper));  
        plot(timeVar,forceVar_z,'color',col(iCoeff+1,:),'LineStyle','-');
    end
    grid on;
    ylabel('F_{t,z} (N)');
    xlabel('Time (s)')';
%     legend('\mu = 0.0','\mu = 0.1','\mu = 0.2','\mu = 0.3','\mu = 0.4');
    ylims = ylim();
    plot([0.5 0.5],ylims,'k-');
end
subplot(3,4,8);
[legend_h,~,~,~] = legend('\mu = 0.0','\mu = 0.1','\mu = 0.2','\mu = 0.3','\mu = 0.4');
set(legend_h,'Position',[0.9227 0.4670 0.0477 0.1028]);
suptitle(strcat('Contact Forces - ',crashName));

%% Plot Quadrotor States
figure()

% ----------- World Positions ---------%

subplot(3,4,1); %XPosn
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('posnVar_x = Plot_mu%d.posns(1,:);',iCoeff));  
    plot(timeVar,posnVar_x,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('X Position (m)');
xlabel('Time (s)');
title('Positions');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,5); %YPosn
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('posnVar_y = Plot_mu%d.posns(2,:);',iCoeff));  
    plot(timeVar,posnVar_y,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('Y Position (m)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,9); %ZPosn
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('posnVar_z = Plot_mu%d.posns(3,:);',iCoeff));  
    plot(timeVar,posnVar_z,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('Z Position (m)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

% ----------- Body Linear Velocities ---------%

subplot(3,4,2); %u
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.linVels(1,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('u (m/s)');
xlabel('Time (s)');
title('Body Linear Velocities');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,6); %v
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.linVels(2,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('v (m/s)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,10); %w
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.linVels(3,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('w (m/s)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

% ----------- Attitude ---------%

subplot(3,4,3); %roll
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.eulerAngles(1,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('\phi (rad)');
xlabel('Time (s)');
title('Euler Angles');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,7); %pitch
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.eulerAngles(2,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('\theta (rad)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,11); %yaw
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.eulerAngles(3,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('\psi (rad)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

% ----------- Body Angular Rates ---------%

subplot(3,4,4); %p
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.angVels(1,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('p (rad/s)');
xlabel('Time (s)');
title('Angular Velocities');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,8); %q
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.angVels(2,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('q (rad/s)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,12); %r
hold on;
for iCoeff = 0:4
    eval(sprintf('timeVar = Plot_mu%d.times;',iCoeff));
    eval(sprintf('plotVar = Plot_mu%d.angVels(3,:);',iCoeff));  
    plot(timeVar,plotVar,'color',col(iCoeff+1,:),'LineStyle','-');
end
grid on;
ylabel('r (rad/s)');
xlabel('Time (s)');
ylims = ylim();
plot([0.5 0.5],ylims,'k-');

subplot(3,4,8);
[legend_h,~,~,~] = legend('\mu = 0.0','\mu = 0.1','\mu = 0.2','\mu = 0.3','\mu = 0.4');
set(legend_h,'Position',[0.9227 0.4670 0.0477 0.1028]);
suptitle(strcat('Quadrotor States - ',crashName));


