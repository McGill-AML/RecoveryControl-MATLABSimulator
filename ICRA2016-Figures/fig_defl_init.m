load('fig_defl_data.mat');
data = fig_defl_data;

% figure('Position',[390 260 920 480]);
% subplot(1,2,1);
figure();
head = pi/4;

Vc = 0.5;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'r-o','LineWidth',2);
hold on;

Vc = 0.75;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'b--o','LineWidth',2);

Vc = 1.0;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'m-.o','LineWidth',2);

xlim([-21 21]);
ylim([0 0.1]);
% title('\psi = \pi/4');

str = {'$$ \dot{X}_C = 0.50 \pm 5 \% $$', '$$ \dot{X}_C = 0.75 \pm 5 \% $$', '$$ \dot{X}_C = 1.0 \pm 5 \% $$'};
legend(str, 'Interpreter','latex', 'Location','NW')

label_tilt =sprintf('Inclination (%c)', char(176));
xlabel(label_tilt);
ylabel('\delta_{init} (m)');
set(gca,'FontSize',12);
% %------------------------------------------------------------------------%

figure()
% subplot(1,2,2);
head = 0;

Vc = 0.5;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'r-o','LineWidth',2);
hold on;

Vc = 0.75;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'b--o','LineWidth',2);

Vc = 1.0;
plot(data(data(:,1) + data(:,3) == head + Vc,2),data(data(:,1) + data(:,3) == head + Vc,5),'m-.o','LineWidth',2);

xlim([-21 21]);
ylim([0 0.1]);
% title('\psi = 0');

str = {'$$ \dot{X}_C = 0.50 \pm 5 \% $$', '$$ \dot{X}_C = 0.75 \pm 5 \% $$', '$$ \dot{X}_C = 1.0 \pm 5 \% $$'};
legend(str, 'Interpreter','latex', 'Location','NW')

label_tilt =sprintf('Inclination (%c)', char(176));
xlabel(label_tilt);
ylabel('\delta_{init} (m)');
set(gca,'FontSize',12);


% figure('Position',[680 120 560 850])
% 
% subplot(3,1,1)
% plot_title = sprintf('\\bfv\\rm_{CX} = 0.5 m/s');
% title(plot_title);
% 
% subplot(3,1,2)
% plot_title = sprintf('\\bfv\\rm_{CX} = 0.75 m/s');
% title(plot_title);
% 
% subplot(3,1,3)
% plot_title = sprintf('\\bfv\\rm_{CX} = 1.0 m/s');
% title(plot_title);
% 
% 
% 
% suptitle('\psi = \pi/4');
% 
% %------------------------------------------------------------------------%
% figure('Position',[680+560 120 560 850])
% 
% subplot(3,1,1)
% plot_title = sprintf('\\bfv\\rm_{CX} = 0.5 m/s');
% title(plot_title);
% 
% subplot(3,1,2)
% plot_title = sprintf('\\bfv\\rm_{CX} = 0.75 m/s');
% title(plot_title);
% 
% subplot(3,1,3)
% plot_title = sprintf('\\bfv\\rm_{CX} = 1.0 m/s');
% title(plot_title);
% 
% 
% 
% suptitle('\psi = 0');
