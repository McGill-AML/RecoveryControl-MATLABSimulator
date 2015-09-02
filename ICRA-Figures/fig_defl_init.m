load('fig_defl_data.mat');
data = fig_defl_data;

figure('Position',[390 260 920 480]);
subplot(1,2,1);
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
title('\psi = \pi/4');
% %------------------------------------------------------------------------%


subplot(1,2,2);
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
title('\psi = 0');

legend1 = sprintf('\\bfv\\rm_{CX} = 0.50 \\pm 5%% (m/s)');
legend2 = sprintf('\\bfv\\rm_{CX} = 0.75 \\pm 5%% (m/s)');
legend3 = sprintf('\\bfv\\rm_{CX} = 1.00 \\pm 5%% (m/s)');
legend(legend1,legend2,legend3);

label_tilt =sprintf('Tilt (%c)', char(176));
xlabel(label_tilt);
ylabel('\delta_{init} (m)');


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
