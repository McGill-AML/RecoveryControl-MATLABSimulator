% close all;
clear all;
load('data_45head.mat');
data = all_data_recover;



figure('Position',[10,400,2000,1000]);

%% ----------------------------------------------------------------%%
Vc = 0.1;
e = 0.95;
tilt_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.9;
tilt_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.85;
tilt_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.8;
tilt_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

subplot(3,2,1);
plot(tilt_e1,defl_e1,'x-',tilt_e2,defl_e2,'x-',tilt_e3,defl_e3,'x-',tilt_e4,defl_e4,'x-');
hold on;
plot(tilt_e1,deflmax_e1,'o--',tilt_e2,deflmax_e2,'o--',tilt_e3,deflmax_e3,'o--',tilt_e4,deflmax_e4,'o--');
title(['Vc =',blanks(1),num2str(Vc)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Tilt (deg)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
Vc = 0.3;
e = 0.95;
tilt_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.9;
tilt_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.85;
tilt_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.8;
tilt_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

subplot(3,2,2);
plot(tilt_e1,defl_e1,'x-',tilt_e2,defl_e2,'x-',tilt_e3,defl_e3,'x-',tilt_e4,defl_e4,'x-');
hold on;
plot(tilt_e1,deflmax_e1,'o--',tilt_e2,deflmax_e2,'o--',tilt_e3,deflmax_e3,'o--',tilt_e4,deflmax_e4,'o--');
title(['Vc =',blanks(1),num2str(Vc)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Tilt (deg)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
Vc = 0.5;
e = 0.95;
tilt_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.9;
tilt_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.85;
tilt_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.8;
tilt_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

subplot(3,2,3);
plot(tilt_e1,defl_e1,'x-',tilt_e2,defl_e2,'x-',tilt_e3,defl_e3,'x-',tilt_e4,defl_e4,'x-');
hold on;
plot(tilt_e1,deflmax_e1,'o--',tilt_e2,deflmax_e2,'o--',tilt_e3,deflmax_e3,'o--',tilt_e4,deflmax_e4,'o--');
title(['Vc =',blanks(1),num2str(Vc)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Tilt (deg)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
Vc = 0.7;
e = 0.95;
tilt_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.9;
tilt_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.85;
tilt_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.8;
tilt_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

subplot(3,2,4);
plot(tilt_e1,defl_e1,'x-',tilt_e2,defl_e2,'x-',tilt_e3,defl_e3,'x-',tilt_e4,defl_e4,'x-');
hold on;
plot(tilt_e1,deflmax_e1,'o--',tilt_e2,deflmax_e2,'o--',tilt_e3,deflmax_e3,'o--',tilt_e4,deflmax_e4,'o--');
title(['Vc =',blanks(1),num2str(Vc)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Tilt (deg)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
Vc = 0.9;
e = 0.95;
tilt_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0).*(data(:,9)~=0) == Vc*e,2);
defl_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0).*(data(:,9)~=0) == Vc*e,4);
deflmax_e1 = data(data(:,1).*data(:,3).*(data(:,9)~=0).*(data(:,9)~=0) == Vc*e,5);

e = 0.9;
tilt_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e2 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.85;
tilt_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e3 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

e = 0.8;
tilt_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,2);
defl_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,4);
deflmax_e4 = data(data(:,1).*data(:,3).*(data(:,9)~=0) == Vc*e,5);

subplot(3,2,5);
plot(tilt_e1,defl_e1,'x-',tilt_e2,defl_e2,'x-',tilt_e3,defl_e3,'x-',tilt_e4,defl_e4,'x-');
hold on;
plot(tilt_e1,deflmax_e1,'o--',tilt_e2,deflmax_e2,'o--',tilt_e3,deflmax_e3,'o--',tilt_e4,deflmax_e4,'o--');
title(['Vc =',blanks(1),num2str(Vc)]);
legend('e = 0.95, first impact','e = 0.90, first impact','e = 0.85, first impact','e = 0.80, first impact','e = 0.95, max defl','e = 0.90, max defl','e = 0.85, max defl','e = 0.80, max defl');
xlabel('Tilt (deg)');
ylabel('Defl (m)');
%% ----------------------------------------------------------------%%

ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,{'Tilt vs. Deflection','Recovery Controller On'},'HorizontalAlignment' ,'center','VerticalAlignment', 'top','FontSize',14);