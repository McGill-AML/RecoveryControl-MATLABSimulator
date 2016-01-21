% close all;
clear all;
load('data_0head.mat');
data = all_data_recover;


figure('Position',[10,400,2000,1000]);
%% ----------------------------------------------------------------%%
tilt = -15;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,1);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');
%% ----------------------------------------------------------------%%
tilt = -10;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,2);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');
%% ----------------------------------------------------------------%%
tilt = -5;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,3);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');
%% ----------------------------------------------------------------%%
tilt = 0.001;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0,5);

subplot(3,4,4);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');


%% ----------------------------------------------------------------%%
tilt = 5;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.01,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,5);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
tilt = 10;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,6);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
tilt = 15;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,7);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
tilt = 20;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,8);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
tilt = 25;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,9);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
% legend('e = 0.95','e = 0.9','e = 0.85','e = 0.8');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%
tilt = 30;
e = 0.95;
Vc_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e1 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.9;
Vc_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e2 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.85;
Vc_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e3 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

e = 0.8;
Vc_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,10);
defl_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,4);
deflmax_e4 = data(abs(data(:,1).*data(:,2).*(data(:,9)~=0).*(data(:,10)~=0) - tilt*e) <=0.1,5);

subplot(3,4,10);
plot(Vc_e1,defl_e1,'x-',Vc_e2,defl_e2,'x-',Vc_e3,defl_e3,'x-',Vc_e4,defl_e4,'x-');
hold on;
plot(Vc_e1,deflmax_e1,'o--',Vc_e2,deflmax_e2,'o--',Vc_e3,deflmax_e3,'o--',Vc_e4,deflmax_e4,'o--');
title(['tilt = ', blanks(1), num2str(tilt)]);
legend('e = 0.95, first impact','e = 0.90, first impact','e = 0.85, first impact','e = 0.80, first impact','e = 0.95, max defl','e = 0.90, max defl','e = 0.85, max defl','e = 0.80, max defl');
xlabel('Vc (m/s)');
ylabel('Defl (m)');

%% ----------------------------------------------------------------%%

ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,{'Vc vs. Deflection','Recovery Controller On'},'HorizontalAlignment' ,'center','VerticalAlignment', 'top','FontSize',14);
