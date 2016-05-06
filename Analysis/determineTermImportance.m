copy2Excel = zeros(12,3);
%% Posns
figure()
% X Posns
subplot(3,1,1);
plot(Hist_termsIncluded.times,Plot_termsIncluded.posns(1,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.posns(1,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.posns(1,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.posns(1,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('X Posn');

sizeTermsIncluded = size(Hist_termsIncluded.times,1);
maxError_noGyroMoment = max(abs(Plot_termsIncluded.posns(1,:)-Plot_noGyroMoment.posns(1,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.posns(1,:)-Plot_noRpmDeriv.posns(1,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.posns(1,:)-Plot_noTerms.posns(1,1:sizeTermsIncluded)));
display('X Posn');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(1,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% Y Posns
subplot(3,1,2);
plot(Hist_termsIncluded.times,Plot_termsIncluded.posns(2,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.posns(2,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.posns(2,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.posns(2,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('Y Posn');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.posns(2,:)-Plot_noGyroMoment.posns(2,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.posns(2,:)-Plot_noRpmDeriv.posns(2,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.posns(2,:)-Plot_noTerms.posns(2,1:sizeTermsIncluded)));
display('Y Posn');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(2,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% Z Posns
subplot(3,1,3);
plot(Hist_termsIncluded.times,Plot_termsIncluded.posns(3,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.posns(3,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.posns(3,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.posns(3,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('Z Posn');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.posns(3,:)-Plot_noGyroMoment.posns(3,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.posns(3,:)-Plot_noRpmDeriv.posns(3,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.posns(3,:)-Plot_noTerms.posns(3,1:sizeTermsIncluded)));
display('Z Posn');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(3,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

suptitle('Crash 6 - Type B');

%% Euler Angles
figure()
% Roll
subplot(3,1,1);
plot(Hist_termsIncluded.times,Plot_termsIncluded.eulerAngles(1,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.eulerAngles(1,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.eulerAngles(1,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.eulerAngles(1,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('Roll');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.eulerAngles(1,:)-Plot_noGyroMoment.eulerAngles(1,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.eulerAngles(1,:)-Plot_noRpmDeriv.eulerAngles(1,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.eulerAngles(1,:)-Plot_noTerms.eulerAngles(1,1:sizeTermsIncluded)));
display('Roll');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(4,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% Pitch
subplot(3,1,2);
plot(Hist_termsIncluded.times,Plot_termsIncluded.eulerAngles(2,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.eulerAngles(2,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.eulerAngles(2,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.eulerAngles(2,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('Pitch');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.eulerAngles(2,:)-Plot_noGyroMoment.eulerAngles(2,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.eulerAngles(2,:)-Plot_noRpmDeriv.eulerAngles(2,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.eulerAngles(2,:)-Plot_noTerms.eulerAngles(2,1:sizeTermsIncluded)));
display('Pitch');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(5,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% Yaw
subplot(3,1,3);
plot(Hist_termsIncluded.times,Plot_termsIncluded.eulerAngles(3,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.eulerAngles(3,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.eulerAngles(3,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.eulerAngles(3,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('Yaw');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.eulerAngles(3,:)-Plot_noGyroMoment.eulerAngles(3,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.eulerAngles(3,:)-Plot_noRpmDeriv.eulerAngles(3,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.eulerAngles(3,:)-Plot_noTerms.eulerAngles(3,1:sizeTermsIncluded)));
display('Yaw');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(6,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

suptitle('Crash 6 - Type B');

%% Linear Velocity, body
figure()
% u
subplot(3,1,1);
plot(Hist_termsIncluded.times,Plot_termsIncluded.linVels(1,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.linVels(1,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.linVels(1,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.linVels(1,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('u');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.linVels(1,:)-Plot_noGyroMoment.linVels(1,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.linVels(1,:)-Plot_noRpmDeriv.linVels(1,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.linVels(1,:)-Plot_noTerms.linVels(1,1:sizeTermsIncluded)));
display('u');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(7,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% v
subplot(3,1,2);
plot(Hist_termsIncluded.times,Plot_termsIncluded.linVels(2,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.linVels(2,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.linVels(2,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.linVels(2,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('v');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.linVels(2,:)-Plot_noGyroMoment.linVels(2,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.linVels(2,:)-Plot_noRpmDeriv.linVels(2,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.linVels(2,:)-Plot_noTerms.linVels(2,1:sizeTermsIncluded)));
display('v');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(8,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% w
subplot(3,1,3);
plot(Hist_termsIncluded.times,Plot_termsIncluded.linVels(3,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.linVels(3,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.linVels(3,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.linVels(3,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('w');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.linVels(3,:)-Plot_noGyroMoment.linVels(3,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.linVels(3,:)-Plot_noRpmDeriv.linVels(3,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.linVels(3,:)-Plot_noTerms.linVels(3,1:sizeTermsIncluded)));
display('w');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(9,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

suptitle('Crash 6 - Type B');

%% Angular Velocity, body
figure()
% p
subplot(3,1,1);
plot(Hist_termsIncluded.times,Plot_termsIncluded.angVels(1,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.angVels(1,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.angVels(1,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.angVels(1,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('p');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.angVels(1,:)-Plot_noGyroMoment.angVels(1,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.angVels(1,:)-Plot_noRpmDeriv.angVels(1,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.angVels(1,:)-Plot_noTerms.angVels(1,1:sizeTermsIncluded)));
display('p');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(10,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% q
subplot(3,1,2);
plot(Hist_termsIncluded.times,Plot_termsIncluded.angVels(2,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.angVels(2,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.angVels(2,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.angVels(2,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('q');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.angVels(2,:)-Plot_noGyroMoment.angVels(2,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.angVels(2,:)-Plot_noRpmDeriv.angVels(2,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.angVels(2,:)-Plot_noTerms.angVels(2,1:sizeTermsIncluded)));
display('q');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(11,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

% r
subplot(3,1,3);
plot(Hist_termsIncluded.times,Plot_termsIncluded.angVels(3,:),'b');
hold on;
plot(Hist_noTerms.times,Plot_noTerms.angVels(3,:),'r');
plot(Hist_noGyroMoment.times,Plot_noGyroMoment.angVels(3,:),'g');
plot(Hist_noRpmDeriv.times,Plot_noRpmDeriv.angVels(3,:),'m');
legend('termsIncluded','noTerms','noGyroMoment','noRpmDeriv','Location','eastoutside');
title('r');

maxError_noGyroMoment = max(abs(Plot_termsIncluded.angVels(3,:)-Plot_noGyroMoment.angVels(3,1:sizeTermsIncluded)));
maxError_noRpmDeriv = max(abs(Plot_termsIncluded.angVels(3,:)-Plot_noRpmDeriv.angVels(3,1:sizeTermsIncluded)));
maxError_noTerms = max(abs(Plot_termsIncluded.angVels(3,:)-Plot_noTerms.angVels(3,1:sizeTermsIncluded)));
display('r');
display([maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms]);
copy2Excel(12,:) = [maxError_noGyroMoment, maxError_noRpmDeriv, maxError_noTerms];

suptitle('Crash 6 - Type B');