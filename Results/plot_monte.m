%% Plot Monte Carlo Results
load('impacts_aug_19_with_fuzzy.mat');

%% Post-processing for plots
Plot = monte2plot(Monte);
num_iter = length(Monte.trial);

%% Recovery Percentages
    disp([num2str(100*Plot.fractionReachedStageOne) '% crashed into the wall.']);
disp(['---> ' num2str(100*Plot.fractionReachedStageTwo) '% stabilized attitude.']);
disp(['     ---> ' num2str(100*Plot.fractionReachedStageThree) '% stabilized uprightness.']);
disp(['          ---> ' num2str(100*Plot.fractionReachedStageFour) '% stabilized vertical velocity.']);
%%
close all
hold on
% histogram(rad2deg(Plot.initAngles(2,(Monte.recovery(:,1)==0))),num_iter/10)
histogram(rad2deg(Plot.initAngles(2,(Plot.failure))),-50:4:50);
% histogram(rad2deg(Plot.initAngles(2,(Plot.failure))),-50:4:50);
% histogram(rad2deg(Plot.initAngles(3,(Plot.failure))),-50:4:50);

% % 

axis([-50 50 0 25]);
title('Initial Pitch Angles for Failures');
xlabel('Initial Pitch Angle (deg)');
ylabel('Number of trials');
% histogram(-1*Monte.fuzzyInput(Plot.failure,2),num_iter/10);

legend('Initial Pitch');

% no failures until initial pitch angle/inclination angle are greater than
% about 28 degrees toward the wall.

%% Height Loss
close all
hold on
histogram(Monte.heightLoss(:,1),10);
% histogram(Monte.heightLoss(:,2),num_iter/10);
% histogram(Monte.heightLoss(:,3),num_iter/10);
title('Height loss for 1000 trials');
xlabel('Height loss (meters)');
ylabel('Number of trials');

% half of trials recover easily
%% Horizontal Loss
% histogram(Plot.horizLoss,num_iter/10);
histogram(Monte.horizLoss(Monte.horizLoss(:,1)~=0,1),10)
hold on

%% Height Loss
close all
hold on
histogram(Monte.horizLoss(:,1),num_iter/10);
% histogram(Monte.horizLoss(:,2),num_iter/10);
histogram(Monte.horizLoss(:,3),num_iter/10);
title('Horizontal loss for 1000 trials');
xlabel('Horizontal loss (meters)');
ylabel('Number of trials');

%% How long each trial took to reach stages 
close all
hold on
histogram(Plot.timeUntilStageTwo(Plot.timeUntilStageTwo~=0), 0:0.1:2);
histogram(Plot.timeUntilStageThree(Plot.timeUntilStageThree~=0), 0:0.1:2);
histogram(Plot.timeUntilStageFour(Plot.timeUntilStageFour~=0), 0:0.1:2);

title('Time to reach each recovery stage');
xlabel('Time(seconds)');
ylabel('Number of trials');

%% Segmentation 

%% Matching ICs of trials with stage two time in second peak
 
secondWaveTrials = Plot.trial(Plot.timeUntilStageTwo > 1.2);
 
incomingRoll = rad2deg(Plot.initAngles(1,:));
incomingPitch = rad2deg(Plot.initAngles(2,:));
incomingYaw = rad2deg(Plot.initAngles(3,:));
 
subplot(2,1,1);
bar(incomingPitch);
title('Trials which took LESS than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Pitch Angles');
axis([0 1000 -90 90]);
subplot(2,1,2);
bar(incomingPitch(secondWaveTrials));
title('Trials which took MORE than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Pitch Angles');
axis([0 290 -90 90]);

%% Final horizontal velocity
close all
hold on
histogram(Monte.finalHorizVel(Monte.trial(Monte.recovery(:,1)<0.1)),num_iter/10)
% histogram(Monte.finalHorizVel,num_iter/10)

%% Plot initial pitch for trials that took more than 0.5 seconds to recover
close all
histogram(rad2deg(Plot.initAngles(2, Monte.trial(Monte.recovery(:,2)<0.1))),num_iter/10)


%% Plot correlation to incoming roll angle
% subplot(2,1,1);
% bar(incomingRoll);
% title('Trials which took LESS than 0.6 seconds to reach stage two');
% xlabel('Trials');
% ylabel('Incoming Roll Angles');
% axis([0 1000 -90 90]);
% subplot(2,1,2);
% bar(incomingRoll(secondWaveTrials));
% title('Trials which took MORE than 0.6 seconds to reach stage two');
% xlabel('Trials');
% ylabel('Incoming Roll Angles');
% axis([0 290 -90 90]);



% subplot(2,1,1);
% bar(incomingYaw);
% title('Trials which took LESS than 0.6 seconds to reach stage two');
% xlabel('Trials');
% ylabel('Incoming Yaw Angles');
% axis([0 1000 -90 90]);
% subplot(2,1,2);
% bar(incomingYaw(secondWaveTrials));
% title('Trials which took MORE than 0.6 seconds to reach stage two');
% xlabel('Trials');
% ylabel('Incoming Yaw Angles');
% axis([0 290 -90 90]);
 
% Height loss for second wave trials
% subplot(2,1,1);
% histogram(Plot.heightLoss,num_iter/10);
% title('Height loss for 1000 trials');
% xlabel('Height loss (meters)');
% ylabel('Number of trials');
% axis([-10 0 0 300]);
% subplot(2,1,2);
% histogram(Plot.heightLoss(secondWaveTrials),num_iter/10);
% title('Height loss for 1000 trials');
% xlabel('Height loss (meters)');
% ylabel('Number of trials');
% axis([-10 0 0 300]);



