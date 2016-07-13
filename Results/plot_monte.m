%% Plot Monte Carlo Results
load('Monte_carlo_1000.mat');
Monte.trial = trial;
Monte.IC = IC;
Monte.recovery = recovery;
Monte.heightLoss = heightLoss;
Monte.horizLoss = horizLoss;

Plot = monte2plot(Monte);
num_iter = 1000;

%% Facts
 
% fraction reached Stage One:   1.0000;
% fraction reached Stage Two:   0.9120;
% fraction Reached Stage Three: 0.9120;
% fraction Reached Stage Four:  0.8830;

% Conclusion: if it reaches stages two it always reaches stage three
% 90% success rate for incoming angles up to 72 degrees -- not bad
%% Height Loss
histogram(Plot.heightLoss,num_iter/10);
title('Height loss for 1000 trials');
xlabel('Height loss (meters)');
ylabel('Number of trials');

%% Horizontal Loss
histogram(Plot.horizLoss,num_iter/10);
title('Horizontal loss for 1000 trials');
xlabel('Horizontal loss (meters)');
ylabel('Number of trials');

%% How long each trial took to reach stages two and three
histogram(Plot.timeUntilStageTwo, num_iter/10);
hold on
% plot how long each trial took to reach stage three
histogram(Plot.timeUntilStageThree, num_iter/10);
title('Time to reach stages two and three');
xlabel('Time(seconds)');
ylabel('Number of trials');
%% Matching ICs of trials with stage two time in second peak

secondWaveTrials = Plot.trial(Plot.timeUntilStageTwo > 0.6);

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

%% Plot correlation to incoming roll angle
subplot(2,1,1);
bar(incomingRoll);
title('Trials which took LESS than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Roll Angles');
axis([0 1000 -90 90]);
subplot(2,1,2);
bar(incomingRoll(secondWaveTrials));
title('Trials which took MORE than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Roll Angles');
axis([0 290 -90 90]);
%% Plot correlation to incoming yaw angle

subplot(2,1,1);
bar(incomingYaw);
title('Trials which took LESS than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Yaw Angles');
axis([0 1000 -90 90]);
subplot(2,1,2);
bar(incomingYaw(secondWaveTrials));
title('Trials which took MORE than 0.6 seconds to reach stage two');
xlabel('Trials');
ylabel('Incoming Yaw Angles');
axis([0 290 -90 90]);

%% Height loss for second wave trials
subplot(2,1,1);
histogram(Plot.heightLoss,num_iter/10);
title('Height loss for 1000 trials');
xlabel('Height loss (meters)');
ylabel('Number of trials');
axis([-10 0 0 300]);
subplot(2,1,2);
histogram(Plot.heightLoss(secondWaveTrials),num_iter/10);
title('Height loss for 1000 trials');
xlabel('Height loss (meters)');
ylabel('Number of trials');
axis([-10 0 0 300]);
