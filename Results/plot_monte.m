%% Plot Monte Carlo Results
load('recovery.mat');
%%
% Monte.trial = trial;
% Monte.IC = IC;
% Monte.recovery = recovery;
% Monte.heightLoss = heightLoss;
% Monte.horizLoss = horizLoss;
 
Plot = monte2plot(Monte);
%%
num_iter = 1000;

%% Recovery Percentages

disp([num2str(100*Plot.fractionReachedStageOne) '% crashed into the wall.']);
disp(['---> ' num2str(100*Plot.fractionReachedStageTwo) '% stabilized attitude.']);
disp(['     ---> ' num2str(100*Plot.fractionReachedStageThree) '% stabilized uprightness.']);
disp(['          ---> ' num2str(100*Plot.fractionReachedStageFour) '% stabilized vertical velocity.']);
%%

no_crash = Plot.trial(Monte.recovery(:,1)==0);
no_recover = Plot.trial(Plot.timeUntilStageTwo==0);
failure = no_recover(ismember(no_recover,no_crash)==0);

hold on
histogram(rad2deg(Plot.initAngles(2,(Monte.recovery(:,1)==0))),num_iter/10)
histogram(rad2deg(Plot.initAngles(2,(failure))),num_iter/10);

% trials that didn't crash into the wall: 112 of them
axis([-60 60 0 5]);
title('No Crash and No Recovery Initial Pitch Angles');
xlabel('Initial Pitch Angle (deg)');
ylabel('Number of trials');
histogram(-1*Monte.inclination(failure),num_iter/10);

legend('No crash','No recovery pitch','No recovery inclination');
% Alternately Inclination angle

% Note: no correspondance between roll/yaw and failure cases

%% Height Loss
histogram(Monte.heightLoss(Monte.heightLoss~=0),num_iter/10);
title('Height loss for 1000 trials');
xlabel('Height loss (meters)');
ylabel('Number of trials');
 
%% Horizontal Loss
% histogram(Plot.horizLoss,num_iter/10);
histogram(Monte.horizLoss(Monte.horizLoss~=0),num_iter)
title('Horizontal loss for 1000 trials');
xlabel('Horizontal loss (meters)');
ylabel('Number of trials');


%% How long each trial took to reach stages two and three
histogram(Plot.timeUntilStageTwo(Plot.timeUntilStageTwo~=0), num_iter/10);
hold on
% plot how long each trial took to reach stage three
histogram(Plot.timeUntilStageThree(Plot.timeUntilStageThree~=0), num_iter/10);
title('Time to reach stages two and three');
xlabel('Time(seconds)');
ylabel('Number of trials');

%% Plot ICs for trials which go less than 0.05 horizontally and vertically



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



