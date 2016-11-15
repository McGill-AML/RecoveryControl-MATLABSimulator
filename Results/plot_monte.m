%% Plot Monte Carlo Results
load('icra.mat');

%% Post-processing for plots
Plot = monte2plot(Monte);
num_iter = length(Monte.trial);


%% Failures for ICs
% scatter(-Monte.fuzzyInput(:,2),Monte.xVelocity(:,1));
close all
figure
hold on;
failed = find(Monte.recovery(:,4)==0);
success = find(Monte.recovery(:,4)~=0);
p1 = plot(Monte.fuzzyInput(success,2),Monte.xVelocity(success,1),'bo','MarkerSize',3,'MarkerFaceColor','b');
p2 = plot(Monte.fuzzyInput(failed,2),Monte.xVelocity(failed,1),'rx','MarkerSize',3,'LineWidth',3);

ylabel('Incoming velocity (m/s)')
xlabel('Inclination angle (deg)')
% title('Successes and failures for 51 trials');
legend([p1 p2],'Successful recovery','Failed recovery')
grid on
% histogram(Monte.xVelocity(:,1),30,'FaceColor','w');
axis([-60 60 0.5 2.5]);
% legend('Successful Recoveries','Failed Recoveries')
%%
histogram(Monte.recovery(:,4),8)

%% Recovery Percentages
    disp([num2str(100*Plot.fractionReachedStageOne) '% crashed into the wall.']);
disp(['---> ' num2str(100*Plot.fractionReachedStageTwo) '% stabilized attitude.']);
disp(['     ---> ' num2str(100*Plot.fractionReachedStageThree) '% stabilized uprightness.']);
disp(['          ---> ' num2str(100*Plot.fractionReachedStageFour) '% stabilized vertical velocity.']);
%%
close all
hold on
% histogram(rad2deg(Plot.initAngles(2,(Monte.recovery(:,1)==0))),num_iter/10)
% histogram(Monte.fuzzyInput(:,2),-45:3:45,'FaceColor','w');
% histogram(Monte.fuzzyInput(Plot.failure,2),-45:3:45,'FaceColor','k');
% histogram(rad2deg(Plot.initAngles(2,(Plot.failure))),-50:4:50);
% histogram(rad2deg(Plot.initAngles(3,(Plot.failure))),-50:4:50);
histogram(Monte.xVelocity(:,1),30,'FaceColor','w');
histogram(Monte.xVelocity(Plot.failure,1),30,'FaceColor','k');
% % 

axis([0.4 2.6 0 500]);
% title('Inclination Angles for Failed Recoveries');
xlabel('Incoming Velocity (m/s)','Interpreter','LaTex');
ylabel('Number of trials','Interpreter','Latex');
% histogram(-1*Monte.fuzzyInput(Plot.failure,2),num_iter/10);
legend('Successful Recoveries','Failed Recoveries')
% no failures until initial pitch angle/inclination angle are greater than
% about 28 degrees toward the wall.
grid on
%% Height Loss
% close all
hold on
histogram(Monte.heightLoss(Monte.heightLoss(:,3)~=0,3),20);
% histogram(Monte.heightLoss(:,3),20);

% histogram(Monte.heightLoss(:,2),num_iter/10);
% histogram(Monte.heightLoss(:,3),num_iter/10);
% title('Height loss for 1000 trials');
xlabel('Height loss (meters)','Interpreter','Latex');
ylabel('Number of trials','Interpreter','Latex');
grid on
% half of trials recover easily
%% Horizontal Loss
% histogram(Plot.horizLoss,num_iter/10);
histogram(Monte.horizLoss(Monte.horizLoss(:,3)~=0,3),20)
xlabel('Horizontal drift (meters)','Interpreter','Latex');
ylabel('Number of trials','Interpreter','Latex');
grid on

%% Height Loss
close all
hold on
histogram(Monte.horizLoss(:,1),num_iter/10);
% histogram(Monte.horizLoss(:,2),num_iter/10);
histogram(Monte.horizLoss(:,3),num_iter/10);
title('Horizontal loss for 1000 trials');
xlabel('Horizontal loss (meters)');
ylabel('Number of trials');
%%
close all
hold on
histogram(Monte.xVelocity(:,1),20);
histogram(Monte.xVelocity(Plot.failure,1),20);
%% How long each trial took to reach stages 
close all
hold on
histogram(Plot.timeUntilStageTwo(Plot.timeUntilStageTwo~=0), 0:0.1:2);

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


