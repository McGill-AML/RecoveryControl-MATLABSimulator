function [ flp ] =initfuzzylogicprocess( )
%Initialize membership functions and rules for Fuzzy Logic Process

%%
flp = newfis('responseIntensity');

flp = addvar(flp, 'input', 'accMagHoriz', [0 8]);
flp = addmf(flp, 'input', 1, 'accVeryLow', 'trimf', [0 0 2]);
flp = addmf(flp, 'input', 1, 'accLow', 'trimf', [0 2 4]);
flp = addmf(flp, 'input', 1, 'accMedium', 'trimf', [2 4 6]);
flp = addmf(flp, 'input', 1, 'accHigh', 'trapmf', [4 6 8 8]);

% flp = addvar(flp, 'input', 'forceExternalMag', [0 8]*9.81);
% flp = addmf(flp, 'input', 1, 'forceVeryLow', 'trimf', [0 0 2]*9.81);
% flp = addmf(flp, 'input', 1, 'forceLow', 'trimf', [0 2 4]*9.81);
% flp = addmf(flp, 'input', 1, 'forceMedium', 'trimf', [2 4 6]*9.81);
% flp = addmf(flp, 'input', 1, 'forceHigh', 'trapmf', [4 6 8 8]*9.81);

% flp = addvar(flp, 'input', 'forceExternalMag', [0 40]);
% flp = addmf(flp, 'input', 1, 'forceVeryLow', 'trapmf', [0 0 10 15]);
% flp = addmf(flp, 'input', 1, 'forceLow', 'trimf', [10 15 20]);
% flp = addmf(flp, 'input', 1, 'forceMedium', 'trimf', [15 20 25]);
% flp = addmf(flp, 'input', 1, 'forceHigh', 'trapmf', [20 25 40 40]);

flp = addvar(flp, 'input', 'inclination', [-60 60]);
flp = addmf(flp, 'input', 2, 'inclinedAway', 'trapmf', [-90 -60 -15 0]);
flp = addmf(flp, 'input', 2, 'inclinedLevel', 'trimf', [-10 0 10]);
flp = addmf(flp, 'input', 2, 'inclinedToward', 'trapmf', [0 15 60 90]);

% flp = addvar(flp, 'input', 'gamma', [0 180]);
% flp = addmf(flp, 'input', 3, 'flipAway', 'trapmf', [0 0 50 90]);
% flp = addmf(flp, 'input', 3, 'flipSideway', 'trimf', [50 90 130]);
% flp = addmf(flp, 'input', 3, 'flipToward', 'trapmf', [90 130 180 180]);

flp = addvar(flp, 'input', 'gamma', [0 180]);
flp = addmf(flp, 'input', 3, 'flipAway', 'trapmf', [0 0 50 90]);
flp = addmf(flp, 'input', 3, 'flipSideway', 'trimf', [70 90 110]);
flp = addmf(flp, 'input', 3, 'flipToward', 'trapmf', [90 130 180 180]);

flp = addvar(flp, 'input', 'gyroHorizMag', [0 10]);
flp = addmf(flp, 'input', 4, 'gyroLow', 'trapmf', [0 0 1 2]);
flp = addmf(flp, 'input', 4, 'gyroMedium', 'trimf', [1 3 6]);
flp = addmf(flp, 'input', 4, 'gyroHigh', 'trapmf', [4 6 10 10]);

% flp = addvar(flp, 'output', 'responseIntensity', [-1 1]);
% flp = addmf(flp, 'output', 1, 'towardBig', 'trimf', [-1 -1 -0.5]);
% flp = addmf(flp, 'output', 1, 'towardSmall', 'trimf', [-1 -0.5 0]);
% flp = addmf(flp, 'output', 1, 'level', 'trimf', [-0.5 0 0.5]);
% flp = addmf(flp, 'output', 1, 'awaySmall', 'trimf', [0 0.5 1]);
% flp = addmf(flp, 'output', 1, 'awayBig', 'trimf', [0.5 1 1]);


flp = addvar(flp, 'output', 'responseIntensity', [-1 1]);
flp = addmf(flp, 'output', 1, 'towardBig', 'trapmf', [-1 -1 -0.75 -0.5]);
flp = addmf(flp, 'output', 1, 'towardSmall', 'trimf', [-0.7 -0.4 -0.1]);
flp = addmf(flp, 'output', 1, 'level', 'trimf', [-0.3 0 0.3]);
flp = addmf(flp, 'output', 1, 'awaySmall', 'trimf', [0.1 0.4 0.7]);
flp = addmf(flp, 'output', 1, 'awayBig', 'trapmf', [0.5 0.75 1 1]);

ruleList = [1	0	0	0	3	1	1; ...
            0	2	0	0	3	1	1; ...
            2	3	0	0	2	1	1; ...
            2	1	0	0	3	1	1; ...
            3	3	0	0	1	1	1; ...
            3	1	0	0	4	1	1; ...
            4	3	0	0	1	1	1; ...
            4	1	0	0	5	1	1; ...
            0	0	0	1	3	1	1; ...
            0	0	3	2	2	1	1; ... 
            0	0	3	3	1	1	1; ...
            0	0	1	2	4	1	1; ...
            0	0	1	3	5	1	1];
flp = addrule(flp,ruleList);

%% Plot membership functions
% figure();
% plotmf(flp,'input',1);
% figure();
% plotmf(flp,'input',2);
% figure();
% plotmf(flp,'input',3);
% figure();
% plotmf(flp,'input',4);
% figure();
% plotmf(flp,'output',1);


end

