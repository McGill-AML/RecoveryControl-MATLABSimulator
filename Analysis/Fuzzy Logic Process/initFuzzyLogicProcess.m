function [ flp ] =initFuzzyLogicProcess( )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

flp = newfis('responseIntensity');

flp = addvar(flp, 'input', 'accMagHoriz', [0 8]);
flp = addmf(flp, 'input', 1, 'accVeryLow', 'trimf', [0 0 2]);
flp = addmf(flp, 'input', 1, 'accLow', 'trimf', [0 2 4]);
flp = addmf(flp, 'input', 1, 'accMedium', 'trimf', [2 4 6]);
flp = addmf(flp, 'input', 1, 'accHigh', 'trapmf', [4 6 8 8]);

flp = addvar(flp, 'input', 'inclination', [-45 45]);
flp = addmf(flp, 'input', 2, 'inclinedAway', 'trapmf', [-45 -45 -7.5 0]);
flp = addmf(flp, 'input', 2, 'inclinedLevel', 'trimf', [-7.5 7.5 22.5]);
flp = addmf(flp, 'input', 2, 'inclinedToward', 'trapmf', [15 22.5 45 45]);

flp = addvar(flp, 'input', 'gamma', [0 180]);
flp = addmf(flp, 'input', 3, 'flipAway', 'trapmf', [0 0 50 90]);
flp = addmf(flp, 'input', 3, 'flipSideway', 'trimf', [50 90 130]);
flp = addmf(flp, 'input', 3, 'flipToward', 'trapmf', [90 130 180 180]);

flp = addvar(flp, 'input', 'gyroHorizMag', [0 10]);
flp = addmf(flp, 'input', 4, 'gyroLow', 'trapmf', [0 0 2 4]);
flp = addmf(flp, 'input', 4, 'gyroMedium', 'trimf', [2 5 8]);
flp = addmf(flp, 'input', 4, 'gyroHigh', 'trapmf', [6 8 10 10]);

flp = addvar(flp, 'output', 'responseIntensity', [-1 1]);
flp = addmf(flp, 'output', 1, 'towardBig', 'trimf', [-1 -1 -0.5]);
flp = addmf(flp, 'output', 1, 'towardSmall', 'trimf', [-1 -0.5 0]);
flp = addmf(flp, 'output', 1, 'level', 'trimf', [-0.5 0 0.5]);
flp = addmf(flp, 'output', 1, 'awaySmall', 'trimf', [0 0.5 1]);
flp = addmf(flp, 'output', 1, 'awayBig', 'trimf', [0.5 1 1]);

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

end

