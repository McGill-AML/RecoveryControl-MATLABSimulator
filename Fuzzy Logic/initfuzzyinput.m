function [FuzzyInfo, PREIMPACT_ATT_CALCSTEPFWD] = initfuzzyinput()
%initfuzzyinput.m Initialize FLP input calculation times and FLP object in MATLAB
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

% Number of previous steps to calculate pre-impact attitude at
PREIMPACT_ATT_CALCSTEPFWD = 2; %2

%% FLP input 1: External Force Estimation Magnitude
FuzzyInput.ID = 1;
FuzzyInput.calcStepDelay = 0.010; %10 to 15 ms
FuzzyInput.value = 0;
FuzzyInfo.InputArray = FuzzyInput;

%% FLP input 2: Inclination
FuzzyInput.ID = 2;
FuzzyInput.calcStepDelay = 0; %available at t_detection
FuzzyInput.value = 0;
FuzzyInfo.InputArray = [FuzzyInfo.InputArray; FuzzyInput];

%% FLP input 3: Flipping Direction Angle
FuzzyInput.ID = 3;
FuzzyInput.calcStepDelay = 0.010; %5 to 10 ms
FuzzyInput.value = 0;
FuzzyInfo.InputArray = [FuzzyInfo.InputArray; FuzzyInput];

%% FLP input 4: Gyro Horizontal Magnitude
FuzzyInput.ID = 4;
FuzzyInput.calcStepDelay = 0.015; %10 to 15 ms
FuzzyInput.value = 0;
FuzzyInfo.InputArray = [FuzzyInfo.InputArray; FuzzyInput];

%% Keep track of which inputs have been calculated
FuzzyInfo.InputsCalculated = [0;0;0;0];

%% Fuzzy Output
FuzzyInfo.output = 0;

%% Initialize FLP
FuzzyInfo.intensityFuzzyProcess = initfuzzylogicprocess();