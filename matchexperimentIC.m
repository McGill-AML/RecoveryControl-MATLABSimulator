function [posnDeriv, attEuler, altitude, setpointAltitude, xAcc, Experiment] = matchexperimentIC(crash)

global m g globalFlag


switch crash
    case 3 % ---- Crash 3 ---- %
        posnDeriv = 0.6856;
        attEuler = [deg2rad(-1.3);deg2rad(10);-pi + deg2rad(26)];
        altitude = 0.2;
        setpointAltitude = 1.1; %roll0 = -6: 2.1
        xAcc = 1.7170;%1.4896; %pitch0 = 6.3: 1.489

        rawManualCmd.times = [0;0.5];
        rawManualCmd.rolls = [deg2rad(2.5);0];
        rawManualCmd.pitches = [deg2rad(3.5);0];
        rawManualCmd.yawDerivs = [deg2rad(-10);0];
        rawManualCmd.thrusts = [-m*9.12;-m*g];        
               
        inputData = load('crash3_motorslopes.mat');
        
        display('Did you remember to change Fc2 = 2*Fc2?');
%         Change Fc2 = 2*Fc2 in SpiriMotion_4Circles_att

    case 5  % ---- Crash 5 ---- %
        posnDeriv = 1.1056;
        attEuler = [deg2rad(0.6); deg2rad(4.4); -pi + deg2rad(4.6)];
        altitude = 0.2;
        setpointAltitude = 0.51; %roll0 = -6: 2.1
        xAcc = 0.7811;%1.4896; %pitch0 = 6.3: 1.4896

        rawManualCmd.times = [0;0.2];
        rawManualCmd.rolls = [deg2rad(2.5);0];
        rawManualCmd.pitches = [deg2rad(3.5);0];
        rawManualCmd.yawDerivs = [0;0];
        rawManualCmd.thrusts = [-m*g;-m*g];

        inputData = load('crash5_motorslopes.mat');
        
    case 6 % ---- Crash 6 ---- %
        posnDeriv = 1.15;%1.17;
        attEuler = [deg2rad(4); deg2rad(6); -pi + deg2rad(11)];
        altitude = 0.7;
        setpointAltitude = 1.37; %roll0 = -6: 2.1
        xAcc = 1.2595;%1.4896; %pitch0 = 6.3: 1.4896

        rawManualCmd.times = [0;0.4];
        rawManualCmd.rolls = [deg2rad(3);0];
        rawManualCmd.pitches = [deg2rad(5.5);0];
        rawManualCmd.yawDerivs = [deg2rad(-1);0];
        rawManualCmd.thrusts = [-m*g;-m*g];

        inputData = load('crash6_motorslopes.mat');
        
    case 7 % ---- Crash 7 ---- %
        posnDeriv = 1.351;
        attEuler = [deg2rad(-5.1); deg2rad(6.7); -pi + deg2rad(8.86)]; 
        altitude = 0.6;
        setpointAltitude = 0.88; %roll0 = -6: 2.1
        xAcc =1.0580;%1.4896; %pitch0 = 6.3: 1.4896

        rawManualCmd.times = [0;0.35];
        rawManualCmd.rolls = [deg2rad(-1);0];
        rawManualCmd.pitches = [deg2rad(8);0];
        rawManualCmd.yawDerivs = [deg2rad(7);0];
        rawManualCmd.thrusts = [-m*10.11;-m*g];

        inputData = load('crash7_motorslopes.mat');
        
    case 9 % ---- Crash 9 ---- %
        posnDeriv = 2.0283;
        attEuler = [deg2rad(-2.7); deg2rad(9.4); -pi + deg2rad(17)]; 
        altitude = 0.6;
        setpointAltitude = 1.02; 
        xAcc = 1.5356;

        rawManualCmd.times = [0;0.2];
        rawManualCmd.rolls = [deg2rad(2);0];
        rawManualCmd.pitches = [deg2rad(12);0];
        rawManualCmd.yawDerivs = [0;0];
        rawManualCmd.thrusts = [-m*g;-m*g];

        inputData = load('crash9_motorslopes.mat');
        
    case 10 % ---- Crash 10 ---- %
        posnDeriv = 2.6799;
        attEuler = [deg2rad(-1.2);deg2rad(16);-pi + deg2rad(4)];
        altitude = 0.6;
        setpointAltitude = 0.66; 
        xAcc = 2.7158;

        rawManualCmd.times = [0;0.65];
        rawManualCmd.rolls = [deg2rad(2);0];
        rawManualCmd.pitches = [deg2rad(11);0];
        rawManualCmd.yawDerivs = [deg2rad(3);0];
        rawManualCmd.thrusts = [-m*g;-m*g];

        inputData = load('crash10_motorslopes.mat');
        
    case 11 % ---- Crash 11 ---- %
        posnDeriv = 1.98; %1.98
        attEuler = [deg2rad(-4.4+2.2); deg2rad(20.2);-pi + deg2rad(11-2.5)]; 
        altitude = 0;
        setpointAltitude = 0.92; %1.08
        xAcc = 3.8768; %roll0 = -6: 3.995, roll0 = 6: 4.891

        rawManualCmd.times = [0;0.14;0.3];
        rawManualCmd.rolls = [deg2rad(0.6883);deg2rad(0.6883);0];
        rawManualCmd.pitches = [deg2rad(12);deg2rad(12);0];
        rawManualCmd.yawDerivs = [deg2rad(-0.1337);0;0];
        rawManualCmd.thrusts = [-m*10.5;-m*10.5;-m*g];

        inputData.motors_time = [0;0.0616;0.0984;0.2084];
        inputData.motors_slope = [[-3863.6;-20897;-2345.5;0],[0;0;0;0],...
                        [-31218;-9592.4;0;0],[-1704.5;13342;0;0]];
    otherwise
        error('Invalid experiment crash number');
end

Experiment = matchexperimentcmds(rawManualCmd,inputData);

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,size(inputData.motors_time,1));

end