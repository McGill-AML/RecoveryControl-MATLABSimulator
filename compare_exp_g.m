% 
% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g03_alldata.mat')
% timeImpact_exp_vicon = 54.69;
% zOffset = 0.9191-(0.6);
% timeImpact_exp_px4 = 648742128;
% yawExp = rad2deg(1.978);

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g08_alldata.mat')
% timeImpact_exp_vicon = 38.32;
% zOffset = 0.9191-(0.6);
% timeImpact_exp_px4 = 807622822;
% yawExp = 117.8;

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g10_alldata.mat')
% timeImpact_exp_vicon = 32.48;
% zOffset = 0.8975-(0.6);
% timeImpact_exp_px4 = 664945969;
% yawExp = 117;
% crashStr = 'Crash G10';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g11_alldata.mat')
% timeImpact_exp_vicon = 24.73;
% zOffset = 1.038-(0.6);
% timeImpact_exp_px4 = 799355062;
% yawExp = 116.7;
% crashStr = 'Crash G11';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g12_alldata.mat')
% timeImpact_exp_vicon = 26.83;
% zOffset = 1.194-(0.6);
% timeImpact_exp_px4 = 975062555;
% yawExp = 119.2;
% crashStr = 'Crash G12';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g13_alldata.mat')
% timeImpact_exp_vicon = 34.99;
% zOffset = 0.9925-(0.6);
% timeImpact_exp_px4 = 581185996;
% yawExp = 112.4;
% crashStr = 'Crash G13';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g14_alldata.mat')
% timeImpact_exp_vicon = 26.05;
% zOffset = 0.9925-(0.6);
% timeImpact_exp_px4 = 754053126;
% yawExp = 115;
% crashStr = 'Crash G14';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g15_alldata.mat')
% timeImpact_exp_vicon = 33.8;
% xOffset = 0.7955 - Plot.posns(1,vlookup(Plot.times,timeImpact));
% yOffset = -0.01833 - Plot.posns(2,vlookup(Plot.times,timeImpact));
% zOffset = 1.074-(0.6);
% timeImpact_exp_px4 = 859865204;
% yawExp = 113.9;
% crashStr = 'Crash G15';
% 
% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g16_alldata.mat')
% timeImpact_exp_vicon = 38.88;
% zOffset = 1.113-(0.6);
% timeImpact_exp_px4 = 987647087;
% yawExp = 113.3;
% crashStr = 'Crash G16';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g17_alldata.mat')
% timeImpact_exp_vicon = 25.184373911000002;
% zOffset = 0.979-(0.6);
% timeImpact_exp_px4 = 1.102892288000000e+09;
% yawExp = 113.5;
% crashStr = 'Crash G17';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g19_alldata.mat')
% timeImpact_exp_vicon = 12.54;
% zOffset = 0.8039-(0.6);
% timeImpact_exp_px4 = 1.394267737000000e+09;
% yawExp = 111.3;
% crashStr = 'Crash G19';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g20_alldata.mat')
% timeImpact_exp_vicon = 31.39;
% zOffset = 0.9183-(0.6);
% timeImpact_exp_px4 = 1.504168296000000e+09;
% yawExp = 112.7;
% crashStr = 'Crash G20';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g21_alldata.mat')
% timeImpact_exp_vicon = 18.2;
% zOffset = 0.959-(0.6);
% timeImpact_exp_px4 = 1.638942751000000e+09;
% yawExp = 113.5;
% crashStr = 'Crash G21';


% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g22_alldata.mat')
% timeImpact_exp_vicon = 25.96;
% zOffset = 1.2-(0.6);
% timeImpact_exp_px4 = 321267268;
% yawExp = 113.7;
% crashStr = 'Crash G22';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g23_alldata.mat')
% timeImpact_exp_vicon = 32.4;
% zOffset = 1.364-(0.6);
% timeImpact_exp_px4 = 420240157;
% yawExp = 115.3;
% crashStr = 'Crash G23';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g24_alldata.mat')
% timeImpact_exp_vicon = 28.44;
% zOffset = 1.442-(1);
% timeImpact_exp_px4 = 554540272;
% yawExp = 114.4;
% crashStr = 'Crash G24';

load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g25_alldata.mat')
timeImpact_exp_vicon = 37.17;
xOffset = 0.8037 - Plot.posns(1,vlookup(Plot.times,timeImpact));
yOffset = 0.0136 - Plot.posns(2,vlookup(Plot.times,timeImpact));
zOffset = 1.612-(0.6);
timeImpact_exp_px4 = 737873619;
yawExp = 116.4;
crashStr = 'Crash G25';

% load('/home/thread/fmchui/Spiri/Collison Experiments/2016_07_20 - VII/Combined Data/g26_alldata.mat')
% timeImpact_exp_vicon = 26.05;
% zOffset = 0.9925-(0.6);
% timeImpact_exp_px4 = 974057648;
% yawExp = 0;
% crashStr = 'Crash G26';


%%

yaw_world = zeros(numel(Plot.times),1);

for i = 1:numel(Plot.times)
    yaw_correction = Plot.eulerAngles(3,i) - deg2rad(65);
    if yaw_correction < -pi
        yaw_correction = 2*pi+(yaw_correction);
    elseif yaw_correction > pi
        yaw_correction = yaw_correction - 2*pi;
    end
    yaw_world(i) = yaw_correction;
end

%% Plot posns

figure()
subplot(2,1,1)
plot(v_vicon__pose___time-timeImpact_exp_vicon,[v_vicon__pose_pose_position_x,...
                                                v_vicon__pose_pose_position_y+1.62,...
                                                v_vicon__pose_pose_position_z]);
hold on;
plot(Plot.times-timeImpact,[Plot.posns(1,:)+xOffset;Plot.posns(2,:)+yOffset;Plot.posns(3,:)+zOffset],'--')
xlim([0 0.5])

legend('X,exp','Y,exp','Z,exp','X,sim','Y,sim','Z,sim','Location','eastoutside');
title('Comparison of Immediate Position Response');
ylabel('Position (m)');
xlabel('Time After First Impact (s)');
grid on

%% Plot attitudes
if yawExp == 0
    yawOffset = 0;
else
    yawOffset = yawExp - rad2deg(yaw_world(vlookup(Plot.times,timeImpact)));
end

subplot(2,1,2)
plot((TIMEStartTime - timeImpact_exp_px4)/1e6,[ATTRoll,ATTPitch,ATTYaw-deg2rad(yawOffset)]*180/pi);
hold on;
plot(Plot.times-timeImpact,Plot.eulerAngles(1:2,:)*180/pi,'--');
plot(Plot.times-timeImpact,yaw_world*180/pi,'r--');
xlim([0 0.5])

legend('\phi,exp','\theta,exp','\psi,exp','\phi,sim','\theta,sim','\psi,sim','Location','eastoutside');
title('Comparison of Immediate Attitude Response');
ylabel('Angle (deg)');
xlabel('Time After First Impact (s)');
grid on

suptitle(strcat(crashStr,' Comparison'));

%% Find posn errors
vicon_startIdx = vlookup(v_vicon__pose___time,timeImpact_exp_vicon);
vicon_endIdx = vlookup(v_vicon__pose___time,timeImpact_exp_vicon+0.5);
posn_err = zeros(3,vicon_endIdx-vicon_startIdx);

for iVicon = vicon_startIdx:vicon_endIdx
    vicon_timeAfterImpact = v_vicon__pose___time(iVicon) - timeImpact_exp_vicon;
    posn_err_x = v_vicon__pose_pose_position_x(iVicon) - Plot.posns(1,vlookup(Plot.times,timeImpact+vicon_timeAfterImpact))  - xOffset;
    posn_err_y = v_vicon__pose_pose_position_y(iVicon) + 1.62 - Plot.posns(2,vlookup(Plot.times,timeImpact+vicon_timeAfterImpact))  - yOffset;
    posn_err_z = v_vicon__pose_pose_position_z(iVicon) - Plot.posns(3,vlookup(Plot.times,timeImpact+vicon_timeAfterImpact)) - zOffset;

    posn_err(:,iVicon-vicon_startIdx+1) = [posn_err_x;posn_err_y;posn_err_z];
end

figure
subplot(2,1,1)
plot(v_vicon__pose___time(vicon_startIdx:vicon_endIdx)-timeImpact_exp_vicon,posn_err);
legend('X','Y','Z','Location','eastoutside');
title('Immediate Position Response Errors');
ylabel('Position (m)');
xlabel('Time After First Impact (s)');
xlim([0 0.5])
grid on

%% Find att errors
sim_startIdx = vlookup(Plot.times,timeImpact);
sim_endIdx = min(vlookup(Plot.times,timeImpact+0.5),numel(Plot.times));
att_err = zeros(3,sim_endIdx-sim_startIdx);

for iSim = sim_startIdx:sim_endIdx
    sim_timeAfterImpact = Plot.times(iSim) - timeImpact;
    
    real_roll = ATTRoll(vlookup(TIMEStartTime,timeImpact_exp_px4+sim_timeAfterImpact*1e6));
    real_pitch = ATTPitch(vlookup(TIMEStartTime,timeImpact_exp_px4+sim_timeAfterImpact*1e6));
    real_yaw = ATTYaw(vlookup(TIMEStartTime,timeImpact_exp_px4+sim_timeAfterImpact*1e6)) -deg2rad(yawOffset);

    att_err_roll = (real_roll - Plot.eulerAngles(1,iSim))*180/pi;
    att_err_pitch = (real_pitch - Plot.eulerAngles(2,iSim))*180/pi;
    att_err_yaw = (real_yaw - yaw_world(iSim))*180/pi;

    att_err(:,iSim-sim_startIdx+1) = [att_err_roll;att_err_pitch;att_err_yaw];

end

subplot(2,1,2)
plot(Plot.times(sim_startIdx:sim_endIdx)-timeImpact,att_err);
legend('\phi','\theta','\psi','Location','eastoutside');
title('Immediate Attitude Response Errors');
ylabel('Angle (deg)');
xlabel('Time After First Impact (s)');

xlim([0 0.5])
grid on

suptitle(strcat(crashStr,' Errors'));

