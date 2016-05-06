% % ICUAS Crash 3 (Crash 6), type B response
% simSnapTimes = [0; 26; 40; 53; 66; 73];
% snapshotView = 'C6';

% ICUAS Crash 6 (Crash 10), type D response
simSnapTimes = [0; 14; 20; 26; 43; 50];
simSnapTimes = simSnapTimes + 6;
snapshotView = 'CT';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Crash 3 ****
% simSnapTimes = [0; 6; 20; 26; 33; 46];
% snapshotView = 'C3';

% % Crash 5
% simSnapTimes = [0; 6; 14; 20; 26; 33; 40; 46; 53; 60; 66; 73; 79];
% snapshotView = 'C5';

% % Crash 6
% simSnapTimes = [0; 6; 14; 26; 33; 40; 46; 53; 60; 66; 73; 80; 86; 93; 100; 106; 113];
% snapshotView = 'C6';

% % Crash 7
% simSnapTimes = [0; 14; 20; 26; 33; 40; 46; 53; 66; 73; 76; 80; 86; 93];
% snapshotView = 'C7';

% % Crash 9
% simSnapTimes = [0; 14; 20; 26; 36; 38; 40; 46; 53; 60; 66; 73; 80; 86; 93];
% simSnapTimes = simSnapTimes + 3;
% snapshotView = 'C9';

% % Crash 10
% simSnapTimes = [0; 6; 14; 20; 26; 40; 43; 46; 50; 53];
% simSnapTimes = simSnapTimes + 6;
% snapshotView = 'CT';
% 
% % Crash 11
% simSnapTimes = [0; 6; 21; 31; 50];
% snapshotView = 'VV';

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(1),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_1 -transparent

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(2),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_2 -transparent

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(3),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_3 -transparent

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(4),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_4 -transparent

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(5),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_5 -transparent

snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(6),Hist,snapshotView,ImpactParams,timeImpact)
export_fig fig_typeDsim_6 -transparent

% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(7),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_7 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(8),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_8 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(9),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_9 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(10),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_10 -transparent

% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(11),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_11 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(12),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_12 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(13),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_13 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(14),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_14 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(15),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_15 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(16),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_16 -transparent
% 
% snapshot(vlookup(Hist.times,timeImpact)+simSnapTimes(17),Hist,snapshotView,ImpactParams,timeImpact)
% export_fig fig_typeDsim_17 -transparent