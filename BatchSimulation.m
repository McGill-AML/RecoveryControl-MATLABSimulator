% traj_posns1 = [0 0 0;0 0 1;0 0 0;0 0 2;0 0 0;0 0 5;0 0 0;0 0 10];
% traj_heads1 = [0;0;0;0;0;0;0;0];
% traj_times1 = [0;5;0;5;0;5;0;5];
% 
% traj_posns2 = [0 0 0;1 0 0;0 0 0;2 0 0;0 0 0;5 0 0;0 0 0;10 0 0];
% traj_heads2 = [0;0;0;0;0;0;0;0];
% traj_times2 = [0;5;0;5;0;5;0;5];
% 
% traj_posns3 = [0 0 0;0 1 0;0 0 0;0 2 0;0 0 0;0 5 0;0 0 0;0 10 0];
% traj_heads3 = [0;0;0;0;0;0;0;0];
% traj_times3 = [0;5;0;5;0;5;0;5];
% 
% traj_posns4 = [0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0];
% traj_heads4 = [0;pi/3;0;-pi/3;0;pi/6;0;-pi/6];
% traj_times4 = [0;5;0;5;0;5;0;5];
% 
% traj_posns = [traj_posns1;traj_posns2;traj_posns3;traj_posns4];
% traj_heads = [traj_heads1;traj_heads2;traj_heads3;traj_heads4];
% traj_times = [traj_times1;traj_times2;traj_times3;traj_times4];

traj_posns = [0 0 0;0 0 5;0 0 0;5 0 0;0 0 0;0 5 0;0 0 0;5 5 5;0 0 0;5 5 5;0 0 0;0 0 0;0 0 5;0 0 0];
traj_heads = [0;0;0;0;0;0;0;0;0;pi/3;0;pi/3;0;0];
traj_times = repmat([0;10],7,1);

% traj_posns2 = [0 0 0;0 0 pi/
% traj_heads2 = [0;pi/6;pi/3;pi/2;pi;-pi/2;-pi/3;-pi/6];
%     
start_idx = 1;
idx = start_idx;

SettlingTimes = zeros(4,size(traj_times,1)/2);
POvershoots = zeros(4,size(traj_times,1)/2);

for i = 1:2:size(traj_times,1)-1
    [SettlingTimes(:,idx),POvershoots(:,idx)] = StartSimulator(traj_posns(i:i+1,:),traj_heads(i:i+1),traj_times(i:i+1),idx);
    idx = idx + 1;
end