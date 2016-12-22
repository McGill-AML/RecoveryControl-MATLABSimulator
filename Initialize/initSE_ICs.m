function [Est_ICs] = initSE_ICs(IC,sensParams, loop_no)

Est_ICs.posn = IC.posn;
Est_ICs.linVel = IC.linVel;

if loop_no <= 40
    IC.attEuler = IC.attEuler;% + randn(3,1).*[2; 2; 5]*pi/180;
else   
    IC.attEuler = IC.attEuler + randn(3,1).*[2; 2; 5]*pi/180;
end

Est_ICs.q = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
Est_ICs.omega = [0;0;0];

if loop_no <= 40 
    Est_ICs.bias_acc =   sensParams.bias.acc; % + randn(3,1)*0.01;
    Est_ICs.bias_gyr =   sensParams.bias.gyr; % + randn(3,1)*0.01; 
else
    Est_ICs.bias_acc =    [0;0;0]; % sensParams.bias.acc; % + randn(3,1)*0.01;
    Est_ICs.bias_gyr =  [0;0;0]; % sensParams.bias.gyr; % + randn(3,1)*0.01; 
end

% initial covariance q, gyr_bias
Est_ICs.P_init_att = diag([0.1,0.1,0.1,0.1, 0.0001,0.0001, 0.0001]);


% initial covariance pos, vel, acc_bias
Est_ICs.P_init_pos = diag([1,1,1, 0.1,0.1,0.1, 0.0001, 0.0001, 0.0001]);