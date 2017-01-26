function [Est_ICs] = initSE_ICs(IC,sensParams, loop_no, useExpData)

Est_ICs.posn = IC.posn;
Est_ICs.linVel = IC.linVel;

if loop_no <= 0 || useExpData == 1
    IC.attEuler = IC.attEuler;% + randn(3,1).*[2; 2; 5]*pi/180;
else
    phi = randn(1);
    a_tmp = -1 + 2*rand(3,1);
    a_tmp = a_tmp/norm(a_tmp);
    quat_tmp = [cos(phi*pi/180/2); a_tmp*sin(phi*pi/180/2)];
    [r1, r2, r3] = quat2angle(quat_tmp);
    IC.attEuler = IC.attEuler + [r1; r2; r3];
%     IC.attEuler = IC.attEuler + randn(3,1).*[2; 2; 5]*pi/180;
end

if useExpData
    Est_ICs.q = angle2quat((IC.attEuler(1)),IC.attEuler(2),IC.attEuler(3),'xyz')';
    Est_ICs.q = [-0.6786;   -0.0116;    0.0243;   -0.7340];
else
    Est_ICs.q = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
end
Est_ICs.omega = [0;0;0];

if loop_no <= 0 && useExpData == 0
    Est_ICs.bias_acc =   sensParams.bias.acc; % + randn(3,1)*0.01;
    Est_ICs.bias_gyr =   sensParams.bias.gyr; % + randn(3,1)*0.01; 
else
    Est_ICs.bias_acc =    [0;0;0]; % sensParams.bias.acc; % + randn(3,1)*0.01;
    Est_ICs.bias_gyr =  [0;0;0]; % sensParams.bias.gyr; % + randn(3,1)*0.01; 
end

% initial covariance q, gyr_bias
Est_ICs.P_init_att = diag([0.25,0.25,0.25,0.25, 0.0001,0.0001, 0.0001]);


% initial covariance pos, vel, acc_bias
Est_ICs.P_init_pos = diag([1,1,1, 0.1,0.1,0.1, 0.0001, 0.0001, 0.0001]);