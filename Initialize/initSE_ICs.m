function [Est_ICs] = initSE_ICs(IC,sensParams, loop_no, useExpData)

Est_ICs.posn = IC.posn;
Est_ICs.linVel = IC.linVel;

if loop_no <= 0 || useExpData == 1
    IC.attEuler = IC.attEuler;% + randn(3,1).*[2; 2; 5]*pi/180;
else
    phi = randn(1);
%     phi = [-1.3118, -.0515, -.4289, .4676, 2.2786]; % set of randomized init conditions that are constant
%     phi = phi(mod(loop_no-1,5)+1);
    a_tmp = -1 + 2*rand(3,1);
    a_tmp = a_tmp/norm(a_tmp);
%     a_tmp = [-0.7946    0.2722   -0.6521   -0.4342    0.0362;
%              0.6004    0.7482    0.7337    0.8628    0.8265;
%              0.0902    0.6051   -0.1908    0.2588    0.5617]; % set of randomized init conditions that are constant
%     a_tmp = a_tmp(:,mod(loop_no-1,5)+1);
    
    quat_tmp = [cos(phi*pi/180/2); a_tmp*sin(phi*pi/180/2)];
    [r1, r2, r3] = quat2angle(quat_tmp);
    IC.attEuler = IC.attEuler + [r1; r2; r3];
%     IC.attEuler = IC.attEuler + randn(3,1).*[2; 2; 5]*pi/180;
end

if useExpData
    Est_ICs.q = angle2quat((IC.attEuler(1)),IC.attEuler(2),IC.attEuler(3),'xyz')';
%     [Est_ICs.q, rotMat]  = initOrientExpData([IMU_AccX'; IMU_AccY'; IMU_AccZ'], [IMU_GyroX'; IMU_GyroY';IMU_GyroZ']);
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