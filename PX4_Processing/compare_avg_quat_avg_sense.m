%compare averaging quaternions to averaging measurements.
mag_i = [ 0.3; 0; 0.9];
mag_i = mag_i/norm(mag_i);

grav = [0; 0; -9.81];

quater = [0.7; 0.1; 0.3; 0.7];
quater = quater/norm(quater);

rotMat = quat2rotmat(quater);

mag_b = rotMat'*mag_i;
grav_b = rotMat'*grav;
num_of_pts_2_use = 80;

for ii = 1:num_of_pts_2_use
    IMU_Mag(:,ii) = mag_b + 0.05*randn(3,1);
    IMU_Acc(:,ii) = grav_b + 0.5*randn(3,1);
end

mag_decl = 0;
Accel = IMU_Acc;
Mag = IMU_Mag;


for ii = 1:num_of_pts_2_use
    [q_NED, rotMatNED] = initOrientExpData(Accel(:,ii), Mag(:,ii), mag_decl);
%     debug2 = quatinv(quatmultiply(  Vicon(:,ii), quatinv(q_NED)));
    delta_q(:,ii) = q_NED;
%     if ii == 10
%         debug2
%         delta_q(:,ii)
%     end
end



M = zeros(4);
for ii = 1:num_of_pts_2_use
    M = M + delta_q(:,ii)*delta_q(:,ii)';
end

K = 4*M-num_of_pts_2_use*eye(4);

[eigVecs, eigVals] = eig(K);

q_avg = eigVecs(:,4)/norm(eigVecs(:,4));


% average input of points -- q attitude might only use first timestep?
IMU_Acc_0 = mean(IMU_Acc,2);
IMU_Mag_0 = mean(IMU_Mag,2);

k_hat = -IMU_Acc_0/norm(IMU_Acc_0); % defines z axis as down

i_hat = (IMU_Mag_0 - (IMU_Mag_0'*k_hat)*k_hat); %define north as mag minus down
i_hat = i_hat/norm(i_hat);

j_hat = cross_mat(k_hat)*i_hat; %orthonormal last component

rotMat_init = [i_hat';
               j_hat';
               k_hat'];
           
       
q_sens = rotmat2quat(rotMat_init);

q_e_avg = quatmultiply(quater, quatinv(q_avg));
q_e_sens = quatmultiply(quater, quatinv(q_sens));

avg_ang_err = 2*acos(q_e_avg(1));
avg_sens_err = 2*acos(q_e_sens(1));


