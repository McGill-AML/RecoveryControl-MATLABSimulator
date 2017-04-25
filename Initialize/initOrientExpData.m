function [q_init, rotMat_init] = initOrientExpData(IMU_Acc, IMU_Mag, mag_decl)
% init script for inertial frame
% inputs should be 3 x 'However many data points you want to use' vectors of the measurement
% data

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
           
mag_decl_yaw = [cos(mag_decl*pi/180/2); 0; 0; sin(mag_decl*pi/180/2)]; %rotate by mag decl

           
q_init = rotmat2quat(rotMat_init);

q_init = quatmultiply(mag_decl_yaw, q_init);

% rotMat_init = quat2rotmat(q_init);