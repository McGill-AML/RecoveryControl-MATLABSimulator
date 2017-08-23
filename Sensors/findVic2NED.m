%% rotate from Vicon to NED using accel/mag measurements
function [vic2NED, q_v2NED] = findVic2NED(Vicon, Mag, Accel, mag_decl, num_of_pts_2_use)

delta_q = zeros(4,num_of_pts_2_use);
%put vicon into right frame (z = down), rotate about x axis by 180
for ii = 1:length(Vicon)
%     debug1 = quatmultiply(Vicon(:,ii),[0;1;0;0]);
    RViVb = quat2rotmat(Vicon(:,ii));
    RVbQb = quat2rotmat([0;1;0;0]);
    Vicon(:,ii) = rotmat2quat((RVbQb'*RViVb')');
%     if ii == 600 
%        debug1
%        Vicon(:,ii)
%     end
end

% find error quaternions between vicon and NED frame as defined by sensors
% at a given timestep
for ii = 1:num_of_pts_2_use
    [q_NED, rotMatNED] = initOrientExpData(Accel(:,ii), Mag(:,ii), mag_decl);
%     debug2 = quatinv(quatmultiply(  Vicon(:,ii), quatinv(q_NED)));
    rotMatV = quat2rotmat(Vicon(:,ii));
    delta_q(:,ii) = rotmat2quat(rotMatNED*rotMatV');
%     if ii == 10
%         debug2
%         delta_q(:,ii)
%     end
end


%average the error quaternions using davenport-q method (alternatively could use
%SVD but this is quicker), from Markley's paper on averaging quaternions

M = zeros(4);
for ii = 1:num_of_pts_2_use
    M = M + delta_q(:,ii)*delta_q(:,ii)';
end

K = 4*M-num_of_pts_2_use*eye(4);

[eigVecs, eigVals] = eig(K);

q_v2NED = eigVecs(:,4)/norm(eigVecs(:,4));

vic2NED = quat2rotmat(q_v2NED);