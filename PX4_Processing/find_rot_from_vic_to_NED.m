%% calc rotation matrix from standstill data
% 
% figure;plot(fieldposetime, [fieldposeorientationw,fieldposeorientationx,fieldposeorientationy,fieldposeorientationz]);grid on;
% title('vicon Orientation');
% %plot EKF orientation
% figure;plot(TIME, [ATT_qw, ATT_qx, ATT_qy, ATT_qz]); grid on;
% title('Pixhakwk EKF Orientation');
% 
% figure;plot([ATT_Roll, ATT_Pitch, ATT_Yaw]);grid on


%two match points 21671 - drift starts, 84584 - change orientation
% time_end = 15100;

%for second section go 11096 to end
% 
% VISN_QuatW = interp1(fieldposetime,fieldposeorientationw,TIME(1:time_end));
% VISN_QuatX = interp1(fieldposetime,fieldposeorientationx,TIME(1:time_end));
% VISN_QuatY = interp1(fieldposetime,fieldposeorientationy,TIME(1:time_end));
% VISN_QuatZ = interp1(fieldposetime,fieldposeorientationz,TIME(1:time_end));

%renormalize incase interp messed up unit norm
% for ii = 1:length(VISN_QuatW)
%     q_e = [VISN_QuatW(ii), VISN_QuatX(ii), VISN_QuatY(ii), VISN_QuatZ(ii)];
%     q_e = q_e/norm(q_e,2);
%     VISN_QuatW(ii) = q_e(1);
%     VISN_QuatX(ii) = q_e(2);
%     VISN_QuatY(ii) = q_e(3);
%     VISN_QuatZ(ii) = q_e(4);
% end

%%least squares to find most likely rotation for first 100 data points?
% ls_length = 100;
% Q_E = zeros(ls_length*4,4);
% q_v = zeros(ls_length*4,1);
% for ii = 3:ls_length
%     q_v((ii-1)*4+1:ii*4) = [VISN_QuatW(ii); VISN_QuatX(ii); VISN_QuatY(ii); VISN_QuatZ(ii)];
%     q_e = [ATT_qw(ii); ATT_qx(ii); ATT_qy(ii); ATT_qz(ii)];
%     Q_E((ii-1)*4+1:ii*4,:) = [q_e(1) -q_e(2) -q_e(3) -q_e(4);q_e(2) q_e(1) -q_e(4) q_e(3);q_e(3) q_e(4) q_e(1) -q_e(2);q_e(4) -q_e(3) q_e(2) q_e(1)];
% end

% ls_weight = ones(ls_length*4,1);
% ls_weight(4:4:end) = 1/(ls_weight(4:4:end)+q_v(4:4:end).^2*1000);
% ls_weight(3:4:end) = 1/(ls_weight(3:4:end)+q_v(3:4:end).^2*1000);
% rot_quat = lscov(Q_E,q_v,ls_weight);
rot_quat = Q_E\q_v;
% rot_quat = quat_mult([ATTqw(init_EKF_ind+1502-1), -ATTqx(init_EKF_ind+1502-1), -ATTqy(init_EKF_ind+1502-1), -ATTqz(init_EKF_ind+1502-1)],[VISN_QuatW(1502), VISN_QuatX(1502), VISN_QuatY(1502), VISN_QuatZ(1502)])



[r1, r2, r3] = quat2angle(rot_quat); %rotation quat to eul, for easier picture
[r1, r2, r3]*180/pi

for ii = 1:length(ATT_qw)
    q_e = [ATT_qw(ii); ATT_qx(ii); ATT_qy(ii); ATT_qz(ii)];
    q_e_upd = quat_mult([0;1;0;0],q_e);
    ATT_qw_i(ii) = q_e_upd(1);
    ATT_qx_i(ii) = q_e_upd(2);
    ATT_qy_i(ii) = q_e_upd(3);
    ATT_qz_i(ii) = q_e_upd(4);
end


for ii = 1:length(ATT_qw)
    q_e = [ATT_qw_i(ii); ATT_qx_i(ii); ATT_qy_i(ii); ATT_qz_i(ii)];
    q_e_upd = quat_mult(q_e,rot_quat);
    ATT_qw_i(ii) = q_e_upd(1);
    ATT_qx_i(ii) = q_e_upd(2);
    ATT_qy_i(ii) = q_e_upd(3);
    ATT_qz_i(ii) = q_e_upd(4);
end


figure;plot(TIME(1:time_end), [VISN_QuatW, VISN_QuatX, VISN_QuatY, VISN_QuatZ]);grid on; title('vicon');
hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);

figure;plot(TIME(1:time_end), [ATT_qw(1:time_end), ATT_qx(1:time_end), ATT_qy(1:time_end), ATT_qz(1:time_end)]);grid on;
title('EKF');hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);

figure;plot(TIME(1:end), [ATT_qw_i(1:end)', ATT_qx_i(1:end)', ATT_qy_i(1:end)', ATT_qz_i(1:end)']);grid on;
title('EKF');hdt = datacursormode;
set(hdt,'UpdateFcn',@datatipWithSubscript);

count = 1;
for ii = 1:length(VISN_QuatW)
    if isnan(VISN_QuatW(ii))
        findisnan(count) = ii;
        count = count+1;
    end
end