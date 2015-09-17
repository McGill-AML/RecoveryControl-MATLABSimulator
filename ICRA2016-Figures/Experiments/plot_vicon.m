hold on
subplot(2,1,1);
plot(v_vicon__pose___time, v_vicon__pose_pose_orientation_w)
hold on;
plot(v_vicon__pose___time, v_vicon__pose_pose_orientation_x)
plot(v_vicon__pose___time, v_vicon__pose_pose_orientation_y)
plot(v_vicon__pose___time, v_vicon__pose_pose_orientation_z)


% subplot(2,1,2);
% for k = 1: size(vvicon__pose_pose_orientation_w,1)
%     q = [vvicon__pose_pose_orientation_y(k,1);vvicon__pose_pose_orientation_x(k,1);vvicon__pose_pose_orientation_w(k,1);vvicon__pose_pose_orientation_z(k,1)];
%     [roll(k),pitch(k),yaw(k)] = quat2angle(q);
% end
% plot(vvicon__pose___time,roll,vvicon__pose___time,pitch,vvicon__pose___time,yaw);
% 
%
subplot(2,1,2);
plot(v_vicon__pose___time, v_vicon__pose_pose_position_x)
hold on;
plot(v_vicon__pose___time, v_vicon__pose_pose_position_y)
plot(v_vicon__pose___time, v_vicon__pose_pose_position_z)