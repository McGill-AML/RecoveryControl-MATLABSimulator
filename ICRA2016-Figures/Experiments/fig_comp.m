close all;
figure()
plot(v_vicon__pose___time, v_vicon__pose_pose_position_x,'r')
hold on;
plot(v_vicon__pose___time, v_vicon__pose_pose_position_y,'b')
plot(v_vicon__pose___time, v_vicon__pose_pose_position_z,'g')

figure()
plot(v_accel___time,v_accel_x,'r')
hold on;
plot(v_accel___time,v_accel_y,'b')

plot(v_accel___time,v_accel_z,'g')
