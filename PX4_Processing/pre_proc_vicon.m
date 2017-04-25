%% pre process vicon data

% flip vicon so quaternion is smooth (usually vicon solution has w
% component always positive so jumps appear).


%fix vicon orientation data so it doesn't flip back and forth
for ii = 2:length(v_vicon__pose_pose_orientation_x)
    if v_vicon__pose_pose_orientation_x(ii) < -v_vicon__pose_pose_orientation_x(ii-1) +0.05 && v_vicon__pose_pose_orientation_x(ii) > -v_vicon__pose_pose_orientation_x(ii-1) -0.05
       v_vicon__pose_pose_orientation_x(ii) = -v_vicon__pose_pose_orientation_x(ii);
       v_vicon__pose_pose_orientation_y(ii) = -v_vicon__pose_pose_orientation_y(ii);
       v_vicon__pose_pose_orientation_z(ii) = -v_vicon__pose_pose_orientation_z(ii);
       v_vicon__pose_pose_orientation_w(ii) = -v_vicon__pose_pose_orientation_w(ii);
    end
%     if v_vicon__pose_pose_orientation_y(ii) < -v_vicon__pose_pose_orientation_y(ii-1) +0.05 && v_vicon__pose_pose_orientation_y(ii) > -v_vicon__pose_pose_orientation_y(ii-1) -0.05
%        
%     end
end