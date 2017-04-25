function [Pose, Twist] = updatekinematics(state, stateDeriv)
    % Initialize structs Twist Pose and Control.

    % Position in world frame
    Pose.posn = state(7:9);

    % Attitude in quaternion form
    Pose.attQuat = state(10:13);
    
    % Attitude in Euler angles
    [yaw, pitch, roll] = quat2angle(Pose.attQuat, 'zyx');
    Pose.attEuler = [yaw; pitch; roll];    
    
    % Linear velocity in body frame
    Twist.linVel = state(1:3);
    
    % Linear velocity in the WORLD frame
    Twist.posnDeriv = stateDeriv(7:9);
    
    % Linear acceleration in the world frame
    Twist.worldAcc = quat2rotmat(Pose.attQuat)'*stateDeriv(1:3);

    % Angular velocities in body frame
    Twist.angVel = state(4:6);

    % Euler angle rates (depends on Euler angles and body rates)
    Twist.attEulerRate =    [                      Twist.angVel(1) + ...
                              sin(roll)*tan(pitch)*Twist.angVel(2) + ...
                              cos(roll)*tan(pitch)*Twist.angVel(3);
                              
                                         cos(roll)*Twist.angVel(2) - ...
                                         sin(roll)*Twist.angVel(3);
                              
                              sin(roll)*sec(pitch)*Twist.angVel(2) + ...
                              cos(roll)*sec(pitch)*Twist.angVel(3)];

end