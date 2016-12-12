function [Pose, Twist] = initposetwist
%initposetwist.m Initialize Pose and Twist structs
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description:
%-------------------------------------------------------------------------%   

    % Position in world frame
    Pose.posn = zeros(3,1);

    % Attitude in quaternion form
    Pose.attQuat = zeros(4,1);
    
    % Attitude in Euler angles
    Pose.attEuler = zeros(3,1); 
    
    % Linear velocity in body frame
    Twist.linVel = zeros(3,1);
    
    % Linear acceleration in world frame
    Twist.worldAcc = zeros(3,1);
    
    % Linear velocity in the world frame
    Twist.posnDeriv = zeros(3,1);

    % Angular velocities in body frame
    Twist.angVel = zeros(3,1);

    % Euler angle rates
    Twist.attEulerRate = zeros(3,1);

end
