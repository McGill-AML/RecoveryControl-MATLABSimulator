function [state, stateDeriv] = initstate(IC)

    % Initialize full state of quadrotor 
    % Index map:
    % [1 2 3 4 5 6 7 8 9 10 11 12 13]
    % [u v w p q r x y z q0 q1 q2 q3]
    state = zeros(13,1);
    % The derivative of the state
    stateDeriv = zeros(13,1);
    
    % compute quaternion (q0 q1 q2 q3)
%     roll = IC.attEuler(1);
%     pitch = IC.attEuler(2);
%     yaw = IC.attEuler(3);
%     state(10:13) = angle2quat(roll, pitch, yaw);
    
    
    state(1:3) = IC.linVel; % u v w
    state(4:6) = IC.angVel;   % p q r
    state(7:9) = IC.posn;   % x y z
    state(10:13) = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
end