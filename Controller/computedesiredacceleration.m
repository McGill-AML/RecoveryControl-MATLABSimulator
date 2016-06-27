function [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage, accelref)

    % Computes the desired acceleration vector. 
    global g pZ pXY dZ dXY;
 
    % TODO: why would c be set to gravity? I thought it was zero
    % TODO: sort out negatives positives
    switch recoveryStage
        case 1
            % Initialize attitude control.
        case 2
            % Set vertical velocity gain.
            dZ = 5; 
        case 3
            % no change if vertical velocity has converged
            dZ = 5; 
        otherwise 
            error('Invalid value for recoveryStage');
    end
    
    % Compute desired acceleration as combination of position and velocity
    % controls plus a gravity term
%     
%     Control.acc = [pXY    0    0; ...
%                     0   pXY    0; ...
%                     0     0   pZ] * ([1; 1; 1] - Pose.posn) ...
%                 + [dXY    0    0; ...
%                     0   dXY    0; ...
%                     0     0   dZ] * ([0; 0; 0] - Twist.posnDeriv) ...
%                 + [0; 0; g];

    Control.acc = accelref + [0;0;g];

end