function [Control] = computedesiredacceleration(Control, Twist)

    % Computes the desired acceleration vector. 
    global g dZ 

    % TODO: why would c be set to gravity? I thought it was zero
    % TODO: sort out negatives positives
    switch Control.recoveryStage
        case 0
            % pre-impact control
            dZ = 0;
        case 1
            % Initialize attitude control.
            dZ = 0;
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
    Control.acc = [0; 0; -dZ*Twist.posnDeriv(3)] + [0; 0; g] + Control.accelRef; 
end