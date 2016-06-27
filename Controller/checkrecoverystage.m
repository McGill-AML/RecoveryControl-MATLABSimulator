function [Control] = checkrecoverystage(Pose, Twist, Control)

    attitudeStable = ...
        Pose.attEuler(1) < 0.2 && Pose.attEuler(2) < 0.2 ...                            % check roll and pitch elements
        && Twist.angVel(1) < 0.2  && Twist.angVel(2) < 0.2 ... % && yawrate ... % check body rates are small enough
        && norm(Control.accelRef) < 0.01;                                               % check if accelRef has gone to zero
        
    zVelocityStable = Twist.linVel(3) < 0.2;
    
    % Stage 1: impact detected, recovery control engaged
    % Stage 2: converged to hover and accelRef = 0
    % Stage 3: vertical velocity stabilized
    
    if Control.recoveryStage == 1
        if (attitudeStable && zVelocityStable)
            Control.recoveryStage = 3;
        elseif attitudeStable
            Control.recoveryStage = 2;
        end
    elseif Control.recoveryStage == 2
        if zVelocityStable
            Control.recoveryStage = 3;
        end
    end
end