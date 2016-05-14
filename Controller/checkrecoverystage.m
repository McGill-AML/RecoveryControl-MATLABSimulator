function [recoveryStage] = checkrecoverystage(Pose, Twist)
    % Checks the stage of the recovery controller.
    
    % TODO: include acceleration check to begin
    
    attitudeStable = Pose.attEuler(1) < 0.35 && Pose.attEuler(2) < 0.35 ...
            && Twist.angVel(1) < 0.18  && Twist.angVel(2) < 0.18;
        
    zVelocityStable = Twist.linVel(3) < 0.3;
    
    xyVelocityStable = Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2;
    
    % having stabilized height, check horizontal velocity
    if (attitudeStable && zVelocityStable && xyVelocityStable)
        recoveryStage = 4;
    % having stabilized attitude, check vertical velocity
    elseif (attitudeStable && zVelocityStable)
        recoveryStage = 3;
    % check Euler angles and body rates
    elseif (attitudeStable)
        recoveryStage = 2;
    else
        recoveryStage = 1;
    end
end