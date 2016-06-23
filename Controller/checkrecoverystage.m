function [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage)

% Change to check quaternion error roll pitch elements

attitudeStable = Pose.attEuler(1) < 0.2 && Pose.attEuler(2) < 0.2 ...
            && Twist.angVel(1) < 0.2  && Twist.angVel(2) < 0.2 && Twist.angVel(3) < 0.2;
        
    zVelocityStable = Twist.linVel(3) < 0.1;
    
    if recoveryStage == 1
        if (attitudeStable && zVelocityStable)
            recoveryStage = 3;
        elseif attitudeStable
            recoveryStage = 2;
        end
    elseif recoveryStage == 2
        if zVelocityStable
            recoveryStage = 3;
        end
    end
end