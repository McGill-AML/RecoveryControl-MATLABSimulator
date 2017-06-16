function [Control, recoverySuccessful] = checkrecoverystage(Pose, Twist, Control, ImpactInfo, recoverySuccessful)
%checkrecoverystage.m  Checks recovery stage for Recovery Controller

    bodyRatesThresh = 2.0; %rad/s
    errorQuaternionThresh = 0.15; % non-dimensioned
    rollPitchThresh = 0.2; % rad
    verticalVelocityThresh = 0.2; % m/s
    
    SWITCH_1 = abs(Control.errQuat(2)) < errorQuaternionThresh && abs(Control.errQuat(3)) < errorQuaternionThresh ... % error quaternion elements 2 and 3
        && abs(Twist.attEulerRate(1)) < bodyRatesThresh  && abs(Twist.attEulerRate(2)) < bodyRatesThresh;    % roll/pitch rates   
    
    SWITCH_2 = abs(Pose.attEuler(1)) < rollPitchThresh && abs(Pose.attEuler(2)) < rollPitchThresh && ...
                  abs(Twist.attEulerRate(1)) < bodyRatesThresh  && abs(Twist.attEulerRate(2)) < bodyRatesThresh ...
                  && Twist.linVel(3) < verticalVelocityThresh;
              
   
    if Control.recoveryStage == 1
        if SWITCH_1
            Control.recoveryStage = 2;
        end
    elseif Control.recoveryStage == 2
        if SWITCH_2 %recovery is done
            Control.recoveryStage = 3;
            recoverySuccessful = 1;
        end
    elseif Control.recoveryStage == 3
        Control.recoveryStage = 3;
    else
        error('Recovery stage switching error');
    end

end