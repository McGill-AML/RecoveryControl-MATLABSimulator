function [Control] = checkrecoverystage(Pose, Twist, Control, ImpactInfo)
%checkrecoverystage.m  Checks recovery stage for Recovery Controller
%   Author: Gareth Dicker (gareth.dicker@mail.mcgill.ca)
%   Last Updated: 
%   Description: 
%-------------------------------------------------------------------------%

%%%%% CONDITIONS %%%%%

    % Stage 2 condition
%     maneuverStable = abs(Control.errQuat(2)) < 0.3 && abs(Control.errQuat(3)) < 0.3 ... % error quaternion elements 2 and 3
%         && abs(Twist.angVel(1)) < 0.5  && abs(Twist.angVel(2)) < 0.5;    % roll/pitch rates   
    maneuverStable = abs(Control.errQuat(2)) < 0.17 && abs(Control.errQuat(3)) < 0.17 ... % error quaternion elements 2 and 3
        && abs(Twist.attEulerRate(1)) < 1  && abs(Twist.attEulerRate(2)) < 1;    % roll/pitch rates   
    
    % Stage 3 condition
    hoverStable = abs(Pose.attEuler(1)) < 0.2 && abs(Pose.attEuler(2)) < 0.2 && ...
                  abs(Twist.attEulerRate(1)) < 1  && abs(Twist.attEulerRate(2)) < 1;
    
    % Stage 4 condition - 0.2 m/s is arbitrary 
    zVelocityStable = Twist.linVel(3) < 0.3;
    
    % Stage 0: pre-impact
    % Stage 1: impact detected
    % Stage 2: converged to accelRef
    % Stage 3: converged to hover after accelRef is put to zero
    % Stage 4: stabilized vertical velocity
    
%%%%% CHECK LOGIC %%%%%
    switch Control.recoveryStage
        case 0
            if ImpactInfo.firstImpactDetected
                Control.recoveryStage = 1;
            end
        case 1
            if maneuverStable
                Control.recoveryStage = 2;
            end
        case 2
            if hoverStable
                Control.recoveryStage = 3;
            end
        case 3
            if zVelocityStable
                Control.recoveryStage = 4;
            end
        case 4
            return
        otherwise 
            error('Invalid recovery stage!');
    end  
end