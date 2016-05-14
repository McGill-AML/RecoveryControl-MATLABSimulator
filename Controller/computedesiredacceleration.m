function [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage)

    % Computes the desired acceleration vector. 
%     global g pZ pXY dZ dXY;
    global g
 
    recoveryPosn = [0;0;2];
    recoveryPosnDeriv = [0; 0; 0];
    accRef = [0;0;0];
    accDesMax = 1.86*9.81; %1.86g
    
    % TODO: why would c be set to gravity? I thought it was zero
    % TODO: sort out negatives positives
    pXY = 0;
    pZ = 0;
    dXY = 0;
    dZ = 0;    
    
    if recoveryStage > 2
        recoveryStage = 2;
    end    
    
    if recoveryStage >= 2
        dZ = 5;
    end
    
    if recoveryStage >= 3
        pZ = 5;
        dXY = 3;
    end
    
    if recoveryStage >=4
        pXY = 3;     
    end
      
    Control.acc = [pXY    0    0; ...
                    0   pXY    0; ...
                    0     0   pZ] * (recoveryPosn - Pose.posn) ...
                + [dXY    0    0; ...
                    0   dXY    0; ...
                    0     0   dZ] * (recoveryPosnDeriv - Twist.posnDeriv) ...
                + accRef ...
                - [0; 0; -g] ;
    if norm(Control.acc) > aDesMax
        warning('a_des is greater than bound');        
    end

end