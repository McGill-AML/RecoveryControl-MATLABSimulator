function [Control] = computedesiredacceleration(Control, Twist)
% computedesiredacceleration.m Computes the desired acceleration vector for Recovery Controller

    global g

    if Control.recoveryStage == 0
         % do nothing
    elseif Control.recoveryStage == 1
        Control.acc = [0; 0; g] + Control.accelRef; % point 30 degrees away
    elseif Control.recoveryStage == 2
        Control.acc = [0; 0; g]; % go to hover
    elseif Control.recoveryStage == 3
        Control.acc = [0; 0; g]; % stay at hover
    else
        error('Error computing desired acceleration');
    end
    
end