function [Control] = computedesiredacceleration(Control, Twist)
%computedesiredacceleration.m Computes the desired acceleration vector for Recovery Controller
%   Author: Gareth Dicker (gareth.dicker@mail.mcgill.ca)
%   Last Updated: 
%   Description: 
%-------------------------------------------------------------------------%

    global g
    % Introduce vertical velocity control gain at recovery stage 3
    switch Control.recoveryStage
        case 0 % Normal Flight
            dZ = 0;
        case 1 % Stabilize Attitude to "away" orientation with zero yaw rate
            dZ = 0;
        case 2 %Stabilize Attitude to "hover" orientation
            Control.accelRef = [0; 0; 0];
            dZ = 0;
            %dZ = 5;
        case 3 %Stabilize Height
            dZ = 5;
            Control.accelRef = [0; 0; 0];
        case 4 %Stabilize Horizontal Position
            dZ = 5;
            Control.accelRef = [0; 0; 0];
        otherwise 
            error('Invalid recovery stage!');    
    end
    % Desired acceleration is the sum of a 1) gravity, 2) reference acceleration
    % and 3) vertical velocity control term
    Control.acc = [0; 0; g] + Control.accelRef + [0; 0; -dZ*Twist.posnDeriv(3)]; 
end