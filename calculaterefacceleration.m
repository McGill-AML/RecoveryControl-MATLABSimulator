function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)

if outputFLP >= 0 %flipping away from wall
    if outputFLP < 0.3 %relatively level
        accelrefmagnitude = 0;
    else
        accelrefmagnitude = 0.7*outputFLP/0.7;
    end
else %flipping towards wall
    accelrefmagnitude = 4*outputFLP/0.7;
end

accelref = -accelrefmagnitude*wallNormalWorld;
% accelref = 2*wallNormalWorld;
end