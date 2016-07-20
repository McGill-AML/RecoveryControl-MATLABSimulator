function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)
global g

if outputFLP <= 0 %flipping away from wall
    if abs(outputFLP) < 0.3 %relatively level
        accelrefmagnitude = 0;
    else
        accelrefmagnitude = 3*outputFLP-0.3; %TODO:scale 3        
    end
else %flipping towards wall
    if abs(outputFLP) < 0.3
        accelrefmagnitude = 2.5; %so Navi will stabilize away from wall, not next to it
    else
        accelrefmagnitude = 9.81*outputFLP-0.3;
    end
end

accelref = accelrefmagnitude*wallNormalWorld;

end