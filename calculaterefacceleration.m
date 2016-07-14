function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)
global g

if outputFLP >= 0 %flipping away from wall
    if outputFLP < 0.3 %relatively level
        accelrefmagnitude = 0;
    else
        accelrefmagnitude = -3*outputFLP; %-7*outputFLP;
%         accelrefmagnitude = -3*outputFLP^2;%6.5*outputFLP;
        
    end
else %flipping towards wall
%     accelrefmagnitude = -1.5*g;
    if abs(outputFLP) < 0.3
        accelrefmagnitude = -2.5;
    else
%         accelrefmagnitude = 3*outputFLP;%6.5*outputFLP;
        accelrefmagnitude = -2*outputFLP^2 - 2.5;%6.5*outputFLP;
    end
end

accelref = -accelrefmagnitude*wallNormalWorld;

end