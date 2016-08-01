function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)

accelref = 9.81*outputFLP*wallNormalWorld;

if outputFLP < 0   % reduce into the wall setpoint by half if away from wall
    accelref = [0; 0; 0];%accelref/2;
end

end