function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)

global g

accelref = g*outputFLP*wallNormalWorld;

if outputFLP < 0   % reduce into the wall setpoint by half if away from wall
    accelref = accelref/2;
end

end