function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)

% accelref = 9.81*outputFLP*wallNormalWorld;

% if outputFLP < 0   % reduce into the wall setpoint by half if away from wall
%     accelref = accelref/2;
% end

accelref = 9.81*wallNormalWorld;


end