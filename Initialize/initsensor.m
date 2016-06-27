function Sensor = initsensor(rotMat, stateDeriv, Twist)

global g

Sensor.accelerometer = (rotMat*[0;0;g] + stateDeriv(1:3) + cross(Twist.angVel,Twist.linVel))/g; %in g's
Sensor.gyro = Twist.angVel;

end