function [ initXPosn, initXVel, timeInit  ] = getinitworldx( ImpactParams, posnDerivX, IC, givenXAcc )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

global PROP_POSNS BUMP_RADIUS

yaw = IC.attEuler(3);
rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')');
timeInit = 0;

if yaw >= 0 && yaw < pi/2
    initialContactBumper = 4;
elseif yaw >= pi/2 && yaw <= pi
    initialContactBumper = 3;
elseif yaw < 0 && yaw(1) >= -pi/2
    initialContactBumper = 1;
elseif yaw < -pi/2 && yaw >= -pi
    initialContactBumper = 2;
else
    error('Choose another initial heading');
end

inclination = getinclination(rotMat,yaw);
posnContactX = ImpactParams.wallLoc - (rotMat(:,1)'*(PROP_POSNS(:,initialContactBumper)) + BUMP_RADIUS*cos(inclination));

xAcc = getinitaccx(IC.attEuler, inclination, givenXAcc); 


initXVel = posnDerivX - xAcc*ImpactParams.timeDes;
initXPosn = posnContactX - (initXVel)*ImpactParams.timeDes - 0.5*xAcc*ImpactParams.timeDes^2;

if initXPosn >= posnContactX
    timeInit = -initXVel / xAcc;
    initXPosn = initXPosn + initXVel*timeInit + 0.5*xAcc*timeInit^2;
    initXVel = 0;    
end

end

