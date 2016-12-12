function [ initXPosn, initXVel, timeInit, xAcc  ] = getinitworldx( ImpactParams, posnDerivX, IC, givenXAcc )
%getinitworldx.m Find initial X position and velocity required for specific
%initial collision conditions
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Need to run simulation once to find correct xAcc to input
%                into this function, to accurately determine initXPosn and
%                initXVel required.
%-------------------------------------------------------------------------%

global PROP_POSNS BUMP_RADII

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
posnContactX = ImpactParams.wallLoc - (rotMat(:,1)'*(PROP_POSNS(:,initialContactBumper)) + BUMP_RADII(1)*cos(inclination));

xAcc = getinitaccx(IC.attEuler, inclination, givenXAcc); 


initXVel = posnDerivX - xAcc*ImpactParams.timeDes;
initXPosn = posnContactX - (initXVel)*ImpactParams.timeDes - 0.5*xAcc*ImpactParams.timeDes^2;

if initXPosn >= posnContactX
    timeInit = -initXVel / xAcc;
    initXPosn = initXPosn + initXVel*timeInit + 0.5*xAcc*timeInit^2;
    initXVel = 0;    
end

end

function xAcc = getinitaccx(eulerAngles, inclination, givenXAcc)
% Returns initial world X acceleration

if givenXAcc == 0
    roll = eulerAngles(1);
    pitch = eulerAngles(2);
    yaw = eulerAngles(3);
    
    if roll == 0 && pitch == 0
        xAcc = 0;
    else
        if abs(yaw - 0) <= deg2rad(10) || abs(abs(yaw) - pi) <= deg2rad(20)
            fitAngle = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25) deg2rad(30)];% deg2rad(35) deg2rad(45)];
            fitAcc = [0.8575 1.6927 2.5043 3.2638 3.9542 4.7896];% 5.5189 6.4832];
            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (-fit(1)*(pitch)+ fit(2));

        elseif abs(yaw) == pi/2
            fitAngle = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25)];% deg2rad(35) deg2rad(45)];
            fitAcc = [0.8575 1.7330 2.5829 3.3559 4.1361];% 5.5189 6.4832];
            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (fit(1)*abs(roll)+ fit(2));

        elseif abs(yaw) == pi/4 || abs(yaw) == 3*pi/4
            
%             if abs(abs(roll) - abs(pitch)) >= eps*10
%                 error('Quadcopter does not move in strictly X direction');
%             end

%             fitAngle = [0.084825925596688 0.173822851313807 0.262463069971117 0.350546591578811 0.437854666001268 0.524146996691413];
%             fitAcc = [0.831401194164598 1.704934243053496 2.539077514413837 3.380150911758541 4.188051734756724 4.920050077041602];
            fitAngle = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25) deg2rad(30)];
            fitAcc = [0.6186 1.2146 1.7814 2.3070 2.7617 3.1208];
            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (fit(1)*inclination + fit(2));

        else
            error('Need to find Ax');
        end
    end
    
else
    xAcc = givenXAcc;
end
end

