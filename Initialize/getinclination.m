function [ inclination] = getinclination( rotMat, heading )
%getinclination.m Returns quadrotor inclination to wall spanning YZ plane
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%
wallNormalWorld = [-1;0;0];
wallTangentWorld = cross([0;0;1],wallNormalWorld);

negBodyZ = [0;0;-1]; %[0;0;1] for NWU, %[0;0;-1] for NED [IMU mounting]
bodyZProjection = rotMat'*negBodyZ - dot((rotMat'*negBodyZ),wallTangentWorld)*wallTangentWorld;
dotProductWithWorldZ = dot(bodyZProjection,[0;0;1]); %[0;0;1] for NWU, %[0;0;-1] for NED [World Frame]
inclinationAngle = acos(dotProductWithWorldZ/norm(bodyZProjection));

dotProductWithWorldNormal = dot(bodyZProjection,wallNormalWorld);
angleWithWorldNormal = acos(dotProductWithWorldNormal/(norm(bodyZProjection)*norm(wallNormalWorld)));  

inclinationSign = sign(angleWithWorldNormal - pi/2);                  
inclination = inclinationSign*(inclinationAngle);

end

