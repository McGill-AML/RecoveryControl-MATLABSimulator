function [  ] = printCollisionInfo( Plot, timeImpact, timeImpactDetected )
%printCollisionInfo.m Displays stats about collision
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

% Calculate inclination @ impact:
attQuatAtImpact = Plot.quaternions(:,vlookup(Plot.times,timeImpact));
rotMatAtImpact = quat2rotmat(attQuatAtImpact);
headingAtImpact = Plot.eulerAngles(3,vlookup(Plot.times,timeImpact));
inclinationAtImpact = rad2deg(getinclination(rotMatAtImpact,headingAtImpact));


display('Crash Information');
display('------------------------');
display(['   Init. Impact Time:', blanks(14), num2str(timeImpact), '[s]']);
display(['   Impact Detection Time:', blanks(10), num2str(timeImpactDetected), '[s]']);
display(['   X-Velocity at timeImpact:', blanks(7), num2str(Plot.posnDerivs(1,vlookup(Plot.times,timeImpact))), ' [m/s]']);
display(['   Inclination at timeImpact:', blanks(6), num2str(inclinationAtImpact), '[deg]']);
display(['   Height Lost:', blanks(20), num2str(Plot.posns(3,end)-Plot.posns(3,1)), '[m]']);

end

