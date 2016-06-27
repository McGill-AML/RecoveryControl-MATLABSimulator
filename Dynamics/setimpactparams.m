function [kContact, eContact, nContact, wallLoc, wallPlane, velocitySliding] = setimpactparams(ImpactParams)
%sets contact model parameters

% Hunt and Crossley non-linear compliant contact model for normal force
kContact = ImpactParams.compliantModel.k;    
eContact = ImpactParams.compliantModel.e;
nContact = ImpactParams.compliantModel.n;

% Simulation wall location and spanning plane
wallLoc = ImpactParams.wallLoc;
wallPlane = ImpactParams.wallPlane;

% Sliding velocity for friction model
velocitySliding = ImpactParams.frictionModel.velocitySliding;

end