function [Contact, ImpactInfo] = initcontactstructs
  
    Contact = initcontact(0);
    
    ImpactInfo.isStable = 1;
    ImpactInfo.firstImpactOccured = 0;
    
    %Init bumperInfo
    bumperInfo.timeImpacts = [];
    bumperInfo.maxDefls = [];
    bumperInfo.maxNormalForces = [];
    bumperInfo.initialNormalVels = [];
    bumperInfo.inclinations = [];
    bumperInfo.numContacts = [];
    bumperInfo.impactDurations = [];

    ImpactInfo.bumperInfos = repmat(bumperInfo,4,1);
    
    ImpactInfo.firstImpactDetected = 0;
%     ImpactInfo.accelRefCalculated = 0;
end
