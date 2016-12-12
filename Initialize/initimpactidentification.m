function ImpactIdentification = initimpactidentification()
%initimpactidentification.m Initialize ImpactIdentification struct
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------% 
    ImpactIdentification.timeImpactDetected = 0;
    ImpactIdentification.rotMatPreImpact = zeros(3);      
    ImpactIdentification.inertialAcc = zeros(3,1); %in g's
    ImpactIdentification.wallNormalWorld = zeros(3,1); 
    ImpactIdentification.wallNormalWorldFromCM = zeros(3,1);
    ImpactIdentification.wallNormalWorldCorrected = zeros(3,1);
    ImpactIdentification.wallNormalWorldCorrected2 = zeros(3,1);

end