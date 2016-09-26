function ImpactIdentification = initimpactidentification()

    ImpactIdentification.timeImpactDetected = 0;
    ImpactIdentification.rotMatPreImpact = zeros(3);      
    ImpactIdentification.inertialAcc = zeros(3,1); %in g's
    ImpactIdentification.wallNormalWorld = zeros(3,1); 
    ImpactIdentification.wallNormalWorldFromCM = zeros(3,1);
    ImpactIdentification.wallNormalWorldCorrected = zeros(3,1);
    ImpactIdentification.wallNormalWorldCorrected2 = zeros(3,1);

end