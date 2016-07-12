function ImpactIdentification = initimpactidentification()

    ImpactIdentification.timeImpactDetected = 0;
    ImpactIdentification.rotMatPreImpact = zeros(3);      
    ImpactIdentification.inertialAcc = zeros(3,1); %in g's
    ImpactIdentification.wallNormalWorld = zeros(3,1); 

end