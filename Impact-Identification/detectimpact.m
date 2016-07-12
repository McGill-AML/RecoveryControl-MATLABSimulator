function [ImpactInfo, ImpactIdentification] = detectimpact(iSim, ImpactInfo,ImpactIdentification, Sensor,Histposes,PREIMPACT_ATT_CALCSTEPFWD)
%DETECTIMPACT Summary of this function goes here
%   Detailed explanation goes here

    if ImpactInfo.firstImpactDetected == 0
        if norm(Sensor.accelerometer(1:2))>= 0.5 %Accelerometer horizontal magnitude
%         if norm(estForceExternalBody) >= 1 %External Force estimation
            ImpactInfo.firstImpactDetected = 1;
            ImpactIdentification.timeImpactDetected = iSim;
            
            %pre-impact attitude
            ImpactIdentification.rotMatPreImpact = quat2rotmat(Histposes(end-PREIMPACT_ATT_CALCSTEPFWD).attQuat);      
                        
            %wall location, i.e. e_n, e_N
            rotMat = quat2rotmat(Histposes(end).attQuat);
            ImpactIdentification.inertialAcc = rotMat'*Sensor.accelerometer + [0;0;-1]; %in g's
            wallNormalDirWorld = [ImpactIdentification.inertialAcc(1:2);0];
            ImpactIdentification.wallNormalWorld = wallNormalDirWorld/norm(wallNormalDirWorld);          
        end
    end


end

