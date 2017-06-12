function [ImpactInfo, ImpactIdentification, timeCalc] = detectimpact(iSim, ImpactInfo,ImpactIdentification, Sensor,Histposes,PREIMPACT_ATT_CALCSTEPFWD,tStep)
%DETECTIMPACT Summary of this function goes here
%   Detailed explanation goes here
    global g
    timeCalc = [0;0;0;0];
    if ImpactInfo.firstImpactDetected == 0
        if norm(Sensor.acc(1:2))>= g  && ImpactInfo.firstImpactOccured %Accelerometer horizontal magnitude
%         if norm(estForceExternalBody) >= 1 %External Force estimation
            tic;
            ImpactInfo.firstImpactDetected = 1;
            ImpactIdentification.timeImpactDetected = iSim;
            
            %pre-impact attitude
            ImpactIdentification.rotMatPreImpact = quat2rotmat(Histposes(round(iSim/tStep+1)-PREIMPACT_ATT_CALCSTEPFWD).attQuat);      
                        
            %wall location, i.e. e_n, e_N
            rotMat = quat2rotmat(Histposes(round(iSim/tStep+1)).attQuat);
            ImpactIdentification.inertialAcc = rotMat'*Sensor.acc + [0;0;-g]; %in Newtons
            wallNormalDirWorld = [ImpactIdentification.inertialAcc(1:2);0];
            ImpactIdentification.wallNormalWorld = wallNormalDirWorld/norm(wallNormalDirWorld);          
            timeCalc(1) = toc;
        end
    end
    
   


end

