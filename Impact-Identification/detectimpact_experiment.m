function [ImpactInfo, ImpactIdentification, timeCalc] = detectimpact_experiment(iSim, tStep, ImpactInfo,ImpactIdentification, Histsensors ,Histposes,PREIMPACT_ATT_CALCSTEPFWD, stateDeriv, state)
%DETECTIMPACT Summary of this function goes here
%   Detailed explanation goes here
    timeCalc = [0;0;0;0];
    if ImpactInfo.firstImpactDetected == 0  
        rotMat = quat2rotmat(Histposes(end).attQuat)'; %rotation matrix transposed in px4
        Sensor = Histsensors(end);
        accelerometerWorld = rotMat'*Sensor.accelerometer;
        disp(norm(accelerometerWorld(1:2)))
        if norm(accelerometerWorld(1:2))>= 1 %Accelerometer horizontal magnitude
            global IMU_POSN g        
%         if norm(estForceExternalBody) >= 1 %External Force estimation
            tic;
            ImpactInfo.firstImpactDetected = 1;
            ImpactIdentification.timeImpactDetected = iSim;            
            
            %pre-impact attitude
            %rotation matrix transposed in px4
            ImpactIdentification.rotMatPreImpact = quat2rotmat(Histposes(end-PREIMPACT_ATT_CALCSTEPFWD).attQuat)';      
                        
            %wall location, i.e. e_n, e_N          
            
            ImpactIdentification.inertialAcc = rotMat'*Sensor.accelerometer + [0;0;-1]; %in g's
            wallNormalDirWorld = [ImpactIdentification.inertialAcc(1:2);0];
            ImpactIdentification.wallNormalWorld = wallNormalDirWorld/norm(wallNormalDirWorld);             
            
            % Corrected wall estimate using estimated angular acceleration
            inertialAccAtIMU = rotMat'*Sensor.accelerometer + [0;0;-1]; %in g's
            worldAngVel = rotMat'*Sensor.gyro;  
           
            worldAngAccEstimated = rotMat'*(Sensor.gyro - Histsensors(end-1).gyro)/tStep;
            inertialAccAtCMEstimated2 = inertialAccAtIMU + (cross(worldAngAccEstimated,-rotMat'*IMU_POSN) + cross(worldAngVel,cross(worldAngVel,-rotMat'*IMU_POSN)))/g;
            wallNormalDirWorld4 = [inertialAccAtCMEstimated2(1:2);0];
            ImpactIdentification.wallNormalWorldCorrected2 = wallNormalDirWorld4/norm(wallNormalDirWorld4);
            
            disp('CM Inertial Acc Estimate 2:')
            disp(inertialAccAtCMEstimated2);
            
            timeCalc(1) = toc;
        end
    end
    
   


end

