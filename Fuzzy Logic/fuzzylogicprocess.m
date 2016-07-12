function [FuzzyInfo] = fuzzylogicprocess(iSim, ImpactInfo, ImpactIdentification,...
                                    Sensor,currentPose, SimParams, Control, FuzzyInfo)

if SimParams.useRecovery == 1 && Control.recoveryStage == 0
    if ImpactInfo.firstImpactDetected == 1 && Control.accelRefCalculated == 0%Calculate fuzzy inputs
        for iInput = 1:4
            if FuzzyInfo.InputsCalculated(iInput) == 0
                if FuzzyInfo.InputArray(iInput).calcStepDelay <= iSim - ImpactIdentification.timeImpactDetected
                %waited enuff time to calculate
                    switch iInput 
                        case 1 %FLP input 1: Accelerometer Horizontal Magnitude
                            FuzzyInfo.InputArray(iInput).value = norm(Sensor.accelerometer(1:2));
%                             FuzzyInfo.InputArray(iInput).value = norm(estForceExternalBody);
                            disp('input 1 calc');
                        case 2 %FLP input 2: Inclination
                            estWallTangentWorld = cross([0;0;1],ImpactIdentification.wallNormalWorld);
                            negBodyZ = [0;0;-1];
                            bodyZProjection = ImpactIdentification.rotMatPreImpact'*negBodyZ - dot((ImpactIdentification.rotMatPreImpact'*negBodyZ),estWallTangentWorld)*estWallTangentWorld;
                            dotProductWithWorldZ = dot(bodyZProjection,[0;0;1]);
                            inclinationAngle = acos(dotProductWithWorldZ/norm(bodyZProjection));
                            
                            dotProductWithWorldNormal = dot(bodyZProjection,ImpactIdentification.wallNormalWorld);
                            angleWithWorldNormal = acos(dotProductWithWorldNormal/(norm(bodyZProjection)*norm(ImpactIdentification.wallNormalWorld)));                            
                            inclinationSign = sign(angleWithWorldNormal - pi/2);                            
                            
                            FuzzyInfo.InputArray(iInput).value = inclinationSign*rad2deg(inclinationAngle); 
                            disp('input 2 calc');
                        case 3 %FLP input 3: Flipping Direction Angle
                            %Try calculating according to world frame:
                            rotMat = quat2rotmat(currentPose.attQuat); 
                            angVelWorld = rotMat'*Sensor.gyro;
                            angVelWorldPerp = cross(angVelWorld,[0;0;1]);
                            angVelWorldPerpHoriz = angVelWorldPerp(1:2);
                            wallNormalWorldHoriz = ImpactIdentification.wallNormalWorld(1:2);
                            FuzzyInfo.InputArray(iInput).value = rad2deg(acos(dot(angVelWorldPerpHoriz,wallNormalWorldHoriz)/...
                                                            (norm(angVelWorldPerpHoriz)*norm(wallNormalWorldHoriz))));
                            disp('input 3 calc');
                        case 4 %FLP input 4: Gyro Horizontal Magnitude
                            FuzzyInfo.InputArray(iInput).value = norm(Sensor.gyro(1:2));
                        disp('input 4 calc');
                    end
                    FuzzyInfo.InputsCalculated(iInput) = 1;
                end
            end
            
        end
        
        if sum(FuzzyInfo.InputsCalculated) == 4 %Inputs complete, now do the fuzzy logic
            [intensityFuzzyProcess] = initfuzzylogicprocess();
            
            temp = struct2cell(FuzzyInfo.InputArray);            
            inputFLP = [temp{3,:}];
            FuzzyInfo.output = evalfis(inputFLP, intensityFuzzyProcess);
            if abs(FuzzyInfo.output) <= eps
                FuzzyInfo.output = 0;
            end

        end
    end
end

end