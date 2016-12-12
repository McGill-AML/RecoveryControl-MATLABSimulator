function [FuzzyInfo,timeCalc] = fuzzylogicprocess_experiment(iSim, ImpactInfo, ImpactIdentification,...
                                    Sensor,currentPose, SimParams, Control, FuzzyInfo)
%fuzzylogicprocess.m Calculates FLP inputs and output for experiment
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Same as fuzzylogicprocess.m, except signs in Inclination
%                calculation for IMU mounting and world frame convention
%                are changed to match experiment convention (both IMU and
%                world frame are NED).
%-------------------------------------------------------------------------%

timeCalc = [0;0;0;0];
if SimParams.useRecovery == 1 && Control.recoveryStage == 0
    if ImpactInfo.firstImpactDetected == 1 && Control.accelRefCalculated == 0%Calculate fuzzy inputs
        tic;
        for iInput = 1:4
            if FuzzyInfo.InputsCalculated(iInput) == 0
                if FuzzyInfo.InputArray(iInput).calcStepDelay <= iSim - ImpactIdentification.timeImpactDetected
                %waited enuff time to calculate
                    switch iInput 
                        case 1 %FLP input 1: Accelerometer Horizontal Magnitude
                            tic;
                            FuzzyInfo.InputArray(iInput).value = norm(Sensor.accelerometer(1:2));
                            timeCalc(4) = timeCalc(4) + toc;
                        case 2 %FLP input 2: Inclination
                            tic;
                            estWallTangentWorld = cross([0;0;1],ImpactIdentification.wallNormalWorld);
                            FuzzyInfo.wallTangentWorld = estWallTangentWorld;
                            negBodyZ = [0;0;-1]; %[0;0;1] for NWU, %[0;0;-1] for NED [IMU mounting]
                            bodyZProjection = ImpactIdentification.rotMatPreImpact'*negBodyZ - dot((ImpactIdentification.rotMatPreImpact'*negBodyZ),estWallTangentWorld)*estWallTangentWorld;
                            dotProductWithWorldZ = dot(bodyZProjection,[0;0;-1]); %[0;0;1] for NWU, %[0;0;-1] for NED [World Frame]
                            inclinationAngle = acos(dotProductWithWorldZ/norm(bodyZProjection));
                            
                            dotProductWithWorldNormal = dot(bodyZProjection,ImpactIdentification.wallNormalWorld);
                            angleWithWorldNormal = acos(dotProductWithWorldNormal/(norm(bodyZProjection)*norm(ImpactIdentification.wallNormalWorld)));                       
                            inclinationSign = sign(angleWithWorldNormal - pi/2);                      
                            
                            FuzzyInfo.InputArray(iInput).value = inclinationSign*rad2deg(inclinationAngle);
                            timeCalc(1) = timeCalc(1) + toc;
                        case 3 %FLP input 3: Flipping Direction Angle
                            tic;
                            rotMat = quat2rotmat(currentPose.attQuat)'; %rotation matrix transposed in px4
                            angVelWorld = rotMat'*Sensor.gyro;
                            angVelWorldPerp = cross(angVelWorld,[0;0;-1]); %[0;0;1] for NWU, %[0;0;-1] for NED [World Frame]
                            angVelWorldPerpHoriz = angVelWorldPerp(1:2);
                            wallNormalWorldHoriz = ImpactIdentification.wallNormalWorld(1:2);
                            FuzzyInfo.InputArray(iInput).value = rad2deg(acos(dot(angVelWorldPerpHoriz,wallNormalWorldHoriz)/...
                                                            (norm(angVelWorldPerpHoriz)*norm(wallNormalWorldHoriz))));
                            timeCalc(3) = timeCalc(3) + toc;
                        case 4 %FLP input 4: Gyro Horizontal Magnitude
                            tic
                            FuzzyInfo.InputArray(iInput).value = norm(Sensor.gyro(1:2));
                            timeCalc(4) = timeCalc(4) + toc;
                    end
                    FuzzyInfo.InputsCalculated(iInput) = 1;
                end
            end
            
        end
        
        if sum(FuzzyInfo.InputsCalculated) == 4 %Inputs complete, now do the fuzzy logic
            tic;
            
            temp = struct2cell(FuzzyInfo.InputArray);            
            inputFLP = [temp{3,:}];
            FuzzyInfo.output = evalfis(inputFLP, FuzzyInfo.intensityFuzzyProcess);
            if abs(FuzzyInfo.output) <= eps
                FuzzyInfo.output = 0;
            end
            timeCalc(4) = timeCalc(4) + toc;

        end
    end
end

end