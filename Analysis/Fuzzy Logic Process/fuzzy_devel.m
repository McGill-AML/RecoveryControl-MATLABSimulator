% For testing and tuning fuzzy logic process 

IMPACT_DETECT_LIMIT = 0.5;
g = 9.81;

intensityFuzzyProcess = initFuzzyLogicProcess();
fuzzyOutput_array = zeros(numel(Batch),1);
for iBatch = 1:numel(Batch)
    
Plot = Batch(iBatch).Plot;

timeDetection = 0;

%% Set up Calculation Delay Times
% PreImpact Attitude
PREIMPACT_ATT_CALCSTEPFWD = 2;

% FLP input 1: Accelerometer Horizontal Magnitude
fuzzyInput.ID = 1;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 2; %10 to 15 ms
fuzzyInput.value = 0;
fuzzyInputArray = fuzzyInput;

%FLP input 2: Inclination
fuzzyInput.ID = 2;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 0; %available at t_detection
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

%FLP input 3: Flipping Direction Angle
fuzzyInput.ID = 3;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 2; %5 to 10 ms
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

%FLP input 4: Gyro Horizontal Magnitude
fuzzyInput.ID = 4;
% fuzzyInput.isCalculated = 0;
fuzzyInput.calcStepDelay = 2; %10 to 15 ms
fuzzyInput.value = 0;
fuzzyInputArray = [fuzzyInputArray; fuzzyInput];

fuzzyInputsCalculated = [0;0;0;0];
%%
for iSim = 1:numel(Plot.times) %run through single simulation
    a_acc = Plot.accelerometers(:,iSim);
    gyro = Plot.angVels(:,iSim);
    
    if timeDetection == 0 %collision has not been detected yet
        if norm(a_acc(1:2)) >= 0.5
            timeDetection = Plot.times(iSim);
            idxDetection = iSim;
            
            %pre-impact attitude
            rotMatPreImpact = quat2rotmat(Plot.quaternions(:,idxDetection-PREIMPACT_ATT_CALCSTEPFWD));          
                        
            %wall location, i.e. e_n, e_N
            rotMat = quat2rotmat(Plot.quaternions(:,idxDetection));

            estimatedWorldAcc = rotMat'*a_acc + [0;0;-1]; %in g's
            wallNormalWorld = [estimatedWorldAcc(1:2);0];
            wallNormalWorld = wallNormalWorld/norm(wallNormalWorld);
            
            wallNormalBody = rotMat*wallNormalWorld;

        end
    end
    
    if timeDetection ~= 0 %now in FLP

        for iInput = 1:4
            if fuzzyInputsCalculated(iInput) == 0
                if fuzzyInputArray(iInput).calcStepDelay <= iSim - idxDetection
                %waited enuff time to calculate
                    switch iInput 
                        case 1 %FLP input 1: Accelerometer Horizontal Magnitude
                            fuzzyInputArray(iInput).value = norm(a_acc(1:2));
                        case 2 %FLP input 2: Inclination
                            wallTangentWorld = cross([0;0;1],wallNormalWorld);
                            negBodyZ = [0;0;-1];
                            bodyZProjection = rotMatPreImpact'*negBodyZ - dot((rotMatPreImpact'*negBodyZ),wallTangentWorld)*wallTangentWorld;
                            dotProductWithWorldZ = dot(bodyZProjection,[0;0;1]);
                            inclinationAngle = acos(dotProductWithWorldZ/norm(bodyZProjection));
                            
                            dotProductWithWorldNormal = dot(bodyZProjection,wallNormalWorld);
                            angleWithWorldNormal = acos(dotProductWithWorldNormal/(norm(bodyZProjection)*norm(wallNormalWorld)));                            
                            inclinationSign = sign(angleWithWorldNormal - pi/2);                            
                            
                            fuzzyInputArray(iInput).value = inclinationSign*rad2deg(inclinationAngle); 
%                             disp('Prescribed Pitch:');
%                             disp(Batch(iBatch).pitch_atImpact);
%                             
%                             disp('Calculated Inclination:');
%                             disp(fuzzyInputArray(2).value);
%                             disp('--------------------');
                        case 3 %FLP input 3: Flipping Direction Angle
                            %Try calculating according to world frame:
                            angVelWorld = rotMat'*gyro;
                            angVelWorldPerp = cross(angVelWorld,[0;0;1]);
                            angVelWorldPerpHoriz = angVelWorldPerp(1:2);
                            wallNormalWorldHoriz = wallNormalWorld(1:2);
                            fuzzyInputArray(iInput).value = rad2deg(acos(dot(angVelWorldPerpHoriz,wallNormalWorldHoriz)/...
                                                            (norm(angVelWorldPerpHoriz)*norm(wallNormalWorldHoriz))));
                            
%                             disp('Vx Impact:')
%                             disp(Batch(iBatch).vel_atImpact);
%                             disp('Pitch:')                            
%                             disp(Batch(iBatch).pitch_atImpact);
%                             disp('---------------------');
                        case 4 %FLP input 4: Gyro Horizontal Magnitude
                            fuzzyInputArray(iInput).value = norm(gyro(1:2));
                    end
                    fuzzyInputsCalculated(iInput) = 1;
                end
            end
            
        end
        
        if sum(fuzzyInputsCalculated) == 4 %Inputs complete, now do the fuzzy logic
            temp = struct2cell(fuzzyInputArray);            
            inputFLP = [temp{3,:}];
            outputFLP = evalfis(inputFLP, intensityFuzzyProcess);
            fuzzyOutput_array(iBatch) = outputFLP;
            break;
        end       
 
    end
end

end

