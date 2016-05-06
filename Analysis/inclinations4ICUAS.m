clc

%in degrees
rollNominal =   1.08;
pitchNominal =  0.25;
yawNominal =    13.5-180;

rollError = 2.7;
pitchError = 6.4;
yawError = 6.4;

rollArray = [rollNominal; rollNominal - rollError; rollNominal + rollError];
pitchArray = [pitchNominal; pitchNominal - pitchError; pitchNominal + pitchError];
yawArray = [yawNominal; yawNominal - yawError; yawNominal + yawError];
inclinationArray = zeros(27,1);
idxInclination = 1;
for iRoll = 1:3
    for jPitch = 1:3
        for kYaw = 1:3
            attEuler = deg2rad([rollArray(iRoll);pitchArray(jPitch);yawArray(kYaw)]);
            rotMat = quat2rotmat(angle2quat(-(attEuler(1)+pi),attEuler(2),attEuler(3),'xyz')');
            inclinationArray(idxInclination) = rad2deg(getinclination(rotMat,attEuler(3)));
            
            if (iRoll + jPitch + kYaw) == 3
                inclinationNominal = inclinationArray(idxInclination);
            end
            
            idxInclination = idxInclination+1;
        end
    end
end

inclinationErrorArray = inclinationArray - inclinationNominal;
inclinationNominal
min(inclinationErrorArray);
max(inclinationErrorArray);
