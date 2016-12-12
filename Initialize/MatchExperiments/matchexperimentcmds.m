function Experiment = matchexperimentcmds(rawManualCmd,inputData)   
%matchexperimentcmds.m ONLY NEED FOR RECREATING SPIRI CRASHES: Match
%post-collision motor commands and manual (joystick) control
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%    
    Experiment.manualCmds = [];
    Experiment.propCmds = [];
    
    if ~isempty(rawManualCmd)
        for iCmds = 1:size(rawManualCmd.times,1)
            manualCmd.time = rawManualCmd.times(iCmds);
            manualCmd.attEuler = [rawManualCmd.rolls(iCmds);rawManualCmd.pitches(iCmds);0];
            manualCmd.angVel = [rawManualCmd.rollDerivs(iCmds);...
                                rawManualCmd.pitchDerivs(iCmds);...
                                rawManualCmd.yawDerivs(iCmds)];            
            manualCmd.thrust = rawManualCmd.thrusts(iCmds);
            Experiment.manualCmds = [Experiment.manualCmds; manualCmd];
        end
    end

    if ~isempty(inputData)
        for iMotorCmds = 1:size(inputData.motors_time,1)
            propCmd.rpmTime = inputData.motors_time(iMotorCmds);
            if isfield(inputData,'motors_slope')
                propCmd.rpmDeriv = inputData.motors_slope(iMotorCmds,:)';
            end
            if isfield(inputData,'motors_rpm')
                propCmd.rpm = inputData.motors_rpm(iMotorCmds,:)';
            end
            Experiment.propCmds = [Experiment.propCmds;propCmd];

        end
    end
end