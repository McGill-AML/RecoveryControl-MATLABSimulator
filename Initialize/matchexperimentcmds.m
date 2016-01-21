function Experiment = matchexperimentcmds(rawManualCmd,inputData)
    
    
    Experiment.manualCmds = [];
    Experiment.propCmds = [];
    
    for iCmds = 1:size(rawManualCmd.times,1)
        manualCmd.time = rawManualCmd.times(iCmds);
        manualCmd.attEuler = [rawManualCmd.rolls(iCmds);rawManualCmd.pitches(iCmds);0];
        manualCmd.angVel = [0;0;rawManualCmd.yawDerivs(iCmds)];
        manualCmd.thrust = rawManualCmd.thrusts(iCmds);
        Experiment.manualCmds = [Experiment.manualCmds; manualCmd];
    end

    for iMotorCmds = 1:size(inputData.motors_time,1)
        propCmd.rpmTime = inputData.motors_time(iMotorCmds);
        propCmd.rpmDeriv = inputData.motors_slope(iMotorCmds,:)';
        Experiment.propCmds = [Experiment.propCmds;propCmd];
    end
end