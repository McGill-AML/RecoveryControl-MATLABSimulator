function Monte = initmontecarlo(IC)
    Monte.trial = 0;
    Monte.impactOccured = 0;
    Monte.impactDetected = 0;
    Monte.IC = IC;
    Monte.xVelocity = 0;
    Monte.failedDetections = 0;
    Monte.recovery = [0 0 0 0];
    Monte.heightLoss = [0 0 0];
    Monte.horizLoss = [0 0 0];
    Monte.fuzzyInput = [0 0 0 0];
    Monte.fuzzyOutput = 0;
    Monte.accelRef = [0 0 0];
    Monte.finalHorizVel = 0;
end
