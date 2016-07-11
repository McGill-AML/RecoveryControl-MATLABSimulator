function Monte = initmontecarlo(IC)
    Monte.trial = 0;
    Monte.IC = IC;
    Monte.recovery = [0 0 0 0];
    Monte.heightLoss = 0;
    Monte.horizLoss = 0;
end