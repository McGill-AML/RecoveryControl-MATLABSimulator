function Monte = updatemontecarlo(k, IC, Hist, Monte)
    Monte.trial = [Monte.trial; k];
    Monte.IC = [Monte.IC; IC];
    Monte.hist = [Monte.hist; Hist];
end