for iBatch = 1:numel(Batch)
    Plot = Batch(iBatch).Plot;

    for iData = 1:numel(Plot.times)    
        rotMat = quat2rotmat(Plot.quaternions(:,iData));
%         Plot.accelerometers(:,iData) = invar2rotmat('x',pi)*(rotMat*[0;0;g] + Plot.bodyAccs(:,iData) + cross(Plot.angVels(:,iData),Plot.linVels(:,iData)))/g;
        Plot.accelerometers(:,iData) = (-rotMat*[0;0;-g] + Plot.bodyAccs(:,iData) + cross(Plot.angVels(:,iData),Plot.linVels(:,iData)))/g;

    end
    Batch(iBatch).Plot.accelerometers = Plot.accelerometers;
end