function Plot = monte2plot(Monte)
    Plot.trial = Monte.trial;
    % ICs
    temp = struct2cell(Monte.IC);
    % positions
    Plot.positions = [temp{1,:}];
    % body rates
    Plot.bodyrates = [temp{2,:}];
    % incoming angles
    Plot.angles = [temp{3,:}];
    % speed
    Plot.speeds = [temp{4,:}];   
    % friction
    Plot.friction = [temp{6,:}];

    
    %% Performance measures
    
    % height loss
%     % recovery bool
    
    % recovery times
    
    % height loss
    
    % horizontal loss
    


    
end