% script to pull out recovery stage beginning and end in order to process
% the experimental data 
% also find when Vicon cuts out
crashIndex = zeros(2,78);
viconDropIndex = [];
timeIsDrop = 10;
maxDropLength = zeros(1,78);

for fileNo = [1:13,15:67, 71:78]
    
    %load data
    initExtData;
    
    %init vicon drops to 0
    viconDropCount = 0;
    maxDropLenThisRun = 0;
    viconDropsThisFile = [];
    
    for jj = 2:length(IRST_RS)
        %check if collision recov controller inits
        if IRST_RS(jj) > 0 && IRST_RS(jj-1) == 0
            crashStart = jj;
        end
        
        %check if collision recov controller exits
        if IRST_RS(jj-1) >= 1 && IRST_RS(jj) == 0
            crashEnd = jj;
        end
        
        %check if vicon signal hasn't changed since last.
        if (VISN_QuatW(jj) == VISN_QuatW(jj-1) && VISN_QuatX(jj) == VISN_QuatX(jj-1) && VISN_QuatY(jj) == VISN_QuatY(jj-1) && VISN_QuatZ(jj) == VISN_QuatZ(jj-1))
            viconDropCount = viconDropCount + 1;
            
            %if vicon is same for more than 3 steps, consider it dropped
            if viconDropCount == timeIsDrop
                viconDropStart = jj-timeIsDrop;
            end
            
        elseif viconDropCount > timeIsDrop %if vicon gets new data after at least 3 of the same, consider it restablished
            viconDropEnd = jj;
            
            %add data to drop index for this file
            viconDropsThisFile = [viconDropsThisFile; viconDropStart; viconDropEnd];
            
            viconDropCount = 0;
            
            if viconDropEnd-viconDropStart > maxDropLenThisRun
                maxDropLenThisRun = viconDropEnd-viconDropStart;
            end
        else
            viconDropCount = 0;
        end
            
    end
    
    viconDropIndex{fileNo} = viconDropsThisFile;
    maxDropLength(fileNo) = maxDropLenThisRun;
    crashIndex(:,fileNo) = [crashStart; crashEnd];
end
            
 save('.\PX4_Processing\qRunIndices.mat', 'viconDropIndex','maxDropLength','crashIndex');           