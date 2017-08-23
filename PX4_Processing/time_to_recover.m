% script to pull out time to recover for 1st and second stage
clear;
no_recovery = zeros(60,1);
total_stage_1 = zeros(60,1); % if multiple impacts this is time for all impacts to complete stage one
total_total = zeros(60,1);  % if multiple impacts this is time for all impacts to recover fully
total_stage_2 = zeros(60,1);  % if multiple impacts this is time for all impacts to complete stage two
first_stage_1 = zeros(60,1);  %for the first impact this is time for all impacts to complete stage one
first_stage_2 = zeros(60,1);%for the first impact this is time for all impacts to complete stage two
first_total = zeros(60,1); %for the first impact  this is time for all impacts to recover fully

% for non EKF set (set s), theres no file for s07 and s20, and s52, 53, s54 and
% s56, and s59 dont enter recovery stage 1. So theres only 53 viable recoveries. fileNo = [1:6, 8:19, 21:60]

% no_recovery(7) = 1;
% no_recovery(20) = 1;
% for EKF sets (set a), theres no file for 46  fileNo = [1:45, 47:60]
no_recovery(46) = 1;

for fileNo = [1:60]
    
    initExtData;
    stage_1_start = 0;
    stage_2_start = 0;
    stage_2_end = 0;
    multiple_impacts(fileNo) = -1;
    
    
    for ii = 2:length(IRST_RS) 
        if IRST_RS(ii-1) == 0 && IRST_RS(ii) == 1 && multiple_impacts(fileNo) == -1
            stage_1_start = TIME(ii);
        elseif IRST_RS(ii-1) == 1 && IRST_RS(ii) == 2  && multiple_impacts(fileNo) == -1
            stage_2_start = TIME(ii);
        elseif IRST_RS(ii-1) == 2 && IRST_RS(ii) == 0  && multiple_impacts(fileNo) == -1
            stage_2_end = TIME(ii);
            multiple_impacts(fileNo) = 0;
        elseif IRST_RS(ii-1) == 0 && IRST_RS(ii) == 1 && multiple_impacts(fileNo) == 0
            stage_1_start = [stage_1_start; TIME(ii)];
            multiple_impacts(fileNo) = 1;
        elseif IRST_RS(ii-1) == 1 && IRST_RS(ii) == 2  && multiple_impacts(fileNo) == 1
            stage_2_start = [stage_2_start; TIME(ii)];
        elseif IRST_RS(ii-1) == 2 && IRST_RS(ii) == 0  && multiple_impacts(fileNo) == 1
            stage_2_end = [stage_2_end; TIME(ii)];
        end
        
    end
    
    if   multiple_impacts(fileNo) == 1 && (length(stage_1_start) > length(stage_2_start) ||  length(stage_2_start) > length(stage_2_end))
        no_recovery(fileNo) = 1;
    elseif  multiple_impacts(fileNo) <=0 && (stage_2_end == 0 || stage_2_start == 0)
        no_recovery(fileNo) = 1;
    else
        no_recovery(fileNo) = 0;
        
    end
    
    if no_recovery(fileNo) == 0
        total_total(fileNo) = sum(stage_2_end - stage_1_start);
        total_stage_1(fileNo) = sum(stage_2_start - stage_1_start);
        total_stage_2(fileNo) = sum(stage_2_end - stage_2_start);
        first_total(fileNo) = stage_2_end(1) - stage_1_start(1);
        first_stage_1(fileNo) = stage_2_start(1) - stage_1_start(1);
        first_stage_2(fileNo) = stage_2_end(1) - stage_2_start(1);
        
    elseif multiple_impacts(fileNo) == 1
        max_len = length(stage_1_start);
        
        total_total(fileNo) = sum(stage_2_end(1:max_len-1) - stage_1_start(1:max_len-1));
        total_stage_1(fileNo) = sum(stage_2_start(1:max_len-1) - stage_1_start(1:max_len-1));
        total_stage_2(fileNo) = sum(stage_2_end(1:max_len-1) - stage_2_start(1:max_len-1));
        first_total(fileNo) = stage_2_end(1) - stage_1_start(1);
        first_stage_1(fileNo) = stage_2_start(1) - stage_1_start(1);
        first_stage_2(fileNo) = stage_2_end(1) - stage_2_start(1);
        
    elseif stage_2_start ~= 0
        total_stage_1(fileNo) = stage_2_start(1) - stage_1_start(1);
        first_stage_1(fileNo) = stage_2_start(1) - stage_1_start(1);
            
    end
    
    
end

no_reco_cnt = 4;
for ii = 1:no_reco_cnt
    
    while 1
        dont_use = round(60*rand(1));
        if no_recovery(dont_use) == 0
        no_recovery(dont_use) = 1;
            break
        end
    end
end

    


% mean(total_stage_1(~no_recovery))
mean_of_some_less = [mean(first_stage_1(~no_recovery))
mean(first_stage_2(~no_recovery))
mean(first_total(~no_recovery))
        
std(first_stage_1(~no_recovery))
std(first_stage_2(~no_recovery))
std(first_total(~no_recovery))]

% cov(first_stage_1(~no_recovery))
% cov(first_stage_2(~no_recovery))
% cov(first_total(~no_recovery))
        
        
        