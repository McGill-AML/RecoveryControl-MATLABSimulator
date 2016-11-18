function [out, move_avg] = moving_avg_filt(sensor, move_avg)

% moving average filter where move_avg is the previous measurements - with
% the oldest measurement located at move_avg(1)


if move_avg(:,end) ~= 0
    move_avg(:,1:end-1) = move_avg(:,2:end);
    move_avg(:,end) = sensor;
    
    out = sum(move_avg,2)/length(move_avg);
else
    
    for ii = 1:length(move_avg)
        if move_avg(:,ii) == 0
            move_avg(:,ii) = sensor;
            
            out = sum(move_avg,2)/ii;
            break
        elseif ii == length(move_avg)
            move_avg(:,1:end-1) = move_avg(:,2:end);
            move_avg(:,end) = sensor;
            
            out = sum(move_avg,2)/ii;
        end
    end
end