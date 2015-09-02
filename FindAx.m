function [Ax] = FindAx(roll, pitch, head, tilt)

if roll == 0 && pitch == 0
    Ax = 0;
else
    if head == 0 || head == pi
        if roll >= eps
            error('Quadcopter does not move in strictly X direction');
        end
        fit_x = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25)];% deg2rad(35) deg2rad(45)];
        fit_y = [0.8575 1.7330 2.5829 3.3559 4.1361];% 5.5189 6.4832];
%         fit = polyfit(fit_x,fit_y,2);
%         Ax = -sign(pitch)*(fit(1)*abs(pitch)^2 + fit(2)*abs(pitch) + fit(1)*abs(pitch));

        fit = polyfit(fit_x,fit_y,1);
        Ax = (-fit(1)*(pitch)+ fit(2));
        
    elseif abs(head) == pi/2
        if pitch >= eps 
            error('Quadcopter does not move in strictly X direction');
        end
        fit_x = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25)];% deg2rad(35) deg2rad(45)];
        fit_y = [0.8575 1.7330 2.5829 3.3559 4.1361];% 5.5189 6.4832];
%         fit = polyfit(fit_x,fit_y,2);
%         Ax = -sign(pitch)*(fit(1)*abs(pitch)^2 + fit(2)*abs(pitch) + fit(1)*abs(pitch));

        fit = polyfit(fit_x,fit_y,1);
        Ax = (fit(1)*abs(roll)+ fit(2));
        
    elseif abs(head) == pi/4 || abs(head) == 3*pi/4
        if abs(abs(roll) - abs(pitch)) >= eps*10
            error('Quadcopter does not move in strictly X direction');
        end
%         fit_x = deg2rad(2:2:20);
%         fit_y = [0.4841 0.9668 1.4469 1.9228 2.3928 2.8552 3.3076 3.7476 4.1725 4.5788];
%         fit = polyfit(fit_x,fit_y,2);
%         Ax = (fit(1)*abs(pitch)^2 + fit(2)*abs(pitch) + fit(3));
        
        fit_x = [0.084825925596688 0.173822851313807 0.262463069971117 0.350546591578811 0.437854666001268 0.524146996691413];
        fit_y = [0.831401194164598 1.704934243053496 2.539077514413837 3.380150911758541 4.188051734756724 4.920050077041602];

        
        fit = polyfit(fit_x,fit_y,1);
        Ax = (fit(1)*tilt + fit(2));
        
    end
end
end