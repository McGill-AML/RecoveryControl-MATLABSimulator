function [xAcc] = getinitaccx(eulerAngles, tilt, givenXAcc)


if givenXAcc == 0
    roll = eulerAngles(1);
    pitch = eulerAngles(2);
    yaw = eulerAngles(3);
    
    if roll == 0 && pitch == 0
        xAcc = 0;
    else
        if abs(yaw - 0) <= deg2rad(10) || abs(abs(yaw) - pi) <= deg2rad(20)
            fitAngle = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25)];% deg2rad(35) deg2rad(45)];
            fitAcc = [0.8575 1.7330 2.5829 3.3559 4.1361];% 5.5189 6.4832];
            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (-fit(1)*(pitch)+ fit(2));

        elseif abs(yaw) == pi/2
            fitAngle = [deg2rad(5) deg2rad(10) deg2rad(15) deg2rad(20) deg2rad(25)];% deg2rad(35) deg2rad(45)];
            fitAcc = [0.8575 1.7330 2.5829 3.3559 4.1361];% 5.5189 6.4832];
            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (fit(1)*abs(roll)+ fit(2));

        elseif abs(yaw) == pi/4 || abs(yaw) == 3*pi/4
            
            if abs(abs(roll) - abs(pitch)) >= eps*10
                error('Quadcopter does not move in strictly X direction');
            end

            fitAngle = [0.084825925596688 0.173822851313807 0.262463069971117 0.350546591578811 0.437854666001268 0.524146996691413];
            fitAcc = [0.831401194164598 1.704934243053496 2.539077514413837 3.380150911758541 4.188051734756724 4.920050077041602];

            fit = polyfit(fitAngle,fitAcc,1);
            xAcc = (fit(1)*tilt + fit(2));

        else
            error('Need to find Ax');
        end
    end
    
else
    xAcc = givenXAcc;
end
end