function [sensParams] = initgps_baro(Sensor, sensParams)

    %normally these would be computed in the estimator, here since they are
    %used in all estimators they are computed and passed as a Sensor value
    Me = 6378137*(1-0.08181919^2)/(1-(0.08181919*sin(Sensor.gps(1)*pi/180.0))^2)^1.5;
    Ne = 6378137/sqrt(1-(0.08181919*sin(Sensor.gps(1)*pi/180.0))^2); 
        
    sensParams.gps_init = [sensParams.gps_init(1:3)';  Me; Ne];  %initial GPS lat / long / height and computation constants
    sensParams.baro_init = ((Sensor.gps(3)/101325)^(1/5.25588)-1)/(-2.25577*10^-5);

end