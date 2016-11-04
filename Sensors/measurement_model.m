function [Sensor,sensParams] = measurement_model(state, stateDeriv, sensParams, tStep)
    global g mag
    
    position = state(7:9);

    velocity = state(1:3); %velocity in body frame

    accel_b = stateDeriv(1:3); %acceleration in body

    eta = state(10);
    epsilon = state(11:13);

    omega_b_ba = state(4:6);

    rotMat = quat2rotmat([eta;epsilon]);

    %if crash occurs use higher variance for noise gen, reduce var
    %exponentially as time from impact grows. reset time of crash if another
    %impact or revert to normal variance if long enough. 
    if ~sensParams.crash.occur
        Sensor.acc = accel_b + rotMat*[0;0;g]+ sensParams.bias.acc + randn(3,1).*sensParams.var_acc;
        Sensor.gyro = omega_b_ba + sensParams.bias.gyr + randn(3,1).*sensParams.var_gyr;
    else
        var_acc_crash = sensParams.crash.var_acc*exp(-sensParams.crash.time_since/sensParams.crash.time_const);
        if  var_acc_crash <= sensParams.var_acc
            sensParams.crash.occur = 0;
            Sensor.acc = accel_b + rotMat*[0;0;g]+ sensParams.bias.acc + randn(3,1).*sensParams.var_acc;
            Sensor.gyro = omega_b_ba + sensParams.bias.gyr + randn(3,1).*sensParams.var_gyr;
            
        else
            
            Sensor.acc = accel_b + rotMat*([0;0;g])+ sensParams.bias.acc + randn(3,1).*var_acc_crash;
            Sensor.gyro = omega_b_ba + sensParams.bias.gyr + randn(3,1).*sensParams.crash.var_gyr*exp(-sensParams.crash.time_since/sensParams.crash.time_const);
            
            sensParams.crash.time_since = sensParams.crash.time_since + tStep;
        end
    end

    Sensor.mag = rotMat*mag + sensParams.bias.mag + randn(3,1).*sensParams.var_mag;

    gps = XYZ_to_GPS(position, rotMat'*velocity, sensParams.gps_init);

    Sensor.gps = gps(1:5) +  [sensParams.bias.gps; zeros(2,1)] + randn(5,1).*sensParams.var_gps;

    % z is negative because here z is down - also we assume the system
    % initialized to whatever the GPS initial height is
    Sensor.baro = 101325*(1-2.25577*10^-5*(-position(3)+sensParams.gps_init(3)))^5.25588 + sensParams.bias.baro + randn(1,1)*sensParams.var_baro;


end
