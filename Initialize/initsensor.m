function Sensor = initsensor(state, state_deriv, sensParams)
    global g mag
    
    position = state(7:9);

    velocity = state(1:3); % velocity in body

    accel = state_deriv(1:3);  %  accel in body

    eta = state(10);
    epsilon = state(11:13);

    omega_b_ba = state(4:6);

    rotMat = quat2rotmat([eta;epsilon]);

    Sensor.acc = accel + rotMat*[0;0;g]+ sensParams.bias.acc + randn(3,1).*sensParams.var_acc;
    
    Sensor.gyro = omega_b_ba + sensParams.bias.gyr + randn(3,1).*sensParams.var_gyr;


    Sensor.mag = rotMat*mag + sensParams.bias.mag + randn(3,1).*sensParams.var_mag;

    gps = XYZ_to_GPS(position, rotMat'*velocity, sensParams.gps_init);

    Sensor.gps = gps(1:5) +  [sensParams.bias.gps; zeros(2,1)] + randn(5,1).*sensParams.var_gps;

    % z is negative because here z is down - also we assume the system
    % initialized to whatever the GPS initial height is
    Sensor.baro = 101325*(1-2.25577*10^-5*(position(3)+sensParams.gps_init(3)))^5.25588 + sensParams.bias.baro + randn(1,1)*sensParams.var_baro;

   
end
