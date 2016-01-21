impact_time = 18.2537;

end_time = impact_time + 1;
impact_idx = vlookup(v_motors___time,impact_time);
end_idx = size(v_accel___time,1)-1; %vlookup(v_motors___time,end_time);

motor1 = v_motors_z;
motor2 = v_motors_w;
motor3 = v_motors_x;
motor4 = v_motors_y;
motors_speed = [motor1,motor2,motor3,motor4];
time = v_motors___time;

motors_time = zeros(end_idx-impact_idx,1);
motors_slope = zeros(end_idx-impact_idx,4);

for k = impact_idx:end_idx
    i = k - impact_idx + 1;
    motors_time(i) = time(k) - impact_time;
    
    for motor_idx = 1:4
        motors_slope(i,motor_idx) = (motors_speed(k+1,motor_idx)~=1)*((motors_speed(k+1,motor_idx)-motors_speed(k,motor_idx))/(time(k+1)-time(k)));
    end
end    

motors_time(1) = 0;
