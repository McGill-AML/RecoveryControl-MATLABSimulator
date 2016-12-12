%FindMotorSlopes.m ONLY NEED FOR RECREATING SPIRI CRASHES: Record motor slopes from Spiri Data.
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

impact_time = 39.17;

if exist('v_motors___time','var') == 0
    v_motors___time = vmotors___time;
%     v_accel___time = vaccel___time;
    v_motors_z = vmotors_z;
    v_motors_w = vmotors_w;
    v_motors_x = vmotors_x;
    v_motors_y = vmotors_y;
end

end_time = min(impact_time + 1,floor(v_motors___time(end)));
impact_idx = vlookup(v_motors___time,impact_time);
end_idx = size(v_motors___time,1)-1; %vlookup(v_motors___time,end_time);

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
