function gps_out = XYZ_to_GPS(position, velocity, init_GPS)

% converts position in the NED frame to GPS coordinates
% init_GPS = [lat_0, long_0, altitude_0]

lat_0 = init_GPS(1);
long_0 = init_GPS(2);
altitude_0 = init_GPS(3); %above sea level

Me = 6378137*(1-0.08181919^2)/(1-(0.08181919*sin(lat_0*pi/180.0))^2)^1.5;
Ne = 6378137/sqrt(1-(0.08181919*sin(lat_0*pi/180.0))^2);

height = altitude_0-position(3);

lat = position(1)*180/pi/(Me+height)+lat_0;
lat = floor(lat*10000000)/10000000;

long = position(2)*180/pi/((Ne+height)*cos(lat_0*pi/180.0))+long_0;
long = floor(long*10000000)/10000000;

NS_vel = velocity(1);

EW_vel = velocity(2);

gps_out = [lat; long; height; NS_vel; EW_vel];
