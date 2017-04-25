function [Sensor] = package_sens_data(IMU_AccX, IMU_AccY, IMU_AccZ, IMU_GyroX, IMU_GyroY, IMU_GyroZ, IMU_MagX, IMU_MagY, IMU_MagZ, GPS, BAR)

Sensor.acc = [IMU_AccX;
              IMU_AccY;
              IMU_AccZ];
          
Sensor.gyro = [IMU_GyroX;
               IMU_GyroY;
               IMU_GyroZ];
           
Sensor.mag = [IMU_MagX;
              IMU_MagY;
              IMU_MagZ];
          
Sensor.gps = GPS ;

Sensor.baro = BAR;