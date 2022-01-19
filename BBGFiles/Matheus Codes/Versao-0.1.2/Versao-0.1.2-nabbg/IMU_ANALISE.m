clc; clear all; close all;

imu_data = load('IMU_QUAD.txt');

accx=imu_data(:,1);
accy=imu_data(:,2);
accz=imu_data(:,3);
gyrox=imu_data(:,4);
gyroy=imu_data(:,5);
gyroz=imu_data(:,6);

roll = atan2(accy, (sqrt(accx .* accx + accz .* accz))) * 180.0 / pi;
pitch = atan2((-accx), (sqrt(accy .* accy + accz .* accz))) * 180.0 / pi;

figure(1)
plot(accx)
title('accx')

figure(2)
plot(accy)
title('accy')

figure(3)
plot(accz)
title('accz')

figure(4)
plot(gyrox)
title('gyrox')

figure(5)
plot(gyroy)
title('gyroy')

figure(6)
plot(gyroz)
title('gyroz')

figure(7)
plot(roll)
title('roll(graus)')

figure(8)
plot(pitch)
title('pitch(graus)')
