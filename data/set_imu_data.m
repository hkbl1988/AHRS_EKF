clear
clc

load('NAV');

d2r = pi/180;
r2d = 180/pi;
g = 9.7803253361;

NAV = NAV;

Freq = 50;

dt = 0.02;                      %%数据频率

IMU(:,1:3) = NAV(:,12:14)*r2d;      %%陀螺数据(deg/s)
IMU(:,4:6) = -NAV(:,9:11)*10;       %%加表数据(g)
IMU(:,7:9) = NAV(:,15:17);      %%磁力计数据

IMU_Ref(:,1:3) = NAV(:,30:32)*r2d;  %%姿态角
IMU_Ref(:,4:6) = NAV(:,21:23);  %%速度
IMU_Ref(:,7) = NAV(:,19);  %%位置
IMU_Ref(:,8) = NAV(:,18);  %%位置
IMU_Ref(:,9) = NAV(:,20);  %%位置

GNSS(:,1:3) = NAV(:,6:8);       %%GNSS速度
GNSS(:,4) = NAV(:,3);       %%GNSS位置
GNSS(:,5) = NAV(:,2);       %%GNSS位置
GNSS(:,6) = NAV(:,4);       %%GNSS位置
GNSS(:,7) = NAV(:,5);           %%气压高度

save sensor.mat IMU IMU_Ref GNSS dt Freq;