%test flight_circle;
clc;clear all;

para.lat0 = 30;
para.lon0 = 120;
para.h0 = 5;
para.R = 20;
para.psi = 30;
para.Clength = 10;
para.CAngle = 45;

obj = FlightCircle(para);
[ned_load, ned_left, ned_right] = obj.cal_ned_circle;
[gps_load, gps_heli_left, gps_heli_right] = obj.cal_gps_circle;

%% plot
plot3(ned_load(1,:), ned_load(2,:), ned_load(3,:)); hold on;
plot3(ned_left(1,:), ned_left(2,:), ned_left(3,:)); hold on;
plot3(ned_right(1,:), ned_right(2,:), ned_right(3,:));
