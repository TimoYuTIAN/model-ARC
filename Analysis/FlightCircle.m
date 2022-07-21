classdef FlightCircle<handle
%     目前只适合双机吊挂，后面可以增加
properties(SetAccess = private)
    lat0; lon0; h0; 
    R;%圆圈半径
    psi;%初始偏航角
    theta;%采样点数组
    nSample;%采样点个数
    wgs84;
    CLength;%吊索长度
    CAngle;%吊索角度
end
methods
    function obj = FlightCircle(para)
        obj.lat0 = para.lat0;
        obj.lon0 = para.lon0;
        obj.h0 = para.h0;
        obj.R = para.R;
        obj.psi = deg2rad(para.psi);

        obj.theta = deg2rad(0:10:360);
        obj.nSample = size(obj.theta, 2);
        obj.wgs84 = wgs84Ellipsoid('meter');

        obj.CLength = para.Clength;
        obj.CAngle = deg2rad(para.CAngle);
    end 

    function [ned_load, ned_heli_left, ned_heli_right] = cal_ned_circle(obj)
       xyz_load = [obj.R*sin(obj.theta); obj.R*cos(obj.theta); zeros(1,obj.nSample)] +  [0 -obj.R 0]';
       ned_load = angle2dcm(-obj.psi, 0, 0) * xyz_load;
       DCM_heli = angle2dcm(-(obj.psi -obj.theta), zeros(1,obj.nSample), zeros(1,obj.nSample));
       for i = 1:obj.nSample
           ned_heli_left(:,i) = ned_load(:,i) + DCM_heli(:,:,i) * [0 -obj.CLength*cos(obj.CAngle) -obj.CLength*obj.CAngle]';
           ned_heli_right(:,i) = ned_load(:,i) + DCM_heli(:,:,i) * [0 obj.CLength*cos(obj.CAngle) -obj.CLength*obj.CAngle]';
       end
    end

    function [gps_load, gps_heli_left, gps_heli_right] = cal_gps_circle(obj)
        [ned_load, ned_heli_left, ned_heli_right] = obj.cal_ned_circle;
        [x, y, z] = ned2ecef(ned_load(1,:), ned_load(2,:), ned_load(3,:), obj.lat0, obj.lon0, obj.h0, obj.wgs84);
        ecef_load = [x' y' z'];
        gps_load = ecef2lla(ecef_load);

        [x, y, z] = ned2ecef(ned_heli_left(1,:), ned_heli_left(2,:), ned_heli_left(3,:), obj.lat0, obj.lon0, obj.h0, obj.wgs84);
        ecef_heli_left = [x' y' z'];
        gps_heli_left = ecef2lla(ecef_heli_left);

        [x, y, z] = ned2ecef(ned_heli_right(1,:), ned_heli_right(2,:), ned_heli_right(3,:), obj.lat0, obj.lon0, obj.h0, obj.wgs84);
        ecef_heli_right = [x' y' z'];
        gps_heli_right = ecef2lla(ecef_heli_right);
    end
end
end