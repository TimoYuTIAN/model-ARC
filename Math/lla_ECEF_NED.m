classdef lla_ECEF_NED<handle
properties (SetAccess = private)
    f = 1/298.257223563;%地球极扁率
    a =  6378137.0;%长轴
    b = 6356752.31424518;%短轴
    e;%偏心率
    e1;
end
methods
    function obj = lla_ECEF_NED()
        obj.e = sqrt(obj.a^2 - obj.b^2)/obj.a;
        obj.e1 = sqrt(obj.a^2 - obj.b^2)/obj.b;
    end
    % lla to ecef
    function pos_ecef = lla_ecef(obj, lla)
        lat = deg2rad(lla(1));
        lon = deg2rad(lla(2));
        h = lla(3);
        N = obj.a/sqrt(1 - obj.e^2*(sin(lat))^2); 
        X = (N+h)*cos(lat)*cos(lon);
        Y = (N+h)*cos(lat)*sin(lon);
        Z = (N*(1-obj.e^2)+h)*sin(lat);   
        pos_ecef = [X Y Z];
    end
    % ecef to lla
    function lla = ecef_lla(obj, pos)
        x = pos(1);
        y = pos(2);
        z = pos(3);
        p = sqrt(x^2 + y^2);
        q = atan2(z*obj.a, p*obj.b);
        lon = atan2(y,x);
        lat = atan2((z+(obj.e1^2)*obj.b*sin(q)^3), ...
            (p-(obj.e^2)*obj.a*cos(q)^2));
        N = obj.a/sqrt(1-(obj.e^2*sin(lat)^2));
        h = p*cos(lat) + (z+obj.e^2*N*sin(lat))*sin(lat)-N;
        lla = [rad2deg([lat, lon]), h];
end
end
end