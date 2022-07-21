function output = trans(u, type)
    angle1 = deg2rad(u(3));
    angle2 = deg2rad(u(2));
    angle3 = deg2rad(u(1));
    switch type
        case 'e2b'
            output = angle2dcm(angle1, angle2, angle3) * u(4:6);
        case 'b2e'
            output = angle2dcm(angle1, angle2, angle3)' * u(4:6);
        otherwise
            error("不支持的类型");
    end
end
