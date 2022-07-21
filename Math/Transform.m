classdef Transform < handle

    properties (SetAccess = private)
        pos = [0; 0; 0] % pos in pareant coordinate
        euler = struct('yaw', 0, 'pitch', 0, 'roll', 0)
        DCMbe = eye(3);
        DCMeb = eye(3);
        Q = [1 0 0 0];
        DCM_pqr2omega;
        DCM_omega2pqr;
    end

    methods

        function obj = Transform(pos, euler)

            if nargin > 0
                obj.pos = pos;
                obj.euler = euler;
                update(obj);
            end

        end

        function setPos(obj, pos)
            obj.pos = pos;
        end

        function setEuler(obj, euler)
            obj.euler = euler;
            update(obj);
        end

        function update(obj)
            angle1 = deg2rad(obj.euler.yaw);
            angle2 = deg2rad(obj.euler.pitch);
            angle3 = deg2rad(obj.euler.roll);
            obj.DCMbe = angle2dcm(angle1, angle2, angle3);
            obj.DCMeb = obj.DCMbe';
            obj.Q = angle2quat(angle1, angle2, angle3);
            obj.DCM_pqr2omega = [1 sin(angle3)*tan(angle2) cos(angle3)*tan(angle2)
                0 cos(angle3) -sin(angle3)
                0 sin(angle3)/cos(angle2) cos(angle3)/cos(angle2)];
            obj.DCM_omega2pqr = [1 0 -sin(angle2)
                0 cos(angle3) sin(angle3)*cos(angle2)
                0 -sin(angle3) cos(angle3)*cos(angle2)];
        end

    end

end

