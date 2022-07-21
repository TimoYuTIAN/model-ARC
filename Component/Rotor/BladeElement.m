classdef BladeElement < handle
    properties (SetAccess=private)
        transform;
        dr;
        chord;
        radius;
        SamplePoint;
        profile;
    end
    methods
        function obj = BladeElement(transform, dr, chord, radius, SamplePoint, profile)
            if nargin > 0
                obj.transform = transform;
                obj.dr = dr;
                obj.chord = chord;
                obj.radius = radius;
                obj.SamplePoint = SamplePoint;
                obj.profile = profile;
            end
        end
        function res = GetFM(obj, state, ctrl, envs)
            if nargin >= 4
                posb = obj.transform.DCMbe * obj.transform.pos;
                euler = obj.transform.euler;
                euler.yaw = euler.yaw + ctrl.yaw;
                euler.pitch = euler.pitch + ctrl.pitch;
                euler.roll = euler.roll + ctrl.roll;
                trans = Transform([0;0;0], euler);
                pose = trans.DCMeb * posb;
                trans.setPos(pose);
            else
                trans = obj.transform;
            end

            Vb = Coordinate.Ve2b(state.uvw, trans, state.pqr);
            V2 = Vb'*Vb;
            alpha = rad2deg(atan2(Vb(3), -Vb(2)));

            switch obj.profile.para.type
                case 'constant'
                    CLCD = obj.profile.GetCLCD([]);
                case 'interp1'
                    CLCD = obj.profile.GetCLCD(alpha);
                case 'interp2'
                    Ma = sqrt(V2) / envs.soundspeed;
                    CLCD = obj.profile.GetCLCD([alpha Ma]);
                case 'UDF'
                    query.obj=obj;
                    query.alpha=alpha;
                    query.state=state;
                    query.envs=envs;
                    query.ctrl=ctrl;
                    CLCD=obj.profile.GetCLCD(query);
            end
            qS = 0.5 * envs.rho * V2 * obj.chord * obj.dr;
            L = CLCD(1) * qS;
            D = CLCD(2) * qS;
            LD = [0; D; -L]; % 风轴系下的力
            Fb = angle2dcm(0, 0, deg2rad(alpha)) * LD; % 叶素坐标系下的力
            FM = Coordinate.FMb2e(Fb, trans); % 桨轴系下的力和力矩
            res.FM = FM;
        end
    end
end