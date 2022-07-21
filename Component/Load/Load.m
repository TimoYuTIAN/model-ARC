classdef Load < six_dof
    %目前仅支持长方形吊挂物
    properties(SetAccess = private)
        Init = struct( 'type', 'mass_point', ...
            'mass', 2.5, ...  %重量2.5 kg
            'AeroCoe', [0 0 0]', ... %吊挂物气动系数
            'NPoints', 1, ... %吊挂点数目
            'PosPoints', [0 0 0]', ...%吊挂点位置，列数应与吊挂点数一致
            'LWH', [1 0.5 0.5]', ... %吊挂物长宽高
            'posAero', [0 0 0]' ... 气动中心相对重心的位置
            );
        cross_area;
    end
    methods
        function obj = Load(Init)
            %初始化吊挂物六自由度方程
            para_for6dom = struct('Jxx', 1/12 * Init.mass * power(norm(Init.LWH([2,3])),2),...
                'Jyy', 1/12 * Init.mass * power(norm(Init.LWH([1,3])),2),...
                'Jzz', 1/12 * Init.mass * power(norm(Init.LWH([1,2])),2),...
                'Jxy', 0, ...
                'Jxz', 0, ...
                'Jyz', 0, ...
                'm', Init.mass);
            obj = obj@six_dof(para_for6dom);

            obj.Init = Init;
            obj.cross_area = [Init.LWH(2)*Init.LWH(3)
                Init.LWH(1)*Init.LWH(3) 
                Init.LWH(1)*Init.LWH(2)];
        end

        function AeroFM = calAero(obj, state, envs)
            switch obj.Init.type
                case "mass_point"
                    state.pqr = [0 0 0]';
                    state.angle = [0 0 0]';
            end
            obj.update_trans(state);
            uvw = state.uvw + obj.tran.DCMbe * envs.v_e(1:3) + ...
                -cross(obj.Init.posAero, state.pqr + envs.v_e(4:6));
            F = 0.5 * envs.rho * obj.Init.AeroCoe .* uvw .* uvw .* obj.cross_area;
            M = zeros(3,1);
            AeroFM = [F; M];
        end

        function FMc = calCable(obj, ctrl)
            Fc = ctrl.Fc;
            index = ctrl.index; %吊索连接吊挂点索引号

            if max(index) > obj.Init.NPoints
                error('吊索对应的吊挂点索引号超过吊挂点数目');
            elseif min(index) <= 0
                error('吊索对应的吊挂点索引号小于等于0');
            end
            
            if size(index,2) ~= size(Fc,2)
                error('吊挂点数目与吊索力数目不匹配');
            end

            PosPoints = obj.Init.PosPoints(:,ctrl.index);
            Fc_total = sum(Fc,2);
            Mc_total = sum(cross(PosPoints, Fc),2);
            FMc = [Fc_total; Mc_total];
        end

        function dif = get_dif(obj, state, ctrl, envs)
            switch obj.Init.type
                case "mass_point"
                    state.pqr = [0 0 0]';
                    state.angle = [0 0 0]';
            end

            FMa = obj.calAero(stateflow, envs); %气动力
            FMc = obj.calCable(ctrl); %吊索力
            FM = FMa + FMc; %除重力以外的合力矩
            dif = obj.six_dif(state, FM, envs);

            switch obj.Init.type
                case 'mass_point'
                    dif = [dif(1:3); dif(10:12)]; %速度导数、位置导数
            end
        end

    end
end
