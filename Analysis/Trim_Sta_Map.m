classdef Trim_Sta_Map < handle
    properties(SetAccess = private)
        type;
        solve_type;
        rotor;
        craft;
        SLift;
        MLift;
    end
    methods
        function obj = Trim_Sta_Map(type, solve_type)
            obj.type = type;
            obj.solve_type = solve_type;
        end

        %% 与配平相关的函数
        function result = trimming(obj, state, ctrl, envs, para, x0)
            %
            switch obj.type
                case 'Rotor'
                    obj.rotor = Rotor(para);
                    [state.uvw, state.pqr] = obj.rotor.update_state(state);
                    fun = @(x) obj.rotor_trim(state, ctrl, envs, x);
                case 'Craft'
                    obj.craft = Craft(para);
                    fun = @(x) obj.craft_trim(state, ctrl, envs, x);
                case 'SingleLift'

                case 'MultiLift'

                otherwise
                    error('不支持的配平类型')
            end

            switch obj.solve_type
                case 'newton'
                    result = Math.Newton(fun, x0);
                case 'fsolve'
                    result = fsolve(fun, x0);
                otherwise
                    error('不支持的配平求解方式');
            end
        end

        function dif = rotor_trim(obj, state, ctrl, envs, x)
            %单个旋翼配平，配平量为三个挥舞角，三个挥舞角速度，三个入流速度
            state.flap = x(1:3); state.flap_d = x(4:6); state.inflow = x(7:9);
            res = obj.rotor.FM(state, ctrl, envs);
            dif = [obj.rotor.dif_flap(state, res, envs);
                obj.rotor.dif_inflow(state, res, envs)];
        end

        function dif = craft_trim(obj, state, ctrl, envs, x)
            %整机配平，配平量为垂向、横向、纵向、航向操纵量、滚转角、俯仰角、
            %三个挥舞角，三个挥舞角速度，三个入流速度：旋翼

            ctrl.Ver = x(1); ctrl.lat = x(2);
            ctrl.Lon = x(3); ctrl.dir = x(4);
            state.angle(1:2) = x(5:6);
            state.flap = zeros(3,1); state.flap_d = zeros(3,1); state.inflow = zeros(3,1);

            %更新uvw
            obj.craft.update_trans(state);
            state.uvw = obj.craft.tran.DCMbe * state.vel;

            j = 0;
            for i = 1:obj.craft.ncomp
                if(eval("strcmp(obj.craft.comp.comp"+ num2str(i) + ".type, 'MainRotor')"))
                    eval("state.flap" + num2str(j+1) + '= x(6 + 9*j +1 : 6 + 9*j +3);');
                    eval("state.flap_d" + num2str(j+1) + '= x(6 + 9*j +4 : 6 + 9*j +6);');
                    eval("state.inflow" + num2str(j+1) + '= x(6 + 9*j +7 : 6 + 9*j +9);');
                    j = j + 1;
                end
            end

            dif = obj.craft.get_dif(state, ctrl, envs, 'trim');

        end

        function dif = SLift_trim(obj, state, ctrl, envs, x)
        end

        function dif = MLift_trim(obj, state, ctrl, envs, x)
        end
        
        %% 与操稳特性相关的函数
        function result = Craft_Sta_Map(obj, state, ctrl, envs, x_trim, dis_step)
            %操纵量配平值
            trim_ctrl = x_trim(1:4);
            ctrl.Ver = trim_ctrl(1); ctrl.lat = trim_ctrl(2);
            ctrl.Lon = trim_ctrl(3); ctrl.dir = trim_ctrl(4);
            %状态配平值
            state.angle(1:2) = x_trim(5:6);
            obj.craft.update_trans(state);
            state.uvw = obj.craft.tran.DCMbe * state.vel;
            trim_state = [state.uvw; state.pqr; state.angle';  state.pos;
                x_trim(7:numel(x_trim))];
            state = obj.update_state(trim_state);
            dif_trim = obj.craft.get_dif(state, ctrl, envs, 'Sta_Map');

            %计算A阵
            num = numel(trim_state);
            x = obj.update_DisState(trim_state, num, dis_step);
            for i = 1 : num
                %状态量小扰动值
                state = obj.update_state(x(:,i));
                A(:,i) = (obj.craft.get_dif(state, ctrl, envs, 'Sta_Map') - dif_trim) ./ dis_step;
            end

            %计算B阵
            %状态量配平值
            state = obj.update_state(trim_state);
            num = numel(trim_ctrl);
            x = obj.update_DisCtrl(trim_ctrl, num, dis_step);
            for i = 1: num
                ctrl.Ver = x(1,i); ctrl.lat = x(2,i);
                ctrl.Lon = x(3,i); ctrl.dir = x(4,i);
                B(:,i) = (obj.craft.get_dif(state, ctrl, envs, 'Sta_Map') - dif_trim) ./ dis_step;
            end

            result.A = A; result.B = B;

        end
        %计算B D阵

        function state = update_state(obj, x)
            state.uvw = x(1:3);
            state.pqr = x(4:6);
            state.angle = x(7:9);
            state.pos = x(10:12);
            obj.craft.update_trans(state);

            j = 0;
            for i = 1:obj.craft.ncomp
                if(eval("strcmp(obj.craft.comp.comp"+ num2str(i) + ".type, 'MainRotor')"))
                    eval("state.flap" + num2str(j+1) + '= x(12 + 9*j +1 : 12 + 9*j +3);');
                    eval("state.flap_d" + num2str(j+1) + '= x(12 + 9*j +4 : 12 + 9*j +6);');
                    eval("state.inflow" + num2str(j+1) + '= x(12 + 9*j +7 : 12 + 9*j +9);');
                    j = j + 1;
                end
            end
        end

        function x = update_DisState(obj, trim_state, num, dis_step)
            %小扰动状态量
            x = repmat(trim_state, 1, num) + diag(ones(num,1)*dis_step);
        end

        function x = update_DisCtrl(obj, trim_map, num, dis_step)
            x = repmat(trim_map, 1, num) + diag(ones(num,1)*dis_step);
        end
    end
end