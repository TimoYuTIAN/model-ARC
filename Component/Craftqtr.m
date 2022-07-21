classdef Craftqtr < six_dof
    properties(SetAccess = private)
        name;%
        ncomp; comp;%部件个数；部件数组
        nMainRotor;%主旋翼个数
        sixdif;%六自由度方程
    end
    methods(Access = public)
        function obj = Craftqtr(Input)
            %初始化飞行器整体参数
            obj = obj@six_dof(Input.Craft);

            %初始化飞行器代称
            obj.name = Input.name;

            %初始化各部件参数
            obj.ncomp = numel(Input.Comp);
            j = 0;
            for i = 1:obj.ncomp
                switch Input.Comp(i).type
                    case 'MainRotor'
                        eval("obj.comp.comp"+ num2str(i) + "= Rotor(Input.Comp(i).para);");
                        j = j+1;
                    case 'TailRotor'

                    case 'Fuse'
                        eval("obj.comp.comp"+ num2str(i) + "= fuse(Input.Comp(i).para);");
                    case 'Wing'
                        eval("obj.comp.comp"+ num2str(i) + "= fuse(Input.Comp(i).para);");
                    case 'Propeller'

                    otherwise
                        error('不支持此种部件类型');
                end
            end
            obj.nMainRotor = j;
        end

        function ctrl_comp = update_ctrl(obj, state, ctrl, envs)
            for i = 1:obj.ncomp

                %前旋翼操纵量
                MapRotor1.theta0 = ctrl.Ver + ctrl.Lon;
                MapRotor1.A = ctrl.lat + ctrl.dir;
                MapRotor1.B = 0;
                MapRotor1.omega = ctrl.omega;

                %后旋翼操纵量
                MapRotor2.theta0 = ctrl.Ver - ctrl.Lon;
                MapRotor2.A = -ctrl.lat + ctrl.dir;
                MapRotor2.B = 0;
                MapRotor2.omega = ctrl.omega;

                Map_fuse = [];
            end
            ctrl_comp.ctrl1 = MapRotor1; ctrl_comp.ctrl2 = MapRotor2;
            ctrl_comp.ctrl3 = Map_fuse;
        end

        function state_comp = update_state(obj, state)
            j = 1;
            for i = 1:obj.ncomp
                [state.uvw, state.pqr] = eval("obj.comp.comp" + num2str(i) + ".update_state(state)");
                eval("state_comp.state" + num2str(i) + "= state;");
                if(eval("strcmp(obj.comp.comp"+ num2str(i) + ".type, 'MainRotor')"))
                    eval("state_comp.state" + num2str(i) + ".flap = state.flap" + num2str(j) + ";");
                    eval("state_comp.state" + num2str(i) + ".flap_d = state.flap_d" + num2str(j) + ";");
                    eval("state_comp.state" + num2str(i) + ".inflow = state.inflow" + num2str(j) + ";");
                    j = j + 1;
                end
            end
        end

        function result = CalFM (obj, state_comp, ctrl_comp, envs)
            %合力、合力矩用于六自由度方程; 旋翼相关力，用于挥舞入流
            HFM_ref = zeros(6,1);
            for i = 1:obj.ncomp
                %计算各部件气动力
                eval("result.FM" + num2str(i) + " = obj.comp.comp" + num2str(i) + ".FM(state_comp.state" ...
                    + num2str(i) + ", ctrl_comp.ctrl" + num2str(i) + ", envs);");
                %相对机体坐标系的合力、合力矩
                HFM_ref = HFM_ref + eval("result.FM" + num2str(i) + ".FM_ref");
            end
            result.HFM_ref = HFM_ref;
        end



        function dif = get_dif(obj, state, ctrl, envs, type)
            state_comp = obj.update_state(state);
            ctrl_comp = obj.update_ctrl(state, ctrl, envs);
            res = obj.CalFM(state_comp, ctrl_comp, envs);
            res.HFM_ref = res.HFM_ref + ctrl.FM_extra;

            dif = zeros(12 + 9 * obj.nMainRotor, 1);
            if strcmp(type, 'trim')
                dif(1:12) = obj.six_dif(state, res.HFM_ref, envs) - ...
                    [ zeros(6,1); obj.tran.DCM_pqr2omega * state.pqr; state.vel;];
            else
                dif(1:12) = obj.six_dif(state, res.HFM_ref, envs);
            end

            j = 0;
            for i = 1:obj.ncomp
                if(eval("strcmp(obj.comp.comp"+ num2str(i) + ".type, 'MainRotor')"))
                    dif(12 + 9*j +1 : 12 + 9*j + 9) = ...
                        [ eval("obj.comp.comp" + num2str(i) + ".dif_flap(state_comp.state" + num2str(i) + ", res.FM" + num2str(i) + ", envs)");
                        eval("obj.comp.comp" + num2str(i) + ".dif_inflow(state_comp.state" + num2str(i) + ", res.FM" + num2str(i) + ", envs)")];
                    j = j+1;
                end
            end
        end

        function dif = get_dif_sim(obj, x, envs)
            state.uvw = x(1:3);
            state.pqr = x(4:6);
            state.angle = x(7:9);
            state.pos = x(10:12);
            state.flap1 = x(13:15);
            state.flap_d1 =  x(16:18);
            state.inflow1 = x(19:21);
            state.flap2 = x(22:24);
            state.flap_d2 = x(25:27);
            state.inflow2 = x(28:30);

            ctrl.Ver = x(31); ctrl.lat = x(32);
            ctrl.Lon = x(33); ctrl.dir = x(34);
            ctrl.omega = x(35);

            obj.update_trans(state);

            ctrl.FM_extra = [obj.tran.DCMbe*x(36:38); 0;0;0];
            dif = obj.get_dif(state, ctrl, envs, 'sim');


%                 %计算FM
%                 state_comp = obj.update_state(state);
%                 ctrl_comp = obj.update_ctrl(state, ctrl, envs);
%                 res = obj.CalFM(state_comp, ctrl_comp, envs);
%                 FM(1:3) =  res.HFM_ref(1:3);
%                 FM(4:6) =  res.HFM_ref(4:6);
%                 dif = [dif; FM'];

        end

    end
end