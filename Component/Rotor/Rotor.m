classdef Rotor < blade&rotor_flap_inflow
    %桨轴系 前右下
    properties (SetAccess = private)
        name;%旋翼代称
        type = 'MainRotor';
        nblade;%桨叶片数
        npsi; Psi;%方位角数，方位角数组        
        Rotate_dir;%旋转方向
        trans;
        result;
    end

    methods (Access = public)

        function obj = Rotor(para)
            obj = obj@blade(para.R, para.chords, para.twists,para.dihedrals, para.profiles);
            obj = obj@rotor_flap_inflow(para);
            obj.name = para.name;
            obj.Rotate_dir = para.RotorDir;
            obj.nblade = para.nBlade;
            euler.yaw = para.Tilt(3); euler.pitch = para.Tilt(2); euler.roll = para.Tilt(1);
            obj.trans = Transform(para.pos, euler);
        end

        function [uvw, pqr] = update_state(obj, state)
            %参考坐标系转到桨轴系
            uvw = Coordinate.Ve2b(state.uvw, obj.trans, state.pqr);
            pqr = obj.trans.DCMbe * state.pqr;
            if ~obj.Rotate_dir%顺时针旋翼,转到左手左边系
                uvw = uvw .* [1; -1; 1];
                pqr = pqr .* [-1; 1 ; -1];
            end
        end

        function result = FM(obj, state, ctrl, envs)  
            %方位角分段数 叶素采样点 
            obj.npsi = 12;
            obj.Psi = Data.index(linspace(0, 2 * pi, obj.npsi + 1), 1:obj.npsi);
            sample = 0.2:0.1:1;
            obj.GenBEs(sample);
            %桨轴系下的力和力矩
            res = obj.rotorFM(state, ctrl, envs);
            result = res;

            %入流方程需要用到
            nondim = 1/(envs.rho * (ctrl.omega*obj.R)^2 * pi * obj.R^2);
            result.Ct = -res.FM(3) * nondim;
            result.Cl = res.FM(4) * nondim / obj.R;
            result.Cm = res.FM(5) * nondim / obj.R;
            result.omega = ctrl.omega;

            %相对参考坐标系的力和力矩
            FM_ref = res.FM(1:6);
            FM_ref(4:5) = res.MGxy;
            if ~obj.Rotate_dir%顺时针旋翼
                FM_ref = FM_ref .* [1; -1; 1; -1; 1; -1];
            end
            result.FM_ref = Coordinate.FMb2e(FM_ref(1:3), obj.trans, FM_ref(4:6));
            obj.result = result;
        end

        function result = rotorFM(obj, state, ctrl, envs)
            B0 = state.flap(1); Bs = state.flap(2); Bc = state.flap(3);
            B0_d = state.flap_d(1); Bs_d = state.flap_d(2); Bc_d = state.flap_d(3);
            V0 = state.inflow(1); Vs = state.inflow(2); Vc = state.inflow(3);

            %旋翼转速
            omega = ctrl.omega;
            state.pqr = state.pqr + [0; 0; -ctrl.omega];            
            
            dFM = zeros(11, obj.npsi); % 6 + Mt Mts Mtc MGx MGy
            %方位角积分
            for i = 1:obj.npsi
                psi = obj.Psi(i); sp = sin(psi); cp = cos(psi);
                %挥舞角、挥舞角速度
                beta = B0 + Bs*sp + Bc*cp;
                beta_dot = B0_d + omega * (Bs*cp -Bc*sp) + Bc_d*cp + Bs_d*sp;
                %该方位角下的桨距
                theta_c = ctrl.theta0 - ctrl.A * cp + ctrl.B * sp;
                %该方位角下桨叶与桨轴系的夹角
                ctrl_p.yaw = 90 - rad2deg(psi);
                ctrl_p.pitch = rad2deg(beta);
                ctrl_p.roll = theta_c;
                DCM_no_roll = angle2dcm(pi - psi, beta, 0); % 用于计算dfp

                dfm = zeros(7, obj.nBEs); % 6 + mt
                %叶素段积分
                for j = 1:obj.nBEs
                    BE = obj.BEs(j);
                    v = omega * obj.R * V0 + Vs * omega * BE.radius * sp + Vc * omega * BE.radius * cp;
                    %桨轴系速度叠加入流速度及挥舞速度
                    state_p = state;
                    state_p.uvw(3) = state_p.uvw(3) - v - beta_dot*BE.radius;

                    res = BE.GetFM(state_p, ctrl_p, envs);
                    dft = -Data.index(DCM_no_roll * res.FM(1:3), 3); % 叶素对挥舞铰的作用力，向上为正
                    dmt = (BE.radius - obj.e) * dft;
                    dfm(:,j) = [res.FM; dmt];%桨轴系叶素微段的力和力矩
                end

                total_dfm = sum(dfm, 2);
                dMt = total_dfm(7);
                dMts = dMt * sp;
                dMtc = dMt * cp;
                dMGx = total_dfm(3) * sp * obj.e;
                dMGy = total_dfm(3) * cp * obj.e;
                dFM(:, i) = [total_dfm; dMts; dMtc; dMGx; dMGy]; % 桨轴系下单片桨叶FM
            end
            FM = sum(dFM, 2) / obj.npsi; % 桨轴系下单片桨叶方位角平均FM
            result.FM = FM(1:6) * obj.nblade; % 桨轴系合力(矩)
            result.Mt = [FM(7); FM(8) * 2; FM(9) * 2];
            % 桨毂力矩 = 弹簧力矩 + 离心力引起的 + 气动力引起的
            result.MGxy =  -0.5*obj.nblade * (obj.K_beta + obj.Mb * omega^2 * obj.e) * [Bs; Bc] + [FM(10:11)];
        end
    end
end