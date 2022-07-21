classdef rotor_flap_inflow < handle
    properties(SetAccess = private)
        e; K_beta;%挥舞铰偏置量;挥舞铰弹性系数
        Ib, Mb % 惯量 静矩
        flap_type;
        inflow_type;
    end
    methods(Access = public)
        function obj = rotor_flap_inflow(para)
            obj.flap_type = para.flap_type;
            obj.inflow_type = para.inflow_type;
            switch para.type
                case 'NoHinge'
                    obj.e = obj.R * 2 * (para.omega_beta^2 - 1) / (1 + 2 * para.omega_beta^2);
                    obj.K_beta = 0;
                case 'Hinge'
                    obj.e = para.e;%需定义
                    obj.K_beta = para.K_beta;%需定义
                otherwise
                    error("参数初始化类型有误");
            end
            obj.Mb = 0.5 * para.m * (para.R - obj.e)^2;%质量静矩
            obj.Ib = 1/3 * para.m * (para.R - obj.e)^3;%惯量
        end

        function flap_dd = dif_flap(obj, state, ctrl, envs)
            %挥舞方程
            omega = ctrl.omega; p = state.pqr(1); q = state.pqr(2);
            Mt = ctrl.Mt(1); Mts = ctrl.Mt(2); Mtc = ctrl.Mt(3);
            B0 = state.flap(1); Bs = state.flap(2); Bc = state.flap(3);
            B0_d = state.flap_d(1); Bs_d = state.flap_d(2); Bc_d = state.flap_d(3);

            %参考挥舞推导.pdf
            B0_dd = (Mt  - omega^2*(obj.Ib + obj.e*obj.Mb)*B0 - obj.K_beta*B0 - obj.Mb*envs.g) / obj.Ib;
            Bs_dd = (Mts - omega^2*obj.e*obj.Mb*Bs + 2*obj.Ib*omega*Bc_d...
                -2*(obj.Ib+obj.e*obj.Mb)*q*omega - obj.K_beta*Bs) / obj.Ib;
            Bc_dd = (Mtc - omega^2*obj.e*obj.Mb*Bc - 2*obj.Ib*omega*Bs_d...
                +2*(obj.Ib+obj.e*obj.Mb)*p*omega - obj.K_beta*Bc) / obj.Ib;

            flap_dd = [B0_d; Bs_d; Bc_d; B0_dd; Bs_dd; Bc_dd];
        end

        function inflow_d = dif_inflow(obj, state, ctrl, envs)
            %入流方程
            %侧滑角
            betas = atan2(state.uvw(2), state.uvw(1));

            %转换
            T = angle2dcm(0, 0, betas);
            C = T * [ctrl.Ct; ctrl.Cl; ctrl.Cm];
            v0sc = T * state.inflow;

            %等效诱导速度
            v = obj.equivalent_v(ctrl.Ct, state, ctrl);
            % 惯性矩阵
            M = diag([128/75 -16/45 -16/45]) / pi;
            [V, L] = obj.idv(v, state, ctrl);
            inflow_d = T' * (M \ (C - V * (L \ v0sc)));
        end

        function v = equivalent_v(obj, Ct, state, ctrl)
            uvw_nondim = state.uvw/ (obj.R * ctrl.omega) .* [1; 1; -1];
            f = @(x)(x - 0.5 * Ct / norm(uvw_nondim + [0; 0; x]));
            opt=optimset('Display','off');
            v = fsolve(f, 0.05, opt);
        end

        function [V_m, L] = idv(obj, v, state, ctrl)
            % 无量纲化
            nondim = 1 / (ctrl.omega * obj.R);
            V = state.uvw * nondim .* [1; 1; -1];
            v = v * [0; 0; 1];

            % 质量流量矩阵
            Vm_ = norm(V + v);
            V_ = (V + v)' * (V + 2 * v) / Vm_;
            V_m = diag([Vm_, V_, V_]);
            % 尾迹倾角
            alpha_w = atan2(v(3) + V(3), norm(V(1:2)));
            % 耦合增益矩阵
            X = sqrt((1 - sin(alpha_w)) / (1 + sin(alpha_w)));
            L = [0.5, 0, 15/64 * pi * X;
                0, -2 * (1 + X^2), 0;
                15/64 * pi * X, 0, 2 * (X^2 - 1)];
        end
    end
end