classdef six_dof < handle
    %state: 欧拉角 体轴系速度 角速度 欧拉角 地轴系速度 
    properties(SetAccess = private)
        m;%整机重量
        J;%惯性矩阵
        tran;
    end
    methods
        function obj = six_dof(para)
            if nargin > 0
                obj.m = para.m;
                
                %若Jyx Jzx Jzy不存在
                if ~isfield(para, 'Jyx') || ~isfield(para, 'Jzx') || ~isfield(para, 'Jzy')
                    [para.Jyx, para.Jzx, para.Jzy] = deal(para.Jxy, para.Jxz, para.Jyz);
                end
                obj.J = [para.Jxx  -para.Jxy  -para.Jxz
                    -para.Jyx   para.Jyy  -para.Jyz
                    -para.Jzx  -para.Jzy   para.Jzz];
            else
                error('六自由度方程初始化输入参数不足');
            end
        end

        function update_trans(obj, state)
            euler.yaw = rad2deg(state.angle(3));
            euler.pitch = rad2deg(state.angle(2));
            euler.roll = rad2deg(state.angle(1));
            obj.tran = Transform([0,0,0], euler);
        end

        function x_d = six_dif(obj, state, ctrl, envs)
            % M * x_d + Cx = g + FM
            M = blkdiag(obj.m*eye(3), obj.J, eye(3), eye(3));
            Cx = [cross(state.pqr,obj.m*state.uvw)
                cross(state.pqr, obj.J*state.pqr)
                -obj.tran.DCM_pqr2omega * state.pqr
                -obj.tran.DCMeb * state.uvw];
            g = [obj.tran.DCMbe * [0;0;obj.m*envs.g]
                zeros(3,1)
                zeros(3,1)
                zeros(3,1)];
            FM = [ctrl; zeros(6,1)];
            x_d = (M)\(g + FM - Cx);
        end

    end
end