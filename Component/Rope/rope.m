classdef rope < handle
    %假定整个吊索质量集中于num个质点
    properties(SetAccess = private)
        mc;%吊索质量 
        l_c0%吊索初始长度
        Kc;%弹簧刚度系数
        Cc;%弹簧阻尼系数
        num;%吊索分段数
        m_kc;%每个质点质量
        l_kc0;%每段吊索长度
        C_Dc%吊索单位长度气动阻力系数
    end
    methods
        function obj = rope(para)
            obj.mc = para.mc;
            obj.l_c0 = para.l_c0;
            obj.Kc = para.Kc;
            obj.Cc = para.Cc;
            obj.num = para.num;
            [obj.m_kc, obj.l_kc0] = obj.InitNRope;
            obj.C_Dc = para.C_Dc;
        end
        
        function [m_kc, l_kc0] = InitNRope(obj)
            %初始化各个质点质量
            m_kc = obj.mc/obj.num * ones(obj.num, 1);
            %初始化每段吊索长度
            l_kc0 = [obj.l_c0/obj.num/2
                obj.l_c0/obj.num * ones(obj.num-1 , 1)
                obj.l_c0/obj.num/2];
        end

        function [dif, FAB] =  three_dom(obj, state, envs)
            xk = [state.xA state.xk state.xB]; 
            xk_d = [state.xA_d state.xk_d state.xB_d];                    
            %每段吊索在地轴系下的投影向量及其导数
            Rkc = xk(:, 2:obj.num+2) - xk(:, 1:obj.num+1);
            Rkc_d = xk_d(:, 2:obj.num+2) - xk_d(:, 1:obj.num+1);
            %计算每段吊索长度和拉伸率
            lkc = Matrix.Cal_norm2_Col(Rkc);
            lkc_d = zeros(obj.num+1, 1);
            for i = 1:obj.num+1
                lkc_d(i) = 1/lkc(i) * Rkc_d(:,i)' * Rkc(:,i);
            end
            %计算吊索张力
            x = lkc - obj.l_kc0; x(x < 0) = 0;
            x_d = lkc_d; 
            x_d(x_d<0) = 0; x_d(find(isnan(x_d)==1)) = 0;
            Fkc = obj.Kc.*x + obj.Cc.*x_d;
            %计算气动阻力
            len = [lkc(1)+lkc(2)/2; 
                lkc(2:obj.num-1)/2 + lkc(3:obj.num)/2
                lkc(obj.num)/2+lkc(obj.num+1)];
            Dka = 0.5 * envs.rho * obj.C_Dc * len .*...
                Matrix.Cal_norm2_Col(state.xk_d - envs.Vkgc) .* Matrix.Cal_norm2_Col(state.xk_d - envs.Vkgc);
            %计算x_dd
            dif = zeros(obj.num*3,1);
            for i = 1:obj.num
                %吊索i+1引起的加速度
                if isequal(lkc(i+1),0)
                    xk_dd1 = zeros(3,1);
                else
                    xk_dd1 = Fkc(i+1)/obj.m_kc(i)/lkc(i+1)*Rkc(:,i+1);
                end
                %吊索i引起的加速度
                if isequal(lkc(i),0)
                    xk_dd2 = zeros(3,1);
                else
                    xk_dd2 = -Fkc(i)/obj.m_kc(i)/lkc(i)*Rkc(:,i);
                end
                %气动力引起的加速度
                if isequal(norm(state.xk_d(:,i) - envs.Vkgc(:,i)), 0)
                    xk_dd3 = zeros(3,1);
                else
                    xk_dd3 = -Dka(i)/obj.m_kc(i)/norm(state.xk_d(:,i) - envs.Vkgc(:,i))*(state.xk_d(:,i) - envs.Vkgc(:,i));
                end
                
                xk_dd = xk_dd1 + xk_dd2 + xk_dd3 + [0 0 envs.g]';
                dif(i*3-2:i*3) = xk_dd;
            end
            FAB.A = Fkc(1)/lkc(1)*Rkc(:,1); FAB.A(find(isnan(FAB.A)==1)) = 0;
            FAB.B = -Fkc(obj.num+1)/lkc(obj.num+1)*Rkc(:,obj.num+1);
            FAB.B(find(isnan(FAB.B)==1)) = 0;
        end

        function out = forTrim(obj, envs, des, option, x)
            switch option
                case 'des_xA'
                    state.xA = des.xA;
                    state.xB = reshape(x(obj.num*3+1 : obj.num*3+3),3, 1);
                    state.xk = reshape(x(1 : obj.num*3), 3, obj.num);
                case 'des_xB'
                    state.xB = des.xB;
                    state.xA = reshape(x(1 : 3), 3, 1);
                    state.xk = reshape(x(4 : obj.num*3+3), 3, obj.num);
                otherwise
                    error('不支持的绳索配平类型');
            end
            state.xk_d = des.xk_d;
            state.xA_d = des.xA_d;
            state.xB_d = des.xB_d;
            [dif, FAB] = obj.three_dom(state, envs);
            out = [ FAB.B - des.FB; dif];%des.FB为吊挂物需要吊索提供的力
        end

        function out = forSim(obj, envs, x)
            state.xk = reshape(x(1 : obj.num*3), 3, obj.num);%num个质点在地轴系下的位置
            state.xk_d = reshape(x(obj.num*3+1 : obj.num*6), 3, obj.num);%num个质点在地轴系下的速度
            state.xA = reshape(x(obj.num*6 + 1 : obj.num*6+3), 3, 1);%直升机端吊挂点对地位置和速度
            state.xA_d = reshape(x(obj.num*6+4 : obj.num*6+6), 3, 1);
            state.xB = reshape(x(obj.num*6+7 : obj.num*6+9), 3, 1);%吊挂物端吊挂点对地位置和速度
            state.xB_d = reshape(x(obj.num*6+10 : obj.num*6+12), 3, 1);
            [dif, FAB] = obj.three_dom(state, envs);
            out = [dif; FAB.A; FAB.B];
        end
    end
end