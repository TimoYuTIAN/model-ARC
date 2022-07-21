classdef fuse < handle
    properties (SetAccess = private)
        name;
        type = 'Fuse';
        trans;
    end
    
    methods (Access = public)
        function obj = fuse(Para)
            obj.name = Para.name;
            euler.yaw = Para.euler(3);
            euler.pitch = Para.euler(2);
            euler.roll = Para.euler(1);
            obj.trans = Transform(Para.pos, euler);
        end

        function [uvw, pqr] = update_state(obj, state)
            uvw = Coordinate.Ve2b(state.uvw, obj.trans, state.pqr);
            pqr = obj.trans.DCMbe * state.pqr;
        end

        function result = FM(obj, state, ctrl, envs)
            %后面改进，此处最好写成外部定义函数

            %以下为我们自己的小纵列xflow计算得到的气动数据
            if(abs(state.uvw(1))<0.01) 
                state.uvw(1) = 0.01;
            end
            if (state.uvw(1)>0.0)
                alpha = atan(state.uvw(3)/state.uvw(1))*57.3;
                beta = atan(state.uvw(2)/sqrt(state.uvw(1)*state.uvw(1) + state.uvw(3)*state.uvw(3)))*57.3;
                Cx = -8E-05*alpha^2 - 0.0007*alpha - 0.0916;
                Cy = 0;
                if (beta<-2.0) 
                  Cy = -1E-05*alpha^2- 0.0014*alpha + 0.0568;
                end
                if (beta>2.0)
                  Cy = 1E-05*alpha^2 + 0.0016*alpha - 0.0568;
                end
                Cz = -2E-05*alpha^2 - 0.0062*alpha + 0.0097;
                X = 0.5*state.uvw(1)*state.uvw(1)*Cx;   
                Y = 0.5*state.uvw(2)*state.uvw(2)*Cy; 
                Z = 0.5*state.uvw(3)*state.uvw(3)*Cz;
            else
                state.uvw(1) = abs(state.uvw(1));
                alpha = atan(state.uvw(3)/state.uvw(1))*57.3;
                beta = atan(state.uvw(2)/sqrt(state.uvw(1)*state.uvw(1) + state.uvw(3)*state.uvw(3)))*57.3;
                Cx = -8E-05*alpha^2 - 0.0007*alpha - 0.0916;
                Cy = 0;
                if (beta<-2.0) 
                   Cy = -1E-05*alpha^2- 0.0014*alpha + 0.0568;
                end
                if (beta>2.0) 
                    Cy = 1E-05*alpha^2 + 0.0016*alpha - 0.0568;
                end
                Cz = -2E-05*alpha^2 - 0.0062*alpha + 0.0097;
                X = -0.5*state.uvw(1)*state.uvw(1)*Cx;   
                Y = 0.5*state.uvw(2)*state.uvw(2)*Cy;
                Z = 0.5*state.uvw(3)*state.uvw(3)*Cz;
            end
            
            %计算参考坐标系下的力和力矩
            FM_ref =[X; Y; Z; 0; 0; 0];
            result.FM_ref = Coordinate.FMb2e(FM_ref(1:3), obj.trans, FM_ref(4:6));
        end
    end
end