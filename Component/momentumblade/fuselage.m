classdef fuselage < handle
    properties (SetAccess = private)
        u; v; w; p; q; r;
        FM; 
    end
    
    methods (Access = public)
        function obj = fuselage(Para)

        end
        function FM = calculate_FM(obj, Input)
            State = Input(1:6);
            obj.u = State(1);
            obj.v = State(2);
            obj.w = State(3);
            obj.p = State(4);
            obj.q = State(5);
            obj.r = State(6);

            if(abs(obj.u)<0.01) 
                obj.u = 0.01;
            end
            if (obj.u>0.0)
                alpha = atan(obj.w/obj.u)*57.3;
                beta = atan(obj.v/sqrt(obj.u*obj.u + obj.w*obj.w))*57.3;
                Cx = -8E-05*alpha^2 - 0.0007*alpha - 0.0916;
                Cy = 0;
                if (beta<-2.0) 
                  Cy = -1E-05*alpha^2- 0.0014*alpha + 0.0568;
                end
                if (beta>2.0)
                  Cy = 1E-05*alpha^2 + 0.0016*alpha - 0.0568;
                end
                Cz = -2E-05*alpha^2 - 0.0062*alpha + 0.0097;
                X = 0.5*obj.u*obj.u*Cx;   
                Y = 0.5*obj.v*obj.v*Cy;
                Z = 0.5*obj.w*obj.w*Cz;
            else
                obj.u = abs(obj.u);
                alpha = atan(obj.w/obj.u)*57.3;
                beta = atan(obj.v/sqrt(obj.u*obj.u + obj.w*obj.w))*57.3;
                Cx = -8E-05*alpha^2 - 0.0007*alpha - 0.0916;
                Cy = 0;
                if (beta<-2.0) 
                   Cy = -1E-05*alpha^2- 0.0014*alpha + 0.0568;
                end
                if (beta>2.0) 
                    Cy = 1E-05*alpha^2 + 0.0016*alpha - 0.0568;
                end
                Cz = -2E-05*alpha^2 - 0.0062*alpha + 0.0097;
                X = -0.5*obj.u*obj.u*Cx;   
                Y = 0.5*obj.v*obj.v*Cy;
                Z = 0.5*obj.w*obj.w*Cz;
            end
            FM =[X; Y; Z; 0; 0; 0];
            obj.FM = FM;         
        end
    end
end