classdef tandem_Mnplt < handle
    methods
        function obj = tandem_Mnplt()
            
        end
        function output=allocate(~,ctrl)
            ctrl.Ver = ctrl.u1; ctrl.lat = ctrl.u2;
            ctrl.Lon = ctrl.u3; ctrl.dir = ctrl.u4;
            output=ctrl;
        end
    end
end