classdef HFM < handle
    properties (SetAccess = private)
        RotorF; RotorR; fuse;
        Input0; FM0; 
    end
    methods (Access = public)
        function obj = HFM(Para)
            method.RotorDir = Para.index_RotorF; method.RotorPos = Para.Pos_RotorF;
            obj.RotorF = rotor_momentum_blade(Para, method);

            method.RotorDir = Para.index_RotorR; method.RotorPos = Para.Pos_RotorR;
            obj.RotorR = rotor_momentum_blade(Para, method);

            obj.fuse = fuselage(Para);           
        end

        function FM = calculate_FM(obj, Input) 
            obj.Input0 = Input;
            State = Input(1:6);
            [Map_ver, Map_lat, Map_lon, Map_dir] = deal(Input(7), Input(8), Input(9), Input(10));
            
            %Decoupling             
            Map_rotorF = [Map_ver+Map_lon; -Map_lat-Map_dir; 0];  
            Map_rotorR = [Map_ver-Map_lon; Map_lat-Map_dir; 0]; 

            obj.FM0 = obj.RotorF.calculate_FM([State; Map_rotorF])+...
                obj.RotorR.calculate_FM([State; Map_rotorR])+...
                obj.fuse.calculate_FM(State);
            FM = obj.FM0;
        end
    end
end