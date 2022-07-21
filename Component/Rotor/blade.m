classdef blade < handle
    %桨叶坐标系 x轴沿桨叶展向，y轴沿来流方向，z轴向下
    properties (SetAccess = private)
        R; 
        hub; 
        chords;%展长数组
        twists;%扭转角数组
        twist_type;%扭转方式 “线性”"定螺距"
        dihedrals;%上反角数组  
        profiles;%翼型数组 
        BEs;%叶素段
        nBEs;%叶素段数
    end
    methods
        function obj = blade(R, chords, twists, dihedrals, profiles)
            if nargin>0               
                obj.R = R;
                obj.chords = chords;
                obj.twists = twists;
                if size(obj.twists, 1) > 1
                    obj.twist_type='pitch';%扭转方式 “线性”"定螺距"
                end
                obj.dihedrals = dihedrals;
                obj.profiles = profiles;
            end
            
        end
        function BEs = GenBEs(obj, BESample)
            obj.hub =  BESample(1) * obj.R; 
            samplePoints = (BESample(1:end - 1)+BESample(2:end)) / 2;
            BEs(numel(samplePoints)) = BladeElement();
            drs = (BESample(2:end) - BESample(1:end - 1)) * obj.R;
            radius = samplePoints * obj.R;

            if size(obj.chords, 1) > 1
                chords_sample = interp1(obj.chords(:, 1), obj.chords(:, 2), samplePoints, 'pchip');
            else
                chords_sample = ones(1, numel(samplePoints)) * obj.chords(1, 2);
            end

            if size(obj.twists, 1) > 1
                if obj.twist_type =='pitch'
                    pitch = obj.twists(2);
                    twists_sample=obj.twists(1,2)+rad2deg(atan((pitch)./(2.0*pi*radius)));
                else
                twists_sample = interp1(obj.twists(:, 1), obj.twists(:, 2), samplePoints, 'pchip');
                end
            else
                pitch = obj.twists(2);
                twists_sample = rad2deg(atan(pitch ./ (2 * pi * radius)));
            end

            profiles_sample = obj.profiles{2}(Data.GetCeilInd(obj.profiles{1}, samplePoints));
            dihedrals_sample = obj.dihedrals(Data.GetCeilInd(obj.dihedrals(:, 1), samplePoints), 2);
            
            for i = 1:numel(samplePoints)

                euler.yaw = 90;
                euler.pitch = dihedrals_sample(i);
                euler.roll = twists_sample(i);
                pos = rotx(-dihedrals_sample(i)) * [0; radius(i); 0];
                trans = Transform(pos, euler);
                BEs(i) = BladeElement(trans, drs(i), chords_sample(i), radius(i), samplePoints(i), profiles_sample(i));
            end

            obj.BEs = BEs;
            obj.nBEs = numel(BEs);
        end
    end
end