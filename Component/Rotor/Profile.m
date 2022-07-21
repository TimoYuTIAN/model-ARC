classdef Profile < handle
    properties(SetAccess = private)
        name = 'default';
        para;
        getCL; getCD;
    end
    methods
        function obj = Profile(name, para)
            if nargin > 1
                obj.name = name;
                obj.para = para;

                switch para.type
                    case 'constant'
                        if Data.ValidStruct(para, {'CL', 'CD'})
                            obj.getCL = @(q)para.CL;
                            obj.getCD = @(q)para.CD;
                        else
                            error('翼型输入数据不正确');
                        end
                    case 'interp1'
                        if Data.ValidStruct(para, {'CL', 'CD', 'alpCL', 'alpCD'})
                            obj.getCL = @(q)interp1(para.alpCL, para.CL, q, "pchip");
                            obj.getCD = @(q)interp1(para.alpCD, para.CD, q, "pchip");
                        else
                            error('翼型输入数据不正确');
                        end
                    case 'interp2'
                        if Data.ValidStruct(para, {'CL', 'CD', 'alpCL', 'alpCD', 'MaCL', 'MaCD'})
                            obj.getCL = @(q)interp2(para.alpCL, para.MaCL, para.CL, q(1), q(2), 'cubic');
                            obj.getCD = @(q)interp2(para.alpCD, para.MaCD, para.CD, q(1), q(2), 'cubic');
                        else
                            error('翼型输入数据不正确');
                        end
                    case 'UDF'
                        if Data.ValidStruct(para, {'getCL','getCD'})
                            obj.getCL = para.getCL;
                            obj.getCD = para.getCD;
                        else
                            error('翼型输入数据不正确');
                        end
                    otherwise
                        error('翼型类型不正确')
                end
            end
        end
        function CLCD = GetCLCD(obj, query)
            CLCD = [obj.getCL(query), obj.getCD(query)];
        end
    end
end