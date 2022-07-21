classdef Coordinate
    %COORDINATE 此处显示有关此类的摘要
    %   此处显示详细说明
    
    
    methods (Static)
        function FM=FMb2e(Fb,transform,Mb)
            if nargin<3
                M=[0;0;0];
            else
                M=transform.DCMeb*Mb;
            end 
            Fe=transform.DCMeb*Fb;
            Me=cross(transform.pos,Fe); % 力矩计算，力臂(OF)X力
            FM=[Fe;Me+M];            
        end

        function V=Ve2b(V,transform,omega)
            if nargin<3
                Vq=[0;0;0];
            else
                Vq=cross(omega,transform.pos);
            end
            V=transform.DCMbe*(V+Vq);
        end
    end
end

