classdef cal_mse
    methods (Static)
        function out = mse_(in,n)
            %求均方误差
            %n=1表示按列 n=2表示按行
            if nargin>1
                n=n;
            else
                n = 1;
            end
            
            if n == 1
                for i = 1:size(in,2)
                   out(:,i) = norm(in(:,i) - mean(in(:,i)))^2/size(in,1); 
                end
            else
               for i = 1:size(in,1)
                   out(i,:) = norm(in(i,:) - mean(in(i,:)))^2/size(in,2); 
                end
            end
        end
    end
end
