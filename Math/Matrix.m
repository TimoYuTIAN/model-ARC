classdef Matrix
    methods (Static)
        function out = skew(vector)
            %产生3维向量的反对称矩阵
            out = [0 -vector(3) vector(2)
                vector(3) 0 -vector(1)
                -vector(2) vector(1) 0];
        end

        function out = Cal_norm2_row(matrix)
            num = size(matrix, 1);
            out = zeros(num, 1);
            for i = 1:num
                out(i) = norm(matrix(i,:));
            end
        end

        function out = Cal_norm2_Col(matrix)
            num = size(matrix, 2);
            out = zeros(num, 1);
            for i = 1:num
                out(i) = norm(matrix(:,i));
            end
        end


    end
end
