classdef Data

    methods (Static)

        function res = ValidStruct(struct, field)
            res = ~numel(find(isfield(struct, field) == 0));
        end

        function res = ValidSample(sample)
            res = (~numel(find(diff(sample) <= 0, 1))) && sample(end) == 1;
        end

        function ind = GetCeilInd(X, q)
            % X=[0.1 0.2 0.3 0.5 0.8 0.9]
            % q=0.4
            % ind=4
            ind = zeros(numel(q), 1);

            for i = 1:numel(q)
                ind(i) = find((X - q(i)) >= 0, 1);
            end

        end

        function result = index(data, ind)
            result = data(ind);
        end

    end

end
