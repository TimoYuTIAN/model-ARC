classdef Math

    methods (Static)

        function res = Newton(fun0, x0, options)

            if nargin < 3
                options = Math.options();
            end

            fun = @(x)Data.index(fun0(x), ':');

            switch options.loss
                case 'MSE'
                    loss = @(x)sum(x.^2);
                case "MaxError"
                    loss = @(x)max(abs(x));
            end

            N = 0;
            x = x0(:);

            while true
                f = fun(x);
                residual = loss(f);
                N = N + 1;

                if residual < options.residual
                    res.success = true;
                    res.x = x;
                    res.N = N;
                    res.f = f;
                    res.residual = residual;
                    break
                elseif N >= options.Nmax
                    res.success = false;
                    res.x = x;
                    res.N = N;
                    res.f = f;
                    res.residual = residual;
                    warning("算法未收敛")
                    break
                else
                    dx = -Math.Jacobian(fun, x, 1e-4,f) \ f;

                    if options.convergence

                        for i = 1:5
                            xx = x + dx;

                            if loss(fun(xx)) < residual
                                break
                            else
                                dx = 0.5 * dx;
                            end

                        end

                    end

                    x = x + dx;
                end

            end

        end

        function Jac = Jacobian(fun, x, delta, y0)

            if nargin < 4
                y0 = fun(x);
            end

            Jac = zeros(numel(y0), numel(x));

            for i = 1:numel(x)
                xx = x;
                xx(i) = xx(i) + delta;
                Jac(:, i) = fun(xx) - y0;
            end

            Jac = Jac / delta;

        end

        function opt = options(Nmax, residual, loss, method, convergence)

            opt = struct('Nmax', 100, 'residual', 1e-8, 'method', 'default', 'loss', 'MSE', 'convergence', false);

            if nargin > 0
                opt.Nmax = Nmax;
            end

            if nargin > 1
                opt.residual = residual;
            end

            if nargin > 2
                opt.loss = loss;
            end

            if nargin > 3
                opt.method = method;
            end

            if nargin > 4
                opt.convergence = convergence;
            end

        end

    end

end
