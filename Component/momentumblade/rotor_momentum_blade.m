classdef rotor_momentum_blade < handle
    %Momentum blade theory
    properties (SetAccess = private)
        rho;
        R; b; k; omega; e %Radius; Chord; Blade number; Rotational speed; Hinge offset
        a00; alpha00; %Slope of the lift coefficient vs attack angle; Zero-lift angle
        m; Ib; Mb; %Unit mass of the blade in the radial direction; Inertia moment; Mass moment
        x; y; z; %rotor position relative to the gravity center in the body axis
        u; v; w; p; q; r;
        theta0; A1; B1; A1s; B1s;
        u1; v1; w1; wxr; wyr;
        miu; lamda0; alphas; betaw;
        lamda_b; sigma;
        Deg2Rad = 180/pi;
        theta1;
        v_0; v_00;
        beta0; a1; b1;
        xt; xs; rstep; r1; psi;
        deltw; index;
        FM;
    end
    
    methods (Access = public)
        
        function obj = rotor_momentum_blade(Para,method)            

            obj.x = method.RotorPos(1);
            obj.y = method.RotorPos(2);
            obj.z = method.RotorPos(3);
            obj.index = method.RotorDir;

            obj.R = Para.R;
            obj.b = Para.b;
            obj.k = Para.k;
            obj.omega = Para.omega;
            obj.e = Para.e;
            obj.a00 = Para.a00;
            obj.alpha00 = Para.alpha00 / obj.Deg2Rad;
            obj.m = Para.m;
            obj.Ib = Para.Ib;
            obj.Mb = Para.Mb;
            obj.rho = Para.rho;
            obj.theta1 = Para.theta1 / obj.Deg2Rad;
            obj.deltw = Para.deltw / obj.Deg2Rad;
            obj.sigma  = Para.sigma;
            obj.lamda_b = Para.lamda_b;          
            
            obj.xt = obj.R;
            obj.xs = 0.1 * obj.R;
            obj.rstep = (obj.xt - obj.xs)/20;
            obj.r1 = obj.xs : obj.rstep : obj.xt - obj.rstep;
            obj.psi = linspace(0,2*pi-pi/6,12);
            
        end
        
        function FM = calculate_FM(obj, Input)
            
            State = Input(1:6);
            Map = Input(7:9);
            obj.u = State(1);
            obj.v = State(2);
            obj.w = State(3);
            obj.p = State(4);
            obj.q = State(5);
            obj.r = State(6);
            obj.theta0 = Map(1) / obj.Deg2Rad;
            obj.A1 = Map(2) / obj.Deg2Rad;
            obj.B1 = Map(3) / obj.Deg2Rad;

            obj.update_velocity();
            obj.induce_v();
            obj.update_flap();
            
            sp = sin(obj.psi);
            cp = cos(obj.psi);
            beta = obj.beta0 - obj.a1*cp - obj.b1*sp;
            beta_d = obj.omega * (obj.a1*sp - obj.b1*cp);
            
            FM = obj.Blade_force( sp, cp, beta, beta_d);
            obj.FM = FM;
        end
        
        function FM = Blade_force(obj, sp, cp, beta, beta_d)
            theta=repmat(obj.theta0+(obj.r1/obj.R-0.7)*obj.theta1,size(beta,2),1)- ...
                repmat(obj.A1s*cp'+obj.B1s*sp', 1, size(obj.r1, 2));
            ut=obj.omega*obj.R*(repmat(obj.r1/obj.R,size(beta,2),1))+repmat(obj.u1*sp' + obj.v1*cp',1, size(obj.r1, 2));
            up= -(repmat(obj.omega*obj.R*(obj.lamda0-obj.miu*beta'.*cp'), 1, size(obj.r1, 2))-...
                obj.v_0*obj.omega*obj.R*(1+cp'*obj.r1/obj.R)-...
                beta_d'* obj.r1 - repmat(obj.v1*beta'.*sp', 1, size(obj.r1, 2)) - ...
                sp'*obj.r1*obj.omega*obj.wxr-cp'*obj.r1*obj.omega*obj.wyr);
            alpha=atan(up./ut);
            alpha11=theta-alpha- obj.alpha00;
            CL=obj.a00*alpha11;
            Cd=0.008-0.003*CL+0.01*CL.*CL;
            q2=0.5*obj.rho*(up.^2+ut.^2);
            Dy=q2.*CL*obj.b;
            Dx=q2.*Cd*obj.b;
            DT=Dy.*cos(alpha)-Dx.*sin(alpha);
            DQ=Dx.*cos(alpha)+Dy.*sin(alpha);
            DTs=(DT.*repmat(cos(beta'),1,size(obj.r1,2))) * obj.rstep;
            DHs=(DQ.*repmat(sp',1,size(obj.r1,2))-DT.*repmat(sin(beta').*cp',1,size(obj.r1,2))) * obj.rstep;
            DSs=(-DQ.*repmat(cp',1,size(obj.r1,2))-DT.*repmat(sin(beta').*sp',1,size(obj.r1,2))) * obj.rstep;
            DM=DQ.*repmat(obj.r1,size(beta,2),1) * obj.rstep;
            
            quant = size(beta,2);
            Ts = sum(sum(DTs))/quant * obj.k; 
            Hs = sum(sum(DHs))/quant * obj.k; 
            Ss = sum(sum(DSs))/quant * obj.k; 
            Mz = sum(sum(DM))/quant * obj.k;
            
            Mx=0.5*obj.k*obj.e*obj.Mb*obj.omega^2*obj.b1;
            My=0.5*obj.k*obj.e*obj.Mb*obj.omega^2*obj.a1;            
            %Mx = 0; My = 0;
            Hs =  Hs*cos(obj.betaw) + Ss*sin(obj.betaw);
            Ss = -Hs*sin(obj.betaw) + Ss*cos(obj.betaw);    
            Trzb = [cos(obj.deltw) -sin(obj.deltw) 0
                0 0 1
                sin(obj.deltw) cos(obj.deltw) 0];
            F0 = Trzb * [-Hs; -Ts; obj.index * Ss];
            
            pos = [obj.x; obj.y; obj.z];
            M0 = Trzb * [obj.index * Mx; obj.index*Mz; My] + cross(pos, F0); 
            FM = [F0;M0];                      
        end
        
        function update_flap(obj)
            obj.beta0=obj.lamda_b/6*(3/4*obj.theta0*(1+obj.miu^2)+3/5*obj.theta1*(1+5/6*obj.miu^2)+...
                obj.miu*(obj.alphas-obj.B1s)-obj.v_0)-1.5*9.8*obj.R/((obj.omega*obj.R)^2);
            a1w=(-(16/obj.lamda_b)*(obj.q/obj.omega)+obj.index * obj.p/obj.omega)/(1-0.5*obj.miu^2)+...
                12/obj.lamda_b*obj.e/obj.R*(-16*obj.index * obj.p/obj.lamda_b/obj.omega/(1-obj.e/obj.R)^2-...
                obj.q/obj.omega)/(1-obj.e/obj.R)^3/(1-0.25*obj.miu^4);
            obj.a1=2*obj.miu/(1-0.5*obj.miu^2)*(4/3*obj.theta0+obj.theta1+obj.miu*obj.alphas-obj.v_0)-...
                (1+1.5*obj.miu^2)*obj.B1s/(1-0.5*obj.miu^2)+a1w;
            b1w=(-(16/obj.lamda_b)*(obj.index * obj.p/obj.omega)-obj.q/obj.omega)/(1+0.5*obj.miu^2)-12/obj.lamda_b*obj.e/obj.R*...
                (-16*obj.q/obj.lamda_b/obj.omega/(1-obj.e/obj.R)^2+obj.index * obj.p/obj.omega)/(1-obj.e/obj.R)^3/(1-0.25*obj.miu^4);
            obj.b1=1/(1+0.5*obj.miu^2)*(4/3*obj.miu*obj.beta0+obj.v_0)+obj.A1s+b1w;
        end
        
        function induce_v(obj)
%                         f_induce_v = @(x) x - ( 0.92 * obj.a00 * obj.sigma * (obj.theta0 * (1/3 + obj.miu^2/2) + ...
%                             obj.theta1*(1+obj.miu^2)/4 - (x - obj.lamda0)/2 - obj.miu * obj.B1s/2) ) / ...
%                             (4 * sqrt(obj.miu^2 + (x - obj.lamda0)^2));
%                         opt=optimset('Display','off');
%                         obj.v_0 = fsolve(f_induce_v, -0.01,opt);
            vO0 = 0.2;
            vOK=0.001;
            vOT = 0.1;
            FvOT = vOT - ( 0.92 * obj.a00 * obj.sigma * (obj.theta0 * (1/3 + obj.miu^2/2) + ...
                obj.theta1*(1+obj.miu^2)/4 - (vOT - obj.lamda0)/2 - obj.miu * obj.B1s/2) ) / ...
                (4 * sqrt(obj.miu^2 + (vOT - obj.lamda0)^2));
            i=0;
            while abs(FvOT)>0.000001
                if i~=0
                    if FvOT>0
                        vOK=vOT;
                    else
                        vO0=vOT;
                    end
                end
                i=1;
                FvO0 = vO0 - ( 0.92 * obj.a00 * obj.sigma * (obj.theta0 * (1/3 + obj.miu^2/2) + ...
                obj.theta1*(1+obj.miu^2)/4 - (vO0 - obj.lamda0)/2 - obj.miu * obj.B1s/2) ) / ...
                (4 * sqrt(obj.miu^2 + (vO0 - obj.lamda0)^2));
                FvOK = vOK - ( 0.92 * obj.a00 * obj.sigma * (obj.theta0 * (1/3 + obj.miu^2/2) + ...
                obj.theta1*(1+obj.miu^2)/4 - (vOK - obj.lamda0)/2 - obj.miu * obj.B1s/2) ) / ...
                (4 * sqrt(obj.miu^2 + (vOK - obj.lamda0)^2));
                vOT=vOK-FvOK*(vOK-vO0)/(FvOK- FvO0);
                FvOT = vOT - ( 0.92 * obj.a00 * obj.sigma * (obj.theta0 * (1/3 + obj.miu^2/2) + ...
                obj.theta1*(1+obj.miu^2)/4 - (vOT - obj.lamda0)/2 - obj.miu * obj.B1s/2) ) / ...
                (4 * sqrt(obj.miu^2 + (vOT - obj.lamda0)^2));
            end
            obj.v_0 = vOT;
            obj.v_00 = obj.v_0 * obj.omega * obj.R;
        end
        
        function update_velocity(obj)
            pos = [obj.x; obj.y; obj.z];
            pqr = [obj.p; obj.q; obj.r];
            uvw_rotor = [obj.u; obj.v; obj.w] - cross(pos, pqr);
            obj.u1 = uvw_rotor(1); obj.v1 = uvw_rotor(2); obj.w1 = uvw_rotor(3);
            obj.miu = norm(uvw_rotor(1:2)) / obj.omega / obj.R;
            obj.lamda0 = obj.w1 / obj.omega / obj.R;
            obj.alphas = atan2(obj.w1, norm(uvw_rotor(1:2)));
            obj.betaw = atan2(obj.v1, norm(uvw_rotor([1,3])));
            obj.A1s = obj.A1*cos(obj.betaw) - obj.B1*sin(obj.betaw);
            obj.B1s = obj.A1*sin(obj.betaw) + obj.B1*cos(obj.betaw);
            obj.wxr = obj.p / obj.omega * cos(obj.betaw) + obj.q / obj.omega * sin(obj.betaw);
            obj.wyr = obj.q / obj.omega * cos(obj.betaw) - obj.p / obj.omega * sin(obj.betaw);
        end  
        
    end
end