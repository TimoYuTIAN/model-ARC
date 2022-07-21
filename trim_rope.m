%% 吊索配平
clc;
para.mc = 0.1;
para.l_c0 = 6;
para.Kc = 1000;
para.Cc = 100;
para.num = 10;
para.C_Dc = 0.1;
obj = rope(para);
% des
vel = [6.1074 0 0]';
des.xk_d = repmat(vel, 1, para.num);
des.xA_d = vel;
des.xB_d = vel;
des.FB = [0 0 -25]';
% envs
envs.rho = 1.225;
envs.g = 9.8;
envs.Vkgc = repmat([0.01 0.01 0.01]', 1, para.num);
% option
option = 'des_xB';
if option == 'des_xA'
    des.xA = [0 0 0]';
    x0 = [zeros(2, para.num+1);
    para.l_c0/para.num/2:para.l_c0/para.num:para.l_c0 para.l_c0] + 2;
elseif option == 'des_xB'
    des.xB = [0 0 0]';
    x0 = [zeros(2, para.num+1);
    [0 para.l_c0/para.num/2:para.l_c0/para.num:para.l_c0]-para.l_c0] - 0.1;
end

% trim
fun = @(x) obj.forTrim(envs,des,option,x);
options = optimoptions('fsolve','Display','iter','Algorithm','levenberg-marquardt');
x = fsolve(fun, x0, options)

% %计算仿真时吊索初始值
% x = x(:,para.num+1:-1:1);
% x4 = [zeros(3,1)
%     reshape(x, (para.num+1)*3, 1)];


% plot3(x(1,:), x(2,:), x(3,:)); hold on
% legend('1','2','3','4');

%% sim_rope 
% clc;clear
% para.l_c0 = 6;
% para.num = 10;
% R_Rope = 0.01;
% load x0_cable.mat
% sim("LoadRopeHeli")
