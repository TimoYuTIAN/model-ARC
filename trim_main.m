clc;clear;
run Para_CH47_our.m;
tandem = Craft(Para);
rope1 = rope(Para_cable);
global Vx_e; %
cable_pos = [zeros(2, Para_cable.num)
    [Para_cable.l_c0/Para_cable.num/2:Para_cable.l_c0/Para_cable.num:Para_cable.l_c0]-Para_cable.l_c0]-[0;0;2];
load x0.mat
advance_ratio = 0.0;
for i = 1:size(advance_ratio,2)
    Vx_e = advance_ratio(i)*113.1*Para_rotor.R
    cable_vel = [ones(1,Para_cable.num)*Vx_e;zeros(2, Para_cable.num)];

    % 单直升机配平     
    %     y0 = [Vx_e 0 0 0 0 0 0 0 0 0 0 0 ...
    %         0.1 0 0 0 0 0 0.025 0 0 0.1 0 0 0 0 0 0.025 0 0]';
    %     iy = [1,2,3,4,5,6,9]';
    %     x0 = y0; ix = [];
    %     u0 = [5,0,0,0]'; iu = [];
    %     dx0 = [zeros(9,1); Vx_e; zeros(20,1)]; idx = [1:30]';

    % 单吊索配平
    %     y0 = []';
    %     iy = []';
    %     x0 = [x(4:33)'; cable_vel(:); Vx_e; 0;0;zeros(3,1)]; ix = [64:66];
    %     u0 = [x(1:3)]'; iu = []';
    %     dx0 = [cable_vel(:); zeros(Para_cable.num*3,1); zeros(3,1); Vx_e; 0; 0]; idx = [1:size(dx0,1)]';

    % 单直升机带吊挂配平
    y0 = [Vx_e 0 0 0 0 0 0 0 0 x_cable(1:3) ...
        0.1 0 0 0 0 0 0.025 0 0 0.1 0 0 0 0 0 0.025 0 0]';
    iy = [1,2,3,4,5,6,9]';
    x0 = [x_heli(1:9); x_cable(1:3)'; x_heli(13:30); x_cable(4:33)'; cable_vel(:); Vx_e; 0;0;zeros(3,1)]; ix = [94:96];
    u0 = [u_heli']'; iu = [];
    dx0 = [zeros(9,1); Vx_e; zeros(20,1); cable_vel(:); zeros(Para_cable.num*3,1);zeros(3,1); Vx_e; 0; 0]; idx = [1:size(dx0,1)]';
    options(1)=1;
    [x,u,y,dx,options]=trim('our_ch47_rope_load',x0,u0,y0,ix,iu,iy,dx0,idx,options);
%     output(:,i) = [u; deg2rad([y(7:8)])];
end

x_cable = reshape(x(31:60),3,10);
x_cable = [x(10:12) x_cable x(94:96)];
x_heli = x(1:30);
u_heli = u(1:4);
save x0.mat x_cable u_heli x_heli

plot(-x_cable(1,:), -x_cable(3,:)); hold on
% save trim.mat x0 y0 u0