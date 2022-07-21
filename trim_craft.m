tic
% clc; clear all;
addpath('Analysis\','Component\Rotor\','Component\','Component\Fuse\','Component\Airfoil\','Math\','Para\')
run Para_CH47_our.m;
advance_ratio = 0.1;

%% demo for 'craft_trim'
%ctrl
ctrl.Ver = 5;%垂向
ctrl.lat = 0;%横向
ctrl.Lon = 0;%纵向
ctrl.dir = 0;%航向
ctrl.omega = 113.1;%旋翼转速

type = 'Craft';
%配平相关
solve_type = 'newton';
x0 = [6 0 0 0 0 0 0.1 0 0 0 0 0 0.025 0 0 0.1 0 0 0 0 0 0.025 0 0]';
%操稳相关
Cal_Sta_Map = true;
dis_step = 0.01;%小扰动步长

%初始化 Trim_Sta_Map
obj = Trim_Sta_Map(type, solve_type);
for i = 1:size(advance_ratio,2)
    %state
    state.pos = [0 0 0]';
    state.vel = [advance_ratio(i)*ctrl.omega*Para_rotor.R 0 0]';%地轴系速度
    state.pqr = [0 0 0]';%参考坐标系角速度
    state.angle(3) = 0;%偏航角
    %trim
    res_trim = obj.trimming(state, ctrl, envs, Para, x0);
    if res_trim.success
        result_trim(:,i) = res_trim.x;
        sprintf('前飞速度 %.2f m/s配平成功', state.vel(1));
        if Cal_Sta_Map
            res_AB = obj.Craft_Sta_Map(state, ctrl, envs, res_trim.x, dis_step); %操稳性
            result_A(:,:,i) = res_AB.A;
            result_B(:,:,i) = res_AB.B;
        end
    end
end
toc

% plot
for i = 1:6
    subplot(3,2,i); plot(advance_ratio, result_trim(i,:)); grid on;
end


