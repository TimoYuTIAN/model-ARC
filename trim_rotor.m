tic
clc; clear all;
addpath('Analysis\','Component\Rotor\','Math\','Para\')
run Para_CH47_our.m;

%% demo for 'Rotor_trim'
% ctrl
ctrl.theta0 = 6;%总距
ctrl.A = 0;%横向周期变距
ctrl.B = 0;%纵向周期变距
ctrl.omega = 113.1;%

% state
state.uvw = [0.1*ctrl.omega*Para_rotor.R 0 0]';%参考坐标系速度
state.pqr = [0 0 0]';%参考坐标系角速度

% Trim
Trim_type = 'Rotor';
solve_type = 'newton';

x0 = [0.1 0 0 0 0 0 0.025 0 0]';
obj = Trim_Sta_Map(Trim_type, solve_type);%三个挥舞速度，三个挥舞速度的导数，三个入流速度
res = obj.trimming(state, ctrl, envs, Para.Comp(1).para, x0);
toc



