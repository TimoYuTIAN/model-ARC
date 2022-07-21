clc;clear
Init = struct( 'type', 'mass_point', ...
    'mass', 2.5, ...  %重量2.5 kg
    'AeroCoe', [1 0 0]', ... %吊挂物气动系数
    'NPoints', 1, ... %吊挂点数目，列数应与吊挂点数一致
    'PosPoints', [0 0 0]', ...%吊挂点位置
    'LWH', [0.5 0.2 0.2]', ... %吊挂物长宽高
    'posAero', [0 0 0]' ... 气动中心相对重心的位置
    );

envs.rho = 1.225;
envs.g = 9.8;
envs.v_e = [0 0 0 0 0 0]';

state.uvw = [10 0 0]';

ctrl.Fc = [0 0 0]'; 
ctrl.index = [1];

load0 = Load(Init);
load0.get_dif(state,ctrl,envs)
