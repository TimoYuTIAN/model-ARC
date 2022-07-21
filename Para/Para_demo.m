%%整机参数
Para_craft.m = 15.0;%整机重量
Para_craft.Ixx = 0.284;%整机惯量
Para_craft.Iyy = 2.065;
Para_craft.Izz = 2.083;
Para_craft.Ixy = 0.0;
Para_craft.Ixz = 0.0;
Para_craft.Iyz = 0.0;

%% 环境变量
envs.rho = 1.225;
envs.g = 9.8;

%% 旋翼参数
name = 'NACA0012';
% para.type = 'interp1';
% testData = load('NACA0012.mat');
% CL = testData.CL;
% CD = testData.CD;
% para.CL = CL(:, 2);
% para.CD = CD(:, 2);
% para.alpCL = CL(:, 1);
% para.alpCD = CD(:, 1);
para.type = "UDF";
% para.getCL = @(x)6 * x.alpha / 57.3;
% para.getCD = @(x)0.0087 - 0.0216 * x.alpha / 57.3 + 0.4 * (x.alpha / 57.3)^2;
para.getCL = @(x)naca0012c81(x.alpha, 1);
para.getCD = @(x)naca0012c81(x.alpha, 0);
NACA0012 = Profile(name, para);

Para_rotor.RotorDir = 1;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.nBlade = 2;%桨叶片数
Para_rotor.R = 0.9;%桨叶半径
Para_rotor.chords = [1; 0.067]';%桨叶弦长数组
Para_rotor.twists = [1; 0]';%初始桨距数组
Para_rotor.dihedrals =  [1 0];%上反角（初始锥度角）数据
Para_rotor.profiles = {1, NACA0012};%截面翼型数组

Para_rotor.type = 'Hinge';%旋翼类型
Para_rotor.omega_beta = 0;%挥舞一阶频率，旋翼类型为无铰式需定义，用来计算挥舞等效偏置量
Para_rotor.e = 0.02;%挥舞偏置，旋翼类型为有铰式需定义
Para_rotor.K_beta = 0;%桨毂弹簧力矩，旋翼类型为有铰式需定义，无铰式默认为0

Para_rotor.m = 0.265;%桨叶质量沿展向分布（kg/m）

Para_rotor.pos = [0; 0; 0];%旋翼相对参考坐标系的位置（m）
Para_rotor.Tilt = [0; 0; 0];%桨轴系相对参考坐标系的夹角数组（一般包含桨轴前倾角，deg）

Para_rotor.flap_type = 'default';%挥舞类型
Para_rotor.inflow_type = 'Pitt';%入流类型

%%
