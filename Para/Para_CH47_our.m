%%整机参数
craft.m = 15.0;%整机重量
craft.Jxx = 0.284;%整机惯量
craft.Jyy = 2.065;
craft.Jzz = 2.083;
craft.Jxy = 0.0;
craft.Jxz = 0.0;
craft.Jyz = 0.0;

%% 环境变量
envs.rho = 1.225;
envs.g = 9.8;

%% 旋翼1参数
name = 'NACA0012';
para.type = "UDF";
para.getCL = @(x)naca0012c81(x.alpha, 1);
para.getCD = @(x)naca0012c81(x.alpha, 0);
NACA0012 = Profile(name, para);

Para_rotor.name = 'rotor1';
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

Para_rotor.pos = [0.5825; 0; -0.25];%旋翼相对参考坐标系的位置（m）
Para_rotor.Tilt = [0; 0; 0];%桨轴系相对参考坐标系的夹角数组（一般包含桨轴前倾角，deg）

Para_rotor.flap_type = 'default';%挥舞类型
Para_rotor.inflow_type = 'Pitt';%入流类型

%% 吊挂物参数
Para_cable.mc = 0.1;
Para_cable.l_c0 = 6;
Para_cable.Kc = 1000;
Para_cable.Cc = 100;
Para_cable.num = 10;
Para_cable.C_Dc = 0.01;
envs.Vkgc = repmat([0.01 0.01 0.01]', 1, Para_cable.num);


%%
Para.Craft = craft;
Para.name = 'Ch47';
Para.Comp(1).type = 'MainRotor';
Para.Comp(1).para = Para_rotor;

%旋翼2参数
Para_rotor.name = 'rotor2';
Para_rotor.RotorDir = 0;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.pos = [-0.5825; 0; -0.25];%旋翼相对参考坐标系的位置（m）
Para.Comp(2).type = 'MainRotor';
Para.Comp(2).para = Para_rotor;

%机身参数
Para_fuse.name = 'Fuse';
Para_fuse.pos = [0 0 0]';%机身相对体轴系的位置
Para_fuse.euler = [0 0 0];%机身相对体轴系的角度
Para.Comp(3).type = 'Fuse';
Para.Comp(3).para = Para_fuse;

%吊索参数
% Para_cable.name = 'cable';
% Para.Comp(4).type = 'cable';
% Para.Comp(4).para = Para_cable;
%操纵
Para.mnplt='tandem';



