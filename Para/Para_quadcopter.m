%%整机参数
craft.m = 17.0;%整机重量
craft.Jxx = 1;%整机惯量
craft.Jyy = 1.2;
craft.Jzz = 1.3;
craft.Jxy = 0.0;
craft.Jxz = 0.0;
craft.Jyz = 0.0;

%% 环境变量
envs.rho = 1.225;
envs.g = 9.8;

%% 4螺旋桨 通用设置
dia=0.51;
chord=0.025;
nblade=2;           % 桨叶片数
pitch=0.3048;       % 螺距

Xr1=0.44;  Yr1=-0.65;    Zr1=-0.02;
Xr2=0.44;  Yr2=0.65;    Zr2=-0.02;
Xr3=-0.375;  Yr3=0.75;    Zr3=-0.1;
Xr4=-0.375;  Yr4=0-0.75;    Zr4=-0.1;

%% 4螺旋桨
name = 'NACA0012';
para.type = "UDF";
para.getCL = @(x)naca0012c81(x.alpha, 1);
para.getCD = @(x)naca0012c81(x.alpha, 0);
NACA0012 = Profile(name, para);

Para_rotor.name = 'rotor1';
Para_rotor.RotorDir = 1;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.nBlade = 2;%桨叶片数
Para_rotor.R = dia/2;%桨叶半径
Para_rotor.chords = [1; chord]';%桨叶弦长数组
Para_rotor.twists = [1,pitch; 0,0]';%初始桨距数组
Para_rotor.dihedrals =  [1 0];%上反角（初始锥度角）数据
Para_rotor.profiles = {1, NACA0012};%截面翼型数组

Para_rotor.type = 'Hinge';%旋翼类型
Para_rotor.omega_beta = 0;%挥舞一阶频率，旋翼类型为无铰式需定义，用来计算挥舞等效偏置量
Para_rotor.e = 0.02;%挥舞偏置，旋翼类型为有铰式需定义
Para_rotor.K_beta = 0;%桨毂弹簧力矩，旋翼类型为有铰式需定义，无铰式默认为0
Para_rotor.m = 0.265;%桨叶质量沿展向分布（kg/m）

Para_rotor.pos = [Xr1; Yr1; Zr1];%旋翼相对参考坐标系的位置（m）
Para_rotor.Tilt = [0; 0; 0];%桨轴系相对参考坐标系的夹角数组（一般包含桨轴前倾角，deg）

Para_rotor.flap_type = 'default';%挥舞类型
Para_rotor.inflow_type = 'Pitt';%入流类型

%%
Para.Craft = craft;
Para.name = 'Ch47';
Para.Comp(1).type = 'Propeller';
Para.Comp(1).para = Para_rotor;

%旋翼2参数
Para_rotor.name = 'rotor2';
Para_rotor.RotorDir = 0;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.pos = [Xr2; Yr2; Zr2];%旋翼相对参考坐标系的位置（m）
Para.Comp(2).type = 'Propeller';
Para.Comp(2).para = Para_rotor;

%旋翼3参数
Para_rotor.name = 'rotor3';
Para_rotor.RotorDir = 0;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.pos = [Xr3; Yr3; Zr3];%旋翼相对参考坐标系的位置（m）
Para.Comp(2).type = 'Propeller';
Para.Comp(2).para = Para_rotor;

%旋翼4参数
Para_rotor.name = 'rotor4';
Para_rotor.RotorDir = 1;%旋转方向（逆时针：1，顺时针：0，默认值：1）
Para_rotor.pos = [Xr4; Yr4; Zr4];%旋翼相对参考坐标系的位置（m）
Para.Comp(2).type = 'Propeller';
Para.Comp(2).para = Para_rotor;

%机身参数
Para_fuse.name = 'Fuse';
Para_fuse.pos = [0 0 0]';%机身相对体轴系的位置
Para_fuse.euler = [0 0 0];%机身相对体轴系的角度
Para.Comp(3).type = 'Fuse';
Para.Comp(3).para = Para_fuse;
