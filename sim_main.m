addpath('Analysis\', 'Component\Rotor\', 'Component\', 'Component\Fuse\', 'Component\Airfoil\', 'Math\', 'Para\')
run Para_CH47_our.m;
tandem = Craft(Para);
load('x0.mat');

%% 控制参数
%滚转角PID
PID_Param1.Kp_roll = 4;
PID_Param1.Ki_roll = 0.5;
PID_Param1.Kp_rollrate = 1;
PID_Param1.Ki_rollrate = 0.05;
%俯仰角PID
PID_Param1.Kp_pitch = 5;
PID_Param1.Ki_pitch = 0;
PID_Param1.Kp_pitchrate = 0.1;
PID_Param1.Ki_pitchrate = 0.05;
%偏航角PID
PID_Param1.Kp_yaw = 5;
PID_Param1.Ki_yaw = 0;
PID_Param1.Kp_yawrate = 1;
PID_Param1.Ki_yawrate = 0.05;
%高度PID
PID_Param1.Kp_Hight = 3;
PID_Param1.Ki_Hight = 0;
PID_Param1.Kp_Vz = 1;
PID_Param1.Ki_Vz = 0.05;
%前向位置PID
PID_Param1.Kp_x = 1;
PID_Param1.Ki_x = 0;
PID_Param1.Kp_Vx = 40;
PID_Param1.Ki_Vx = 0;
%侧向位置PID
PID_Param1.Kp_y = 1;
PID_Param1.Ki_y = 0;
PID_Param1.Kp_Vy = 40;
PID_Param1.Ki_Vy = 0;

%% 各直升机初始位置
pos1 = [-2 -2 -0.3];
pos2 = [-2 2 -0.3];
pos3 = [2 2 -0.3];
pos4 = [2 -2 -0.3];
pos_load = [0 0 -0.25];

sim our_ch47.slx
