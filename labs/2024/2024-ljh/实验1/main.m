
addpath(genpath('robotcore'))
clear;
clc;

%%  定义参考路径
% %参考路径1
path = [2.00    1.00;
        1.25    1.75;
        2.00    5.00;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
% 
% % 参考路径2
% pathx = (0:pi/50:3*pi);
% pathy = 10*exp(-0.5*pathx).*sin(0.5*pi*pathx);

% 参考路径3
% pathx =  0:0.5:15;
% pathy = sqrt(pathx) + sin(pathx);
% path = [pathx', pathy'];

%% 相关参数定义
H = 1;                % 车辆轴距，单位：m
goalRadius = 0.05;    % 当车辆距离目标点小于此值时结束跟踪
sampleTime = 0.1;    % 仿真时间，单位s
vizRate = rateControl(10);  % 控制频率

% 设置机器人初始状态
robotInitialLocation = path(1,:);  % 初始位置
initialOrientation = 0;         % 初始航向角
robotCurrentPose = [robotInitialLocation initialOrientation]';  % [x; y; theta];
v0 = 0;  % 初始线速度
w = 0;   % 初始角速度

robotGoal = path(end,:);  % 确定目标点
vr = 2;  % 目标线速度
distanceToGoal = norm(robotInitialLocation - robotGoal);  % 计算当前位置与目标点的距离
v = v0;  % 初始速度
yaw = initialOrientation;  % 初始航向角

% 确定车辆框架尺寸以最接近代表车辆
frameSize = H/0.5;

con = 0.4;  % 视距常数
k = 0.4;  % 倍率
% 轨迹预处理, 使得相邻轨迹点之间的距离不小于con
x_tmp = [];
y_tmp = [];
for i=1:size(path,1)-1
    add_points = floor(sqrt((path(i,1) - path(i+1,1)) ^ 2 + (path(i,2) - path(i+1,2)) ^ 2) / con) + 2;
    x_tmp = [x_tmp, linspace(path(i,1), path(i+1,1), add_points)];
    y_tmp = [y_tmp, linspace(path(i,2), path(i+1,2), add_points)];
end
path = [x_tmp, path(size(path,1),1); y_tmp, path(size(path,1),2)]';

while( distanceToGoal > goalRadius )
    % 更新轨迹跟踪图
    hold off
    
    % 绘制参考轨迹
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % 计算控制器的输出，将其输入到机器人中
    [v, w] = pure_pursuit_control(robotCurrentPose, path, v, vr, k, con);  % 机器人系下v与w

    % 使用控制输入得到机器人的速度
    vel = derivative(robotCurrentPose, v, w, sampleTime);
    
    % 更新当前姿态
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % 重新计算与目标点的距离
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % 绘制机器人的运行轨迹
%     plotTrVec = [robotCurrentPose(1:2); 0];
%     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
%     plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
%     light;

    scatter(robotCurrentPose(1), robotCurrentPose(2), 20, [0.5,0,0], 'filled');
    plot([robotCurrentPose(1); robotCurrentPose(1) + 1 * cos(robotCurrentPose(3))], [robotCurrentPose(2); robotCurrentPose(2) + 1 * sin(robotCurrentPose(3))], 'color', [0.5,0,0]);

    % 设置图的横纵坐标
    xlim([-5 15]);
    ylim([-5 15]);
    
    waitfor(vizRate);  % 暂停代码执行，知道计时器vizRate出发，vizRate为采样频率
    
    if distanceToGoal<0.1  % 到达目标后停止
        break
    end
end





