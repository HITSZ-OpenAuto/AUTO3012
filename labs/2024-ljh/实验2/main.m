clc
clear
close all

addpath(genpath('robotcore'))

% 栅格地图的大小
mapSize = [30, 30];

% 障碍物占据概率（0-1之间的值）
obstacleProbability = 0.2;

% 生成随机栅格地图
gridMap = rand(mapSize) < obstacleProbability;

% 设置起点和终点
start_coords = [2, 5];
dest_coords  = [25, 28];

gridMap = logical(gridMap);  % 转换为逻辑数组

% % 调用A*算法输入路径
% [route, numExpanded,I,J] = AStarGrid (gridMap, start_coords, dest_coords);  

% 调用Dijkstra算法输入路径
[route, numExpanded,I,J] = DijkstraGrid (gridMap, start_coords, dest_coords);  

path=[I;J]';  % 生成参考路径

% % 调用A*算法输入路径
% [route, numExpanded,I,J] = AStarGrid (BW2, start_coords, dest_coords);  
% 
% % 调用Dijkstra算法输入路径
% % [route, numExpanded,I,J] = DijkstraGrid (BW2, start_coords, dest_coords);  
% 
% path=[I;J]';  % 生成参考路径

%% 此处与实验一代码一致，使用纯跟踪算法对规划好的参考路径进行跟踪

% 相关参数定义
H = 1;                % 车辆轴距，单位：m
goalRadius = 0.05;    % 当车辆距离目标点小于此值时结束跟踪
sampleTime = 0.05;    % 仿真时间，单位s
vizRate = rateControl(10);  % 控制频率

% 设置机器人初始状态
robotInitialLocation = path(1,:);  % 初始位置
initialOrientation = pi/2;         % 初始航向角
robotCurrentPose = [robotInitialLocation initialOrientation]';
idx_target = 1;
v0 = 5;  % 初始线速度
w = 0;   % 初始角速度

robotGoal = path(end,:);  % 确定目标点
vr = 5;  % 目标线速度
distanceToGoal = norm(robotInitialLocation - robotGoal);  % 计算当前位置与目标点的距离
v = v0;  % 初始速度
yaw = initialOrientation;  % 初始航向角

% 确定车辆框架尺寸以最接近代表车辆，以便于在图上显示
frameSize = H/0.5;

while( distanceToGoal > goalRadius )
    
    % 计算控制器的输出，将其输入到机器人中
    [v, w] = pure_pursuit_control(robotCurrentPose, path, v, vr);  

    % 使用控制输入得到机器人的速度
    vel = derivative(robotCurrentPose, v, w, sampleTime);
    
    % 更新当前姿态
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % 重新计算与目标点的距离
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % 更新轨迹跟踪图
    hold off
%     show(map);  % 使栅格地图保留在轨迹跟踪图中
    map1 = imrotate(gridMap,90);
    image(0.5, 0.5, gridMap);
    hold all

    % 绘制参考轨迹
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % 绘制机器人的运行轨迹
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;

    % 设置图的横纵坐标
    xlim([0 27]);
    ylim([0 26]);
    
    waitfor(vizRate);  % 暂停代码执行，知道计时器vizRate出发，vizRate为采样频率
end
