function [route,numExpanded,I,J] = DijkstraGrid (input_map, start_coords, dest_coords)
% 在一个网格上运行 Dijkstra 算法
% Inputs : 
%  input_map : 逻辑数组，其中自由空间单元格为 false 或 0 
%              障碍物为 true 或 1
%  start_coords and dest_coords : 起点和终点单元格的坐标
%                                 第一个元素是行，第二个是列
% Output :
%    route : 包含沿从起点到终点的最短路径的单元格的线性索引的数组
%            如果没有路径，则为空数组。这是一个一维向量。
%    numExpanded: 记得同时返回你在搜索过程中扩展的总节点数
%                 不要将目标节点计算为扩展节点

% 用一个map矩阵来表示每个点的状态
% 设置用于显示的颜色映射
% 1 - white - 空白单元格
% 2 - black - 障碍物
% 3 - red - 相当于CLOSED列表的作用
% 4 - blue  - 相当于OPEN列表的作用
% 5 - green - 起点
% 6 - yellow - 目的地

cmap = [1 1 1; ...   白色，表示空闲的格子
    0 0 0; ...       黑色，表示障碍物
    1 0 0; ...       红色，表示已访问的格子
    0 0 1; ...       蓝色，表示正在考虑的格子
    0 1 0; ...       绿色，表示起点
    1 1 0; ...       黄色，表示终点
    0.5 0.5 0.5];   %灰色，表示路径上的格子
colormap(cmap);

% 是否将每次迭代地图进行可视化展示
drawMapEveryTime = true;
[nrows, ncols] = size(input_map);

% map - 一个二维数组，用于跟踪每个格子的状态
map = zeros(nrows,ncols);

map(~input_map) = 1;   % 标记空闲格子
map(input_map)  = 2;   % 标记障碍物格子

% 产生起点和终点的线性索引
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));
map(start_node) = 5;  % 标记起点
map(dest_node)  = 6;  % 标记终点

% 初始化距离数组，除起点外所有节点的距离都为无穷大
distanceFromStart = Inf(nrows, ncols);

% 存储每个网格单元格的父节点索引
parent = zeros(nrows,ncols);
[X, Y] = meshgrid (1:ncols, 1:nrows);  % 函数可以将输入的向量扩展为二维网格

% 初始化距离数组，除起点外所有节点的距离都为无穷大
distanceFromStart(start_node) = 0;

% 将起点节点的距离设置为0
numExpanded = 0;

% Main Loop
while true
    if (drawMapEveryTime)
        map1=imrotate(map,90);
        image(0.5, 0.5, map1);
        axis image;  % 设置坐标轴的宽高比为1，使地图绘制时不发生形变
        drawnow;  % 立即更新图形窗口 
    end
    
    % 寻找最小距离点，并返回最小值min_dist和对应的索引current
    [min_dist, current] = min(distanceFromStart(:) + abs(map(:) - 4) * 10000);  % 只会访问open(map=4)的点

    % 判断条件是否满足其中之一，说明已找最短路径或无法到达终点
    % 跳出循环，结束路径搜索
    if ((current == dest_node) || isinf(min_dist))
        break;
    end
    
    % 更新地图
    map(current) = 3;   % 将当前节点标记为已访问状态
    
    numExpanded = numExpanded + 1;

    % 计算当前节点的行列坐标
    [i, j] = ind2sub(size(distanceFromStart), current);

   % ********************************************************************* 
    % 两行星号*之间编写代码，实现以下功能：
    % 访问当前节点的每个相邻节点
    % 根据访问结果更新地图、f和g和父节点表
    if i > 1
        if map(i-1, j) == 1 || map(i-1, j) == 6
            map(i-1, j) = 4;
            parent(i-1, j) = sub2ind(size(map), i, j);
            distanceFromStart(i-1, j) = distanceFromStart(i, j) + 1;
        end
    end
    if j > 1
        if map(i, j-1) == 1 || map(i, j-1) == 6
            map(i, j-1) = 4;
            parent(i, j-1) = sub2ind(size(map), i, j);
            distanceFromStart(i, j-1) = distanceFromStart(i, j) + 1;
        end
    end
    if i < nrows
        if map(i+1, j) == 1 || map(i+1, j) == 6
            map(i+1, j) = 4;
            parent(i+1, j) = sub2ind(size(map), i, j);
            distanceFromStart(i+1, j) = distanceFromStart(i, j) + 1;
        end
    end
    if j < ncols
        if map(i, j+1) == 1 || map(i, j+1) == 6
            map(i, j+1) = 4;
            parent(i, j+1) = sub2ind(size(map), i, j);
            distanceFromStart(i, j+1) = distanceFromStart(i, j) + 1;
        end
    end
    % *********************************************************************
end

%% 构建从起点到终点的路径，并进行可视化
if (isinf(distanceFromStart(dest_node))) % 判断是否存在路径
    route = [];
else

    % ********************************************************************* 
    % 两行星号*之间编写代码，实现以下功能：
    % 构建从终点到起点回溯路径
    route = [];
    I = [];
    J = [];
    now = dest_node;
    while now ~= 0  % 循环在起点停止
        route = [route now];
        I = [I X(now)];
        J = [J Y(now)];
        now = parent(now);
    end
    numExpanded = numExpanded - 1;  % 长度去除终点
    I = I - 0.5;
    J = J - 0.5;
    
    % 重绘起点与终点
    map(start_node) = 5;
    map(dest_node)  = 6;
    % ********************************************************************* 

        % 用于可视化地图和路径的代码片段
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        map1 = imrotate(map,90);
        image(0.5, 0.5, map1);
        grid on;
        axis image;
    end
end

end


