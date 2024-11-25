clc
clear
close all

addpath(genpath('robotcore'))

% դ���ͼ�Ĵ�С
mapSize = [30, 30];

% �ϰ���ռ�ݸ��ʣ�0-1֮���ֵ��
obstacleProbability = 0.2;

% �������դ���ͼ
gridMap = rand(mapSize) < obstacleProbability;

% ���������յ�
start_coords = [2, 5];
dest_coords  = [25, 28];

gridMap = logical(gridMap);  % ת��Ϊ�߼�����

% % ����A*�㷨����·��
% [route, numExpanded,I,J] = AStarGrid (gridMap, start_coords, dest_coords);  

% ����Dijkstra�㷨����·��
[route, numExpanded,I,J] = DijkstraGrid (gridMap, start_coords, dest_coords);  

path=[I;J]';  % ���ɲο�·��

% % ����A*�㷨����·��
% [route, numExpanded,I,J] = AStarGrid (BW2, start_coords, dest_coords);  
% 
% % ����Dijkstra�㷨����·��
% % [route, numExpanded,I,J] = DijkstraGrid (BW2, start_coords, dest_coords);  
% 
% path=[I;J]';  % ���ɲο�·��

%% �˴���ʵ��һ����һ�£�ʹ�ô������㷨�Թ滮�õĲο�·�����и���

% ��ز�������
H = 1;                % ������࣬��λ��m
goalRadius = 0.05;    % ����������Ŀ���С�ڴ�ֵʱ��������
sampleTime = 0.05;    % ����ʱ�䣬��λs
vizRate = rateControl(10);  % ����Ƶ��

% ���û����˳�ʼ״̬
robotInitialLocation = path(1,:);  % ��ʼλ��
initialOrientation = pi/2;         % ��ʼ�����
robotCurrentPose = [robotInitialLocation initialOrientation]';
idx_target = 1;
v0 = 5;  % ��ʼ���ٶ�
w = 0;   % ��ʼ���ٶ�

robotGoal = path(end,:);  % ȷ��Ŀ���
vr = 5;  % Ŀ�����ٶ�
distanceToGoal = norm(robotInitialLocation - robotGoal);  % ���㵱ǰλ����Ŀ���ľ���
v = v0;  % ��ʼ�ٶ�
yaw = initialOrientation;  % ��ʼ�����

% ȷ��������ܳߴ�����ӽ����������Ա�����ͼ����ʾ
frameSize = H/0.5;

while( distanceToGoal > goalRadius )
    
    % �����������������������뵽��������
    [v, w] = pure_pursuit_control(robotCurrentPose, path, v, vr);  

    % ʹ�ÿ�������õ������˵��ٶ�
    vel = derivative(robotCurrentPose, v, w, sampleTime);
    
    % ���µ�ǰ��̬
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % ���¼�����Ŀ���ľ���
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % ���¹켣����ͼ
    hold off
%     show(map);  % ʹդ���ͼ�����ڹ켣����ͼ��
    map1 = imrotate(gridMap,90);
    image(0.5, 0.5, gridMap);
    hold all

    % ���Ʋο��켣
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % ���ƻ����˵����й켣
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;

    % ����ͼ�ĺ�������
    xlim([0 27]);
    ylim([0 26]);
    
    waitfor(vizRate);  % ��ͣ����ִ�У�֪����ʱ��vizRate������vizRateΪ����Ƶ��
end
