
addpath(genpath('robotcore'))
clear;
clc;

%%  ����ο�·��
% %�ο�·��1
path = [2.00    1.00;
        1.25    1.75;
        2.00    5.00;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
% 
% % �ο�·��2
% pathx = (0:pi/50:3*pi);
% pathy = 10*exp(-0.5*pathx).*sin(0.5*pi*pathx);

% �ο�·��3
% pathx =  0:0.5:15;
% pathy = sqrt(pathx) + sin(pathx);
% path = [pathx', pathy'];

%% ��ز�������
H = 1;                % ������࣬��λ��m
goalRadius = 0.05;    % ����������Ŀ���С�ڴ�ֵʱ��������
sampleTime = 0.1;    % ����ʱ�䣬��λs
vizRate = rateControl(10);  % ����Ƶ��

% ���û����˳�ʼ״̬
robotInitialLocation = path(1,:);  % ��ʼλ��
initialOrientation = 0;         % ��ʼ�����
robotCurrentPose = [robotInitialLocation initialOrientation]';  % [x; y; theta];
v0 = 0;  % ��ʼ���ٶ�
w = 0;   % ��ʼ���ٶ�

robotGoal = path(end,:);  % ȷ��Ŀ���
vr = 2;  % Ŀ�����ٶ�
distanceToGoal = norm(robotInitialLocation - robotGoal);  % ���㵱ǰλ����Ŀ���ľ���
v = v0;  % ��ʼ�ٶ�
yaw = initialOrientation;  % ��ʼ�����

% ȷ��������ܳߴ�����ӽ�������
frameSize = H/0.5;

con = 0.4;  % �Ӿೣ��
k = 0.4;  % ����
% �켣Ԥ����, ʹ�����ڹ켣��֮��ľ��벻С��con
x_tmp = [];
y_tmp = [];
for i=1:size(path,1)-1
    add_points = floor(sqrt((path(i,1) - path(i+1,1)) ^ 2 + (path(i,2) - path(i+1,2)) ^ 2) / con) + 2;
    x_tmp = [x_tmp, linspace(path(i,1), path(i+1,1), add_points)];
    y_tmp = [y_tmp, linspace(path(i,2), path(i+1,2), add_points)];
end
path = [x_tmp, path(size(path,1),1); y_tmp, path(size(path,1),2)]';

while( distanceToGoal > goalRadius )
    % ���¹켣����ͼ
    hold off
    
    % ���Ʋο��켣
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % �����������������������뵽��������
    [v, w] = pure_pursuit_control(robotCurrentPose, path, v, vr, k, con);  % ������ϵ��v��w

    % ʹ�ÿ�������õ������˵��ٶ�
    vel = derivative(robotCurrentPose, v, w, sampleTime);
    
    % ���µ�ǰ��̬
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % ���¼�����Ŀ���ľ���
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % ���ƻ����˵����й켣
%     plotTrVec = [robotCurrentPose(1:2); 0];
%     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
%     plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
%     light;

    scatter(robotCurrentPose(1), robotCurrentPose(2), 20, [0.5,0,0], 'filled');
    plot([robotCurrentPose(1); robotCurrentPose(1) + 1 * cos(robotCurrentPose(3))], [robotCurrentPose(2); robotCurrentPose(2) + 1 * sin(robotCurrentPose(3))], 'color', [0.5,0,0]);

    % ����ͼ�ĺ�������
    xlim([-5 15]);
    ylim([-5 15]);
    
    waitfor(vizRate);  % ��ͣ����ִ�У�֪����ʱ��vizRate������vizRateΪ����Ƶ��
    
    if distanceToGoal<0.1  % ����Ŀ���ֹͣ
        break
    end
end





