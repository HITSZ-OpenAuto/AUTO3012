function [route,numExpanded,I,J] = AStarGrid (input_map, start_coords, dest_coords)
% ��һ������������ A* �㷨
% Inputs : 
%  input_map : �߼����飬�������ɿռ䵥Ԫ��Ϊ false �� 0 
%              �ϰ���Ϊ true �� 1
%  start_coords and dest_coords : �����յ㵥Ԫ�������
%                                 ��һ��Ԫ�����У��ڶ�������
% Output :
%    route : �����ش���㵽�յ�����·���ĵ�Ԫ�����������������
%            ���û��·������Ϊ�����顣����һ��һά������
%    numExpanded: �ǵ�ͬʱ��������������������չ���ܽڵ���
%                 ��Ҫ��Ŀ��ڵ����Ϊ��չ�ڵ�

% ��һ��map��������ʾÿ�����״̬
% ����������ʾ����ɫӳ��
% 1 - white - �հ׵�Ԫ��
% 2 - black - �ϰ���
% 3 - red - �൱��CLOSED�б������
% 4 - blue  - �൱��OPEN�б������
% 5 - green - ���
% 6 - yellow - Ŀ�ĵ�

cmap = [1 1 1; ...   ��ɫ����ʾ���еĸ���
    0 0 0; ...       ��ɫ����ʾ�ϰ���
    1 0 0; ...       ��ɫ����ʾ�ѷ��ʵĸ���
    0 0 1; ...       ��ɫ����ʾ���ڿ��ǵĸ���
    0 1 0; ...       ��ɫ����ʾ���
    1 1 0; ...       ��ɫ����ʾ�յ�
    0.5 0.5 0.5];   %��ɫ����ʾ·���ϵĸ���
colormap(cmap);

% �Ƿ�ÿ�ε�����ͼ���п��ӻ�չʾ
drawMapEveryTime = true;
[nrows, ncols] = size(input_map);

% map - һ����ά���飬���ڸ���ÿ�����ӵ�״̬
map = zeros(nrows,ncols);

map(~input_map) = 1;   % ��ǿ��и���
map(input_map)  = 2;   % ����ϰ������

% ���������յ����������
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));
map(start_node) = 5;  % ������
map(dest_node)  = 6;  % ����յ�

parent = zeros(nrows,ncols);  % ������¼ÿ���ڵ�ĸ��ڵ�
[X, Y] = meshgrid (1:ncols, 1:nrows);  % �������Խ������������չΪ��ά����

xd = dest_coords(1);  % Ŀ��ڵ�� x ����
yd = dest_coords(2);  % Ŀ��ڵ�� y ����

% �������پ�����Ϊ����ʽ����
% H = abs(X - xd) + abs(Y - yd);
% ��ŷ�Ͼ��������Ϊ����ʽ����
H = sqrt((X - xd).*(X-xd) + (Y - yd).*(Y-yd));
H = H';

% ��ʼ���ɱ����ۺ���������������Ϊ�������ʾ��û̽������Щ�ڵ�
f = Inf(nrows, ncols);
g = Inf(nrows, ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

% ��¼�Ѿ���̽���Ľڵ���
numExpanded = 0;

% Main Loop
while true
    if (drawMapEveryTime)
        map1=imrotate(map,90);
        image(0.5, 0.5, map1);
        axis image;  % ����������Ŀ�߱�Ϊ1��ʹ��ͼ����ʱ�������α�
        drawnow;  % ��������ͼ�δ��� 
    end

    % Ѱ��f��С�ĵ㣬��������Сֵmin_f�Ͷ�Ӧ������current
    [min_f, current] = min(f(:) + abs(map(:) - 4) * 10000);  % ֻ�����open(map=4)�ĵ�

    % �ж������Ƿ���������֮һ��˵���������·�����޷������յ�
    % ����ѭ��������·������
    if ((current == dest_node) || isinf(min_f))
        break;
    end

    % ���µ�ͼ
    map(current) = 3;  % ����ǰ�ڵ���Ϊ�ѷ���״̬
    numExpanded = numExpanded + 1;
   
    % ���㵱ǰ�ڵ����������
    [i, j] = ind2sub(size(f), current);

   % ********************************************************************* 
    % �����Ǻ�*֮���д���룬ʵ�����¹��ܣ�
    % ���ʵ�ǰ�ڵ��ÿ�����ڽڵ�
    % ���ݷ��ʽ�����µ�ͼ��f��g�͸��ڵ��
    % ���ʽڵ�
    if i > 1
        if map(i-1, j) == 1 || map(i-1, j) == 6
            map(i-1, j) = 4;
            parent(i-1, j) = sub2ind(size(map), i, j);
            g(i-1, j) = g(i, j) + 0.1;
        end
    end
    if j > 1
        if map(i, j-1) == 1 || map(i, j-1) == 6
            map(i, j-1) = 4;
            parent(i, j-1) = sub2ind(size(map), i, j);
            g(i, j-1) = g(i, j) + 0.1;
        end
    end
    if i < nrows
        if map(i+1, j) == 1 || map(i+1, j) == 6
            map(i+1, j) = 4;
            parent(i+1, j) = sub2ind(size(map), i, j);
            g(i+1, j) = g(i, j) + 0.1;
        end
    end
    if j < ncols
        if map(i, j+1) == 1 || map(i, j+1) == 6
            map(i, j+1) = 4;
            parent(i, j+1) = sub2ind(size(map), i, j);
            g(i, j+1) = g(i, j) + 0.1;
        end
    end
    f = g + H;
    %*********************************************************************
end

%% ��������㵽�յ��·���������п��ӻ�

if (isinf(f(dest_node))) % �ж��Ƿ����·��
    route = [];
else
    % ********************************************************************* 
    % �����Ǻ�*֮���д���룬ʵ�����¹��ܣ�
    % �������յ㵽������·��
    route = [];
    I = [];
    J = [];
    now = dest_node;
    while now ~= 0  % ѭ�������ֹͣ
        route = [route now];
        I = [I X(now)];
        J = [J Y(now)];
        now = parent(now);
    end
    numExpanded = numExpanded - 1;  % ����ȥ���յ�
    I = I - 0.5;
    J = J - 0.5;
    
    % �ػ�������յ�
    map(start_node) = 5;
    map(dest_node)  = 6;
    
    % *********************************************************************

    % ���ڿ��ӻ���ͼ��·���Ĵ���Ƭ��
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.04);
        map1 = imrotate(map,90);
        image(0.5, 0.5, map1);
        grid on;
        axis image;
    end
    
end
end
