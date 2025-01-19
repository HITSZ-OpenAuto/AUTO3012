function [v, w] = pure_pursuit_control(robotCurrentPose, path, v, vr, k, con) 
    % Ѱ����С������ĵ�
    now_index = 0;
    now_dist = 999999;
    for i=1:size(path,1)
        if sqrt((path(i,1) - robotCurrentPose(1)) ^ 2 + (path(i,2) - robotCurrentPose(2)) ^ 2) < now_dist
            now_dist = sqrt((path(i,1) - robotCurrentPose(1)) ^ 2 + (path(i,2) - robotCurrentPose(2)) ^ 2);
            now_index = i;
        end
    end
    % ѡȡĿ���
    tar_index = size(path,1);
    vis = v * k + con;  % Ԥ�����
    for i=now_index+1:size(path,1)
        if sqrt((path(i,1) - robotCurrentPose(1)) ^ 2 + (path(i,2) - robotCurrentPose(2)) ^ 2) > vis
            tar_index = i;
            break
        end
    end
    scatter(path(tar_index,1), path(tar_index,2), 20, 'b', 'filled');  % ����Ŀ��
    % �������λ��
    if tar_index ~= size(path,1)
        point_pose = atan2(path(tar_index+1,2) - path(tar_index,2), path(tar_index+1,1) - path(tar_index,1));
    else
        point_pose = atan2(path(tar_index,2) - path(tar_index-1,2), path(tar_index,1) - path(tar_index-1,1));
    end
    delPose = [path(tar_index, 1:2)'; point_pose] - robotCurrentPose;
    delPose = [ cos(robotCurrentPose(3)), sin(robotCurrentPose(3)), 0;
               -sin(robotCurrentPose(3)), cos(robotCurrentPose(3)), 0;
                0, 0, 1] * delPose;
    if delPose(3) > pi 
        delPose(3) = delPose(3) - 2*pi;
    elseif delPose(3) < -pi
        delPose(3) = delPose(3) + 2*pi;
    end
    delPose
    v = v + 0.3*(vr - v);  % ʹ�õ�P������ʵ�����ٶȵĿ��� 
    w = 2 * v * delPose(2) / (delPose(1)^2 + delPose(2)^2);
end