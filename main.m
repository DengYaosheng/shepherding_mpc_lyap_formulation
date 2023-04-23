% 参数定义
N = 20;  % 智能体数量
R = 10;  % 视野半径
k = 1;   % 引力系数
a = 1;   % 斥力系数
d = 1;   % 编队距离
alpha = 1; % 李雅普诺夫控制系数

% 初始化智能体位置和速度
pos = rand(N,2)*20-10;   % 随机生成位置
vel = zeros(N,2);        % 初始速度为零

% 设置目标点
target = [0,0];

% 迭代计算
for t = 1:1000
    % 计算每个智能体周围的邻居
    neighbors = cell(N,1);
    for i = 1:N
        for j = 1:N
            if i ~= j && norm(pos(i,:)-pos(j,:)) < R
                neighbors{i} = [neighbors{i}, j];
            end
        end
    end
    
    % 计算每个智能体的速度
    for i = 1:N
        % 计算引力
        att = k*(target - pos(i,:));
        
        % 计算斥力
        rep = zeros(1,2);
        for j = neighbors{i}
            rep = rep + a*(pos(i,:) - pos(j,:))/norm(pos(i,:) - pos(j,:))^2;
        end
        
        % 计算集群控制项
        clus = zeros(1,2);
        for j = neighbors{i}
            clus = clus + (pos(j,:) - pos(i,:));
        end
        clus = (clus - d*(length(neighbors{i})-1)*(pos(i,:) - target))/length(neighbors{i});
        
        % 计算李雅普诺夫控制项
        error = pos(i,:) - (i-1)*d*target/N;
        lyp = -alpha*error;
        
        % 更新速度
        vel(i,:) = vel(i,:) + att + rep + clus + lyp;
        
        % 限制速度大小
        if norm(vel(i,:)) > 1
            vel(i,:) = vel(i,:)/norm(vel(i,:));
        end
    end
    
    % 更新位置
    pos = pos + vel;
    
    % 绘制智能体位置
    scatter(pos(:,1), pos(:,2));
    xlim([-10, 10]);
    ylim([-10, 10]);
    drawnow;
end
