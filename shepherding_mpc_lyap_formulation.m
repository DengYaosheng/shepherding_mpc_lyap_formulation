% 参数定义
N = 20;  % 智能体数量
R = 10;  % 视野半径
k = 1;   % 引力系数
a = 1;   % 斥力系数
d = 1;   % 编队距离
T = 10;  % 控制时长
Np = 5;  % 预测步数
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
        
        % 利用MPC计算速度
        % 构造MPC问题
        x0 = [pos(i,:), vel(i,:)];
        A = [1, 0, T, 0; 0, 1, 0, T; 0, 0, 1, 0; 0, 0, 0, 1];
        B = [T^2/2, 0; 0, T^2/2; T, 0; 0, T];
        Q = diag([1, 1, 0, 0]);
        R = diag([1, 1]);
        P = Q;
        Nc = 2*Np;
        Ns = 4;
        [x, u, ~] = mpc_controller(x0, A, B, Q, R, P, Np, Nc, Ns, clus');
        
        % 更新速度和位置
        vel(i,:)
        vel(i,:) = u(1,:);
        pos(i,:) = x(2,:);
    end
    
    % 绘制智能体位置
    clf
    plot(pos(:,1),pos(:,2),'o')
    axis([-15 15 -15 15])
    axis square
    drawnow
end

function [x, u, J] = mpc_controller(x0, A, B, Q, R, P, Np, Nc, Ns, r)
    % MPC控制器
    x = zeros(Ns, Np+1);
    u = zeros(Nc, Np);
    x(:,1) = x0';
    for i = 1:Np
        [u(:,i), ~, ~] = quadprog(2*(B'*Q*A), 2*R*u(:,i), [], [], [], [], [], [], [], optimset('Display','off'));
        x(:,i+1) = A*x(:,i) + B*u(:,i);
    end
    J = (x(:,end)'*P*x(:,end) + u(:,end)'*R*u(:,end))/2;
end
