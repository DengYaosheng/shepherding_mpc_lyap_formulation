%% 定义障碍物位置
obs_pos = [5, 5];

%% 定义李雅普诺夫控制器参数
k_att = 0.5;
k_rep = 10;
d_rep = 2;

%% 定义CBF控制器参数
k_cbf = 10;
a_cbf = 0.5;
b_cbf = 1;

%% 定义MPC控制器参数
Q = diag([1, 1, 0, 0]);
R = 0.1*eye(2);
P = Q;
Np = 10;
Nc = 2;
Ns = 4;

%% 定义仿真参数
dt = 0.1;
tf = 10;
N = tf/dt;
pos = zeros(N, 2);
vel = zeros(N, 2);
pos(1,:) = [-10, -10];
vel(1,:) = [1, 0];

%% 开始仿真
for t = 1:N-1
    %% 计算控制输入
    x0 = [pos(t,:), vel(t,:)]';
    A = [0, 0, 1, 0;
         0, 0, 0, 1;
         0, 0, 0, 0;
         0, 0, 0, 0];
    B = [0, 0;
         0, 0;
         1, 0;
         0, 1];
     
    % 计算李雅普诺夫控制输入
    u_att = -k_att*(x0(1:2) - obs_pos);
    u_rep = zeros(2,1);
    for i = 1:size(pos,1)
        if i ~= t
            d = norm(pos(i,:) - pos(t,:));
            if d < d_rep
                u_rep = u_rep - k_rep*(1/d - 1/d_rep)*(pos(i,:) - pos(t,:))'/d^3;
            end
        end
    end
    u_lf = u_att + u_rep;
    
    % 计算CBF控制输入
    if norm(x0(1:2) - obs_pos) < 2
        alpha = (obs_pos - x0(1:2))'*u_lf/(norm(obs_pos - x0(1:2))^2);
        u_cbf = -k_cbf*(alpha - a_cbf)^2*(alpha - b_cbf)*((obs_pos - x0(1:2))/norm(obs_pos - x0(1:2)))';
    else
        u_cbf = zeros(2,1);
    end
    
    % 计算MPC控制输入
    r = [obs_pos, zeros(1,Np-1)]';
    [x, u_mpc, ~] = mpc_controller(x0, A, B, Q, R, P, Np, Nc, Ns, r);
    u = u_mpc(:,1) + u_lf + u_cbf;
    
    %% 更新状态
    vel(t+1,:) = u';
    pos(t+1,:) = pos(t,:) + vel(t+1,:)*dt;
    
    %% 绘制智能体位置
    clf
    plot(pos(:,1),pos(:,2))
end
    %% 定义障碍物位置
obs_pos = [5, 5];

%% 定义李雅普诺夫控制器参数
k_att = 0.5;
k_rep = 10;
d_rep = 2;

%% 定义CBF控制器参数
k_cbf = 10;
a_cbf = 0.5;
b_cbf = 1;

%% 定义MPC控制器参数
Q = diag([1, 1, 0, 0]);
R = 0.1*eye(2);
P = Q;
Np = 10;
Nc = 2;
Ns = 4;

%% 定义仿真参数
dt = 0.1;
tf = 10;
N = tf/dt;
pos = zeros(N, 2);
vel = zeros(N, 2);
pos(1,:) = [-10, -10];
vel(1,:) = [1, 0];

%% 开始仿真
for t = 1:N-1
    %% 计算控制输入
    x0 = [pos(t,:), vel(t,:)]';
    A = [0, 0, 1, 0;
         0, 0, 0, 1;
         0, 0, 0, 0;
         0, 0, 0, 0];
    B = [0, 0;
         0, 0;
         1, 0;
         0, 1];
r = [10, 0, 0, 0]';
[x, u, J] = mpc_controller(x0, A, B, Q, R, P, Np, Nc, Ns, r);
u = u(:, 1);

%% 计算合力控制输入
x_vec = reshape(x, [Ns, Np+1])';
x_curr = x_vec(1, 1:2);
v_curr = x_vec(1, 3:4);
att_ctrl = -k_att*(x_curr - r(1:2));
rep_ctrl = zeros(1, 2);
for j = 1:Np+1
    if j ~= 1
        pos_j = x_vec(j, 1:2);
        dist = norm(x_curr - pos_j);
        if dist < d_rep
            rep_ctrl = rep_ctrl - k_rep*(1/dist - 1/d_rep)*((x_curr - pos_j)/dist);
        end
    end
end
total_ctrl = att_ctrl + rep_ctrl + k_cbf*control_barrier_function(x_curr, obs_pos, a_cbf, b_cbf);

%% 更新状态
vel(t+1,:) = v_curr + total_ctrl*dt;
pos(t+1,:) = x_curr + vel(t+1,:)*dt;
end

%% 绘制羊的轨迹和障碍物
figure;
plot(pos(:,1), pos(:,2), 'b-', 'LineWidth', 2);
hold on;
plot(obs_pos(1), obs_pos(2), 'ro', 'LineWidth', 2, 'MarkerSize', 8);
axis equal;
title('Shepherding Control with MPC and CBF');
xlabel('X');
ylabel('Y');

%% 控制障碍物的李雅普诺夫控制函数
function ctrl = control_barrier_function(pos, obs_pos, a, b)
obs_dist = norm(pos - obs_pos);
if obs_dist < a
ctrl = -b*(1/obs_dist - 1/a)*(pos - obs_pos)/obs_dist^3;
else
ctrl = zeros(1, 2);
end
end

%% 定义MPC控制函数
function [x, u, J] = mpc_controller(x0, A, B, Q, R, P, Np, Nc, Ns, r)
    cvx_begin quiet
        variables x(Ns*(Np+1)) u(Nc*Np)
        J = x(1:Ns)'*P*x(1:Ns);
        for k = 1:Np
            x_k = x(Ns*(k-1)+1:Ns*k);
            u_k = u(Nc*(k-1)+1:Nc*k);
            J = J + x_k'*Q*x_k + u_k'*R*u_k;
            x_km1 = x(Ns*k+1:Ns*(k+1));
            x_kp1 = A*x_k + B*u_k;
            J = J + x_kp1'*P*x_kp1;
            subject to
                x_km1 == x_kp1;
        end
        x1 = x(1:Ns:end);
        x2 = x(2:Ns:end);
        u1 = u(1:Nc:end);
        u2 = u(2:Nc:end);
        subject to
            x(1:Ns) == x0;
            -1 <= u1 <= 1;
            -1 <= u2 <= 1;
            -10 <= x1 <= 10;
            -10 <= x2 <= 10;
    cvx_end
end



