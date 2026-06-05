%% state space
format long;
b = 0.0001;
j = 0.001;
Ts = 0.0001;

c = b / j;
A = [0 1; 0 -c];
B = [0; 1];
C = [1 0];
D = 0;

A_d = eye(2) + A * Ts;
B_d = B * Ts;
C_d = C;
D_d = D;

plant = ss(A_d, B_d, C_d, D_d, Ts);

%% construct M_x, M_ud, x_0, \bar Q, \bar S, H and f
N = 10; % horizon

M_x = [];
for i = 1:N     
    M_x = [M_x; A_d^i];
end

% Construct M_ud as a block lower-triangular Toeplitz matrix
% M_ud = [B_d      0       ...  0
%         A_d*B_d  B_d     ...  0
%         ...      ...     ...  ...
%         A_d^(N-1)*B_d  A_d^(N-2)*B_d  ...  B_d]
M_ud = [];
for i = 1:N
    row = [];
    for j = 1:N
        if i >= j
            row = [row, A_d^(i-j) * B_d];
        else
            row = [row, zeros(2, 1)];
        end
    end
    M_ud = [M_ud; row];
end

% Construct x_0
x_0 = [0; 0];

% Construct \bar Q
Q = [10000000 10; 10 100];
Q_bar = kron(eye(N), Q);

% Construct \bar R
R = 1e-2;
R_bar = kron(eye(N), R);

% Construct \bar S
S = 1e-4;
assert(R > S * (cos(pi / (N + 1)) - 1));

% Construct a N-1 x N differential Matrix
D = zeros(N - 1, N);
for i = 1:N-1
    for j = 1:N
        if i == j
            D(i, j) = 1;
        elseif j == i + 1
            D(i, j) = -1;
        end
    end
end

S_bar = kron(D'*D, S) + R_bar;

% Construct X_r
X_r = ones(2 * N, 1);

% Construct H
H = M_ud' * Q_bar * M_ud + S_bar;
H = (H + H') / 2;
f = 2 * M_ud' * Q_bar * (M_x * x_0 - X_r);

%% Construct a and b in a * u <= b
u_max = 100;
v_max = 10;

a_1 = [eye(N); -1 * eye(N)];
b_1 = [ones(N, 1); ones(N,1)] * u_max;

C = zeros(N, 2*N);
for i = 1:N
    for j = 1:2*N
        if j == 2*i
            C(i, j) = 1;
        end
    end
end

a_2 = C * M_ud;
b_2 = C * ones(2*N, 1) * v_max - C * M_x * x_0;

a_3 = -C * M_ud;
b_3 = C * ones(2*N, 1) * v_max + C * M_x * x_0;

a = [a_1; a_2; a_3];
b = [b_1; b_2; b_3];

%% solve constrain QP
mode = 'sin';
disturb = false;

step_num = 3e4;
time = 0:1:step_num-1+10;
time = time * Ts;

% 轨迹参数
P_end = 10;     % 终点位置 (m)
T_total = 5;    % 总运动时间 (s)

p1 = -2 * P_end / T_total^3;
p2 = 3 * P_end / T_total^2;

switch mode
    case 'poly'
        x_0 = [0; 0.5];
        x_r = p1 * time.^3 + p2 * time.^2;
        x_r_dot = 3 * p1 * time.^2 + 2 * p2 * time;
    case 'sin'
        x_0 = [0; 1 * pi];
        x_r = 1 * sin(2 * pi * time);
        x_r_dot = 2 * pi * cos(2 * pi * time);
    case 'step'
        x_0 = [0; 0];
        x_r = 1 * ones(1, length(time));   % step from 0 to 1 at t=0
        x_r_dot = zeros(1, length(time));  % zero velocity reference
    otherwise
        error('Unknown mode: %s', mode);
end

X_r = [x_r; x_r_dot];

options = optimoptions('quadprog', 'Display', 'off', ...
                       'Algorithm', 'interior-point-convex', ...
                       'MaxIterations', 100);

warm = [];

% 重塑参考轨迹为矩阵
X_r_mat = reshape(X_r, 2, []);   % 2行：位置，速度

% 预分配记录
X_record = zeros(length(x_0), step_num);
u_record = zeros(1,step_num);
u_rel_record = zeros(1,step_num);

% ---------- 离线计算 ----------
M_udQ = M_ud' * Q_bar;

% ---------- 在线循环 ----------
x_cur = x_0;
for i = 1:step_num
    % 提取当前及未来 N 步参考
    ref_window = X_r_mat(:, i:i+N-1);
    ref_vec = ref_window(:);   % 列向量
    
    % 计算梯度
    f = 2 * M_udQ * (M_x * x_cur - ref_vec);
    
    % 求解 QP
    [u, ~, exitflag] = quadprog(H, f, a, b, [], [], [], [], warm, options);
    
    warm = u;   % 热启动
    
    % 系统更新
    if(disturb)
        x_next = A_d * x_cur + B_d * (u(1:1)  - 20 * sin(Ts * i * 2 * pi * 10));
    else
        x_next = A_d * x_cur + B_d * u(1:1);
    end
    X_record(:, i) = x_next;
    u_record(:, i) = u(1:1);
    if(disturb)
        u_rel_record(:,i) = u(1:1)  - 20 * sin(Ts * i * 2 * pi * 10);
    else
        u_rel_record(:,i) = u(1:1);
    end
    x_cur = x_next;
end

% plot the position, velocity comparation and u_record, u_rel_record (font type: times new roman)
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');

figure('Position', [100, 100, 800, 600]);

subplot(3, 1, 1);
plot(time(1:step_num), X_r(1, 1:step_num), 'b--', 'LineWidth', 1.5); hold on;
plot(time(1:step_num), X_record(1, :), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Position (m)', 'FontName', 'Times New Roman');
legend('Reference', 'MPC', 'FontName', 'Times New Roman');
title('Position Tracking', 'FontName', 'Times New Roman');
grid on;

subplot(3, 1, 2);
plot(time(1:step_num), X_r(2, 1:step_num), 'b--', 'LineWidth', 1.5); hold on;
plot(time(1:step_num), X_record(2, :), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Velocity (m/s)', 'FontName', 'Times New Roman');
legend('Reference', 'MPC', 'FontName', 'Times New Roman');
title('Velocity Tracking', 'FontName', 'Times New Roman');
grid on;

subplot(3, 1, 3);
plot(time(1:step_num), u_record, 'g-', 'LineWidth', 1.5); hold on;
plot(time(1:step_num), u_rel_record, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Control Input', 'FontName', 'Times New Roman');
legend('u (control)', 'u_{rel} (actual)', 'FontName', 'Times New Roman');
title('Control Input (u)', 'FontName', 'Times New Roman');
grid on;


