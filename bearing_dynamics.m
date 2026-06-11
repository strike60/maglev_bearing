%% bearing_dynamics.m
% 单磁轴承动力学仿真
% 模型描述：
%   一个转子（质量块）在竖直方向受到：
%     1. 重力 (mg) - 向下
%     2. 弹簧弹力 (k_spring * x) - 用于缓解重力，使平衡点位于气隙中点附近
%     3. 电磁力 (F_em) - 由上下差动电磁铁产生，用于主动控制
%
% 动力学方程：
%   m * x_ddot = -k_spring * x + F_em - m * g
%
% 其中 x 为转子偏离机械中点的位移（向上为正）
% 电磁力采用 Frolich 磁饱和模型计算

clear; clc; close all;

%% ==================== 物理参数 ====================
% --- 磁路参数（与 mag_bearing.m 一致）---
mu_0 = 4 * pi * 1e-7;          % 真空磁导率 [H/m]
mu_r_init = 5000;               % 相对初始磁导率
mu_init = mu_r_init * mu_0;     % 绝对初始磁导率 [H/m]
B_sat = 1.9;                    % 饱和磁通密度 [T]

% Frolich 参数
a_frolich = 1 / mu_init;        % [m/H]
b_frolich = 1 / B_sat;          % [1/T]

% 轴承几何参数
A_a = 1e-4;                     % 磁极有效截面积 [m^2]
A_iron = 1.2e-4;                % 铁芯截面积 [m^2]
l_iron = 0.15;                  % 铁芯磁路长度 [m]
N_coil = 200;                   % 线圈匝数
alpha = 30 * pi / 180;          % 磁极倾角 [rad]
x_0 = 0.5e-3;                   % 标称气隙 [m]

% --- 机械参数 ---
m = 2.0;                        % 转子质量 [kg]
g = 9.81;                       % 重力加速度 [m/s^2]

% 弹簧参数：用于缓解重力，使平衡点位于 x = 0 附近
% 在平衡点 (x=0, i1=i2=i_bias)，电磁力净力为 0
% 弹簧弹力需平衡重力：k_spring * x_eq = m * g
% 取 x_eq = 0（平衡在机械中点），则弹簧在 x=0 时提供弹力 = m*g
% 实际上弹簧有一个预压缩量 delta_0，满足 k_spring * delta_0 = m*g
% 当转子偏离平衡位置 x 时，弹簧力为 k_spring * (delta_0 - x) = m*g - k_spring * x
% 因此动力学方程中弹簧项为 -k_spring * x（相对于平衡点的增量）
k_spring = 5000;                % 弹簧刚度 [N/m]

% 偏置电流
i_bias = 3.0;                   % 偏置电流 [A]

fprintf('========== 磁轴承动力学仿真参数 ==========\n');
fprintf('转子质量 m = %.2f kg\n', m);
fprintf('重力 mg = %.2f N\n', m * g);
fprintf('弹簧刚度 k_spring = %.1f N/m\n', k_spring);
fprintf('弹簧预压缩力 = %.2f N (平衡重力)\n', k_spring * (m*g/k_spring));
fprintf('偏置电流 i_bias = %.1f A\n', i_bias);
fprintf('标称气隙 x_0 = %.3f mm\n', x_0 * 1e3);
fprintf('\n');
%% ==================== 电磁力计算函数 ====================
dt = 1e-5;
% 使用 Frolich 磁饱和模型计算单侧电磁力
% 输入: i - 线圈电流 [A], h - 气隙长度 [m]
% 输出: F - 电磁力 [N], Phi - 磁通 [Wb]
calc_force = @(i, h) calc_electromagnetic_force(i, h, a_frolich, b_frolich, ...
                                                  mu_0, A_a, A_iron, l_iron, ...
                                                  N_coil, alpha);

% 电磁力计算函数
[calc_net_force, calc_net_force2, reset_filters] = ...
    create_differential_force_calculator(x_0, calc_force, dt, i_bias, false);
%% ==================== 验证电磁力特性 ====================
fprintf('========== 电磁力特性验证 ==========\n');

% 在平衡点附近线性化
x_test = 0;
i_ctrl_test = linspace(-2, 2, 50);
F_net_test = zeros(size(i_ctrl_test));

for k = 1:length(i_ctrl_test)
    F_net_test(k) = calc_net_force(i_ctrl_test(k), x_test);
end

% 线性拟合求力-电流刚度 k_i
p = polyfit(i_ctrl_test, F_net_test, 1);
k_i = p(1);  % 力-电流刚度 [N/A]
fprintf('在 x=0 处，力-电流刚度 k_i = %.2f N/A\n', k_i);

% 计算力-位移刚度 k_x（在 i_ctrl=0 处）
x_test2 = linspace(-0.2e-3, 0.2e-3, 50);
F_net_test2 = zeros(size(x_test2));

for k = 1:length(x_test2)
    F_net_test2(k) = calc_net_force(0, x_test2(k));
end
p2 = polyfit(x_test2, F_net_test2, 1);
k_x = p2(1);  % 力-位移刚度 [N/m]
fprintf('在 i_ctrl=0 处，力-位移刚度 k_x = %.2f N/m\n', k_x);

% 自然频率
omega_n = sqrt((k_spring - k_x) / m);  % 注意：负刚度会降低等效刚度
fprintf('系统自然频率 f_n = %.2f Hz\n', omega_n / (2*pi));
fprintf('\n');

%% ==================== 绘制电磁力特性曲线 ====================
figure('Position', [50, 50, 1200, 500]);

% 子图1：净力 vs 控制电流（不同位移）
subplot(1, 2, 1);
x_disps = [-0.2, -0.1, 0, 0.1, 0.2] * 1e-3;
i_ctrl_plot = linspace(-3, 3, 100);
colors = lines(length(x_disps));

for k = 1:length(x_disps)
    F_plot = zeros(size(i_ctrl_plot));
    for n = 1:length(i_ctrl_plot)
        F_plot(n) = calc_net_force(i_ctrl_plot(n), x_disps(k));
    end
    plot(i_ctrl_plot, F_plot, 'Color', colors(k, :), 'LineWidth', 1.5); hold on;
end
xlabel('Control Current i_{ctrl} [A]', 'FontSize', 12);
ylabel('Net Force F_{net} [N]', 'FontSize', 12);
title('Net Force vs Control Current', 'FontSize', 13);
legend(arrayfun(@(x) sprintf('x = %.2f mm', x*1e3), x_disps, ...
       'UniformOutput', false), 'Location', 'northwest');
grid on;
xlim([-3, 3]);

% 子图2：净力 vs 位移（不同控制电流）
subplot(1, 2, 2);
i_ctrl_fixed = [-2, -1, 0, 1, 2];
x_plot = linspace(-0.3e-3, 0.3e-3, 100);
for k = 1:length(i_ctrl_fixed)
    F_plot = zeros(size(x_plot));
    for n = 1:length(x_plot)
        F_plot(n) = calc_net_force(i_ctrl_fixed(k), x_plot(n));
    end
    plot(x_plot * 1e3, F_plot, 'Color', colors(k, :), 'LineWidth', 1.5); hold on;
end
xlabel('Displacement x [mm]', 'FontSize', 12);
ylabel('Net Force F_{net} [N]', 'FontSize', 12);
title('Net Force vs Displacement', 'FontSize', 13);
legend(arrayfun(@(i) sprintf('i_{ctrl} = %d A', i), i_ctrl_fixed, ...
       'UniformOutput', false), 'Location', 'northwest');
grid on;

sgtitle('Differential Electromagnetic Force Characteristics', ...
        'FontSize', 14, 'FontWeight', 'bold');
%% ==================== 时域仿真设置 ====================
fprintf('========== 时域仿真 ==========\n');

% 仿真参数
t_end = 2.0;                    % 仿真时长 [s]
dt = 1e-5;                      % 仿真步长 [s]（需足够小以捕捉快速动力学）
t_vec = 0:dt:t_end;
N_steps = length(t_vec);

t_vec_sim = t_vec(1:N_steps/2);
N_steps_sim = N_steps / 2;

% 状态初始化
x = 0;                          % 初始位移 [m]
x_prev = 0;                     % 初始位移 [m]
x_ref_prev = 0;                 % 初始参考位移 [m]
x_ref_dot_prev = 0;             % 初始参考速度 [m/s]
v = 0;                          % 初始速度 [m/s]
F_desired_max = 0;              % 最大电磁力 [N]
F_desired_min = 0;              % 最小电磁力 [N]

% 记录数组
x_rec = zeros(1, N_steps);
v_rec = zeros(1, N_steps);
i_ctrl_rec = zeros(1, N_steps);
F_em_rec = zeros(1, N_steps);
F_desired_rec = zeros(1, N_steps);
F_max_rt_rec = zeros(1, N_steps);
F_min_rt_rec = zeros(1, N_steps);
F_desired_rec = zeros(1, N_steps);
F_spring_rec = zeros(1, N_steps);
F_gravity_rec = zeros(1, N_steps);

% 记录初始状态
x_rec(1) = x;
v_rec(1) = v;
%% ==================== 电磁力计算函数 ====================
% 使用 Frolich 磁饱和模型计算单侧电磁力
% 输入: i - 线圈电流 [A], h - 气隙长度 [m]
% 输出: F - 电磁力 [N], Phi - 磁通 [Wb]
calc_force = @(i, h) calc_electromagnetic_force(i, h, a_frolich, b_frolich, ...
                                                  mu_0, A_a, A_iron, l_iron, ...
                                                  N_coil, alpha);

% 电磁力计算函数
[calc_net_force, calc_net_force2, reset_filters] = ...
    create_differential_force_calculator(x_0, calc_force, dt, i_bias, true);
%% ==================== 参考轨迹 ====================
mode = 'square';

switch mode
    case 'step'
        % 选项 1: 阶跃响应
        x_ref = 0.1e-3 * ones(1, N_steps);  % 参考位置 0.1 mm
        x_ref_dot = zeros(1, N_steps);
    case 'sin'
        % 选项 2: 正弦跟踪
        x_ref = 0.1e-3 * sin(2 * pi * 10 * t_vec);  % 10 Hz 正弦，幅值 0.1 mm
        x_ref_dot = 0.1e-3 * 2 * pi * 10 * cos(2 * pi * 10 * t_vec); 
    case 'square'
        % 选项 3: 方波
        x_ref = 0.1e-3 * square(2 * pi * 5 * t_vec, 50);
        x_ref_dot = zeros(1, N_steps);
    case 'zero'
        % 选项 4: 保持零点（抗扰动）
        x_ref = zeros(1, N_steps);
        x_ref_dot = zeros(1, N_steps);
    otherwise
        error('Unknown mode: %s', mode);
end
x = x_ref(1);
v = x_ref_dot(1);

%% ==================== 控制器设计（PID） ====================
% 为了进行有意义的动力学仿真，设计一个简单的 PID 控制器
% 控制目标：使转子跟踪参考位置 x_ref

% PID 参数（需根据系统特性整定）
Kp = 200000;      % 比例增益 [N/m]
Ki = 50000;      % 积分增益 [N/(m*s)]
Kd = 1000;        % 微分增益 [N/(m/s)]

% 积分器状态
integral_error = 0;

% 上一时刻误差（用于微分项）
prev_error = 0;

fprintf('PID 控制器参数:\n');
fprintf('  Kp = %.1f N/m\n', Kp);
fprintf('  Ki = %.1f N/(m*s)\n', Ki);
fprintf('  Kd = %.1f N/(m/s)\n', Kd);
fprintf('仿真时长: %.2f s, 步长: %.2e s\n', t_end, dt);
%% ==================== 控制器设计（STEPBACK）====================
stepback_k_1 = 200;
stepback_k_2 = 100;

%% ==================== 控制器设计（MPC）====================
mpc_k = k_spring;
mpc_b = 0;
mpc_m = m;
mpc_Ts = 0.001;

mpc_s = mpc_k / mpc_m;
mpc_c = mpc_b / mpc_m;

mpc_A = [0 1; -mpc_s -mpc_c];
mpc_B = [0; 1];

mpc_A_d = eye(2) + mpc_A * mpc_Ts;
mpc_B_d = mpc_B * mpc_Ts;

mpc_N = 10;

mpc_M_x = [];
for i = 1:mpc_N
    mpc_M_x = [mpc_M_x; mpc_A_d^i];
end

mpc_M_ud = [];
for i = 1:mpc_N
    row = [];
    for j = 1:mpc_N
        if i >= j
            row = [row, mpc_A_d^(i-j) * mpc_B_d];
        else
            row = [row, zeros(2, 1)];
        end
    end
    mpc_M_ud = [mpc_M_ud; row];
end

mpc_x_0 = [0; 0];

mpc_Q = [1e20 0; 0 1e3];
mpc_Q_bar = kron(eye(mpc_N), mpc_Q);

mpc_R = 1e6;
mpc_R_bar = kron(eye(mpc_N), mpc_R);

mpc_S = 1e4;
assert(mpc_R > mpc_S * (cos(pi / (mpc_N + 1)) - 1));

mpc_D = zeros(mpc_N - 1, mpc_N);
for i = 1:mpc_N-1
    for j = 1:mpc_N
        if i == j
            mpc_D(i, j) = 1;
        elseif j == i + 1
            mpc_D(i, j) = -1;
        end
    end
end

mpc_S_bar = kron(mpc_D'*mpc_D, mpc_S) + mpc_R_bar;

mpc_X_r = [x_ref(1:100:end); x_ref_dot(1:100:end)];
mpc_X_r_mat = reshape(mpc_X_r, 2, []);

mpc_H = mpc_M_ud' * mpc_Q_bar * mpc_M_ud + mpc_S_bar;
mpc_H = (mpc_H + mpc_H') / 2;
% mpc_f = 2 * mpc_M_ud' * mpc_Q_bar * (mpc_M_x * mpc_x_0 - mpc_X_r);

mpc_u_max = 25;
mpc_v_max = 100;

mpc_a_1 = [eye(mpc_N); -1 * eye(mpc_N)];
mpc_b_1 = [ones(mpc_N, 1); ones(mpc_N, 1)] * mpc_u_max;

mpc_C = zeros(mpc_N, 2*mpc_N);
for i = 1:mpc_N
    for j = 1:2*mpc_N
        if j == 2*i
            mpc_C(i, j) = 1;
        end
    end
end

mpc_a_2 = mpc_C * mpc_M_ud;
mpc_b_2 = mpc_C * ones(2*mpc_N, 1) * mpc_v_max - mpc_C * mpc_M_x * mpc_x_0;

mpc_a_3 = -mpc_C * mpc_M_ud;
mpc_b_3 = mpc_C * ones(2*mpc_N, 1) * mpc_v_max + mpc_C * mpc_M_x * mpc_x_0;

mpc_a = [mpc_a_1; mpc_a_2; mpc_a_3];
mpc_b = [mpc_b_1; mpc_b_2; mpc_b_3];

mpc_options = optimoptions('quadprog', 'Display', 'off', ...
                       'Algorithm', 'interior-point-convex', ...
                       'MaxIterations', 100);

mpc_M_udQ = mpc_M_ud' * mpc_Q_bar;
mpc_warm = [];

%% ==================== 主仿真循环 ====================
fprintf('正在仿真...\n');
tic;

force_cal_methods = 'pi';

force_2_current_methods = 'single';

% 计算 maglev_bearing_control 中公式（2）的 k_0
% k_0 = mu_0 * A_a * N^2 * cos(alpha) / 8
k_0 = mu_0 * A_a * N_coil^2 * cos(alpha) / 8;
fprintf('电磁力系数 k_0 = %.4e N·m^2/A^2\n', k_0);

% 控制频率设定 0.001s（1 kHz）
control_dt = 0.001;                     % 控制周期 [s]
control_step_interval = round(control_dt / dt);  % 每多少仿真步执行一次控制
fprintf('控制周期: %.4f s (%d 个仿真步)\n', control_dt, control_step_interval);

reset_filters();
for k = 1:N_steps_sim - 1
    % --- 计算控制量(控制频率设定0.001s) ---
    % 每 control_step_interval 个仿真步更新一次控制量
    if mod(k - 1, control_step_interval) == 0
        switch force_cal_methods
            case 'pi'
                error = x_ref(k) - x;
                integral_error = integral_error + error * control_dt;  % 使用控制周期积分
                % 微分项：当前误差与上一时刻误差之差 / 控制周期
                derivative_error = (error - prev_error) / control_dt;
                prev_error = error;  % 更新上一时刻误差

                F_desired_max = 4 * k_0 * i_bias * i_bias / (x_0 - x) / (x_0 - x);
                F_desired_min = -4 * k_0 * i_bias * i_bias / (x_0 + x) / (x_0 + x);

                % PID 输出（期望的净电磁力）
                F_desired = k_spring * x + Kp * error + Ki * integral_error + Kd * derivative_error;

                F_desired = max(min(F_desired, F_desired_max), F_desired_min);
            case 'stepback'
                % 假设已有：
                % m, k, stepback_k_1, stepback_k_2 (且 stepback_k_2 > stepback_k_1 > 0)
                % control_dt, x_ref(k), x_ref_prev, x_ref_dot_prev, x_prev, x

                stepback_z_1 = x - x_ref(k);
                x_ref_d     = (x_ref(k) - x_ref_prev) / control_dt;
                x_ref_dd    = (x_ref_d - x_ref_dot_prev) / control_dt;   % 参考加速度

                stepback_z_2 = (x - x_prev) / control_dt + stepback_k_1 * stepback_z_1 - x_ref_d;

                F_desired_max = 4 * k_0 * i_bias * i_bias / (x_0 - x) / (x_0 - x);
                F_desired_min = -4 * k_0 * i_bias * i_bias / (x_0 + x) / (x_0 + x);

                % 控制律
                F_desired = k_spring * x + m * x_ref_dd ...
                            - m * (stepback_k_2 + stepback_k_1) * stepback_z_2 ...
                            - m * (1 - stepback_k_1^2) * stepback_z_1;

                F_desired = max(min(F_desired, F_desired_max), F_desired_min);

                % 更新历史值
                x_ref_prev      = x_ref(k);
                x_ref_dot_prev  = x_ref_d;
                x_prev          = x;
            case 'mpc'
                % 当前的初始值
                x_cur = [x; v];

                % 提取当前及未来 N 步参考位置和速度
                k_jump = (k - 1) / control_step_interval;
                ref_window = mpc_X_r_mat(:, k_jump + 1: k_jump + mpc_N);
                ref_vec = ref_window(:);

                % 计算f
                mpc_f = 2 * mpc_M_udQ * (mpc_M_x * x_cur - ref_vec);

                % 更新约束

                mpc_u_max_rt = 4 * k_0 * i_bias * i_bias ./...
                 (ones(mpc_N, 1) * (x_0 - x)) ./...
                  (ones(mpc_N, 1) * (x_0 - x)); 
                mpc_u_min_rt = 4 * k_0 * i_bias * i_bias ./...
                 (ones(mpc_N, 1) * (x_0 + x)) ./...
                  (ones(mpc_N, 1) * (x_0 + x));

                F_desired_max = mpc_u_max_rt(1);
                F_desired_min = -mpc_u_min_rt(1);

                mpc_b_1 = [mpc_u_max_rt; mpc_u_min_rt];

                mpc_b_2 = mpc_C * ones(2*mpc_N, 1) * mpc_v_max - mpc_C * mpc_M_x * x_cur;
                mpc_b_3 = mpc_C * ones(2*mpc_N, 1) * mpc_v_max + mpc_C * mpc_M_x * x_cur;
                mpc_b = [mpc_b_1; mpc_b_2; mpc_b_3];

                % 求解 QP
                mpc_u = quadprog(mpc_H, mpc_f, mpc_a, mpc_b, [], [], [], [], mpc_warm, mpc_options);
                mpc_warm = mpc_u;
                F_desired = mpc_u(1);
        end
    end
    switch force_2_current_methods
        case 'diff'
            % 方法一：差动方法：将期望力映射为控制电流（使用逆模型或近似线性关系）
            % 在平衡点附近：F_net ≈ k_i * i_ctrl
            i_ctrl = F_desired / k_i;
            i_ctrl = max(min(i_ctrl, i_bias), -i_bias);
            F_em = calc_net_force(i_ctrl, x);
        case 'single'
            if F_desired >= 0
                i_ctrl = sqrt(F_desired / k_0) * (x_0 - x);
                % 限幅
                i_ctrl = max(min(i_ctrl, 2*i_bias), 0);
                F_em = calc_net_force2(i_ctrl, 0, x);
            else
                i_ctrl = sqrt(F_desired / -k_0) * (x_0 + x);
                % 限幅
                i_ctrl = max(min(i_ctrl, 2*i_bias), 0);
                F_em = calc_net_force2(0, i_ctrl, x);
            end
        case 'none'
            i_ctrl = 0;
            F_em = F_desired;
    end
    
    % 控制量保持不变（零阶保持器），直到下一个控制周期
    
    % --- 计算弹簧力（相对于平衡点的增量）---
    % 弹簧总力 = m*g - k_spring * x
    % 其中 m*g 为预压缩力，-k_spring*x 为偏离平衡点产生的恢复力
    F_spring = -k_spring * x;  % 相对于平衡点的弹簧力增量
    
    % --- 重力（常数，已被弹簧预压缩平衡）---
    % 在动力学方程中，重力已被弹簧预压缩抵消
    % 因此净外力 = F_em + F_spring
    
    % --- 动力学更新（半隐式欧拉法）---
    a_net = (F_em + F_spring) / m;  % 加速度 [m/s^2]
    v = v + a_net * dt;
    x = x + v * dt;
    
    % --- 限制位移不能超过 +-x_0 ---
    if x > x_0
        x = x_0;
        v = 0;  % 碰撞后速度归零（完全非弹性碰撞）
    elseif x < -x_0
        x = -x_0;
        v = 0;  % 碰撞后速度归零（完全非弹性碰撞）
    end
    
    % --- 记录数据 ---
    x_rec(k+1) = x;
    v_rec(k+1) = v;
    i_ctrl_rec(k+1) = i_ctrl;
    F_em_rec(k+1) = F_em;
    F_desired_rec(k+1) = F_desired;
    F_max_rt_rec(k+1) = F_desired_max;
    F_min_rt_rec(k+1) = F_desired_min;
    F_spring_rec(k+1) = F_spring;
    F_gravity_rec(k+1) = -m * g;
end

sim_time = toc;
fprintf('仿真完成，耗时 %.2f 秒\n', sim_time);

%% ==================== 仿真结果绘图 ====================
figure('Position', [50, 50, 1400, 900]);

% 子图1：位移跟踪
subplot(3, 2, 1);
plot(t_vec, x_ref * 1e3, 'b--', 'LineWidth', 1.5); hold on;
plot(t_vec, x_rec * 1e3, 'r-', 'LineWidth', 1.5);
xlim([0 t_vec_sim(end)]);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Displacement x [mm]', 'FontSize', 12);
title('Position Tracking', 'FontSize', 13);
legend('Reference', 'Actual', 'Location', 'best');
grid on;

% 子图2：跟踪误差
subplot(3, 2, 2);
error_rec = (x_ref - x_rec) * 1e3;
plot(t_vec, error_rec, 'k-', 'LineWidth', 1.5);
xlim([0 t_vec_sim(end)]);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Error [mm]', 'FontSize', 12);
title('Tracking Error', 'FontSize', 13);
grid on;

% 子图3：速度
subplot(3, 2, 3);
plot(t_vec, v_rec * 1e3, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_vec, x_ref_dot * 1e3, 'r-', 'LineWidth', 1.5);
xlim([0 t_vec_sim(end)]);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Velocity [mm/s]', 'FontSize', 12);
title('Rotor Velocity', 'FontSize', 13);
grid on;

% 子图4：控制电流
subplot(3, 2, 4);
plot(t_vec, i_ctrl_rec, 'g-', 'LineWidth', 1.5);
xlim([0 t_vec_sim(end)]);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Control Current i_{ctrl} [A]', 'FontSize', 12);
title('Control Current', 'FontSize', 13);
grid on;
ylim([-7, 7]);

% 子图5：各力分量
subplot(3, 2, 5);
plot(t_vec, F_em_rec, 'r-', 'LineWidth', 1.5); hold on;
plot(t_vec, F_spring_rec, 'b-', 'LineWidth', 1.5);
plot(t_vec, F_em_rec + F_spring_rec, 'k--', 'LineWidth', 1.5);
plot(t_vec, F_desired_rec, 'm-', 'LineWidth', 1.5);
plot(t_vec, F_max_rt_rec, 'g--', 'Linewidth', 1.5);
plot(t_vec, F_min_rt_rec, 'g--', 'Linewidth', 1.5);

xlim([0 t_vec_sim(end)]);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Force [N]', 'FontSize', 12);
title('Force Components', 'FontSize', 13);
legend('F_{em}', 'F_{spring}', 'F_{net}', 'F_{desired}', 'Location', 'best');
grid on;

% 子图6：相图
subplot(3, 2, 6);
plot(x_rec * 1e3, v_rec * 1e3, 'b-', 'LineWidth', 1.5);
xlabel('Displacement x [mm]', 'FontSize', 12);
ylabel('Velocity v [mm/s]', 'FontSize', 12);
title('Phase Portrait', 'FontSize', 13);
grid on;
axis equal;

sgtitle('Magnetic Bearing Dynamics Simulation (with Frolich Saturation Model)', ...
        'FontSize', 15, 'FontWeight', 'bold');

%% ==================== 抗扰动测试（可选） ====================
% 如需测试抗扰动能力，可取消注释以下代码块
% 
% fprintf('\n========== 抗扰动测试 ==========\n');
% 
% % 重新初始化
% x = 0;
% v = 0;
% integral_error = 0;
% x_ref = zeros(1, N_steps);
% 
% % 在 0.2s 时施加脉冲扰动
% disturbance = zeros(1, N_steps);
% dist_idx = round(0.2 / dt);
% disturbance(dist_idx) = 50;  % 50 N 的脉冲力 [N]
% 
% x_rec_d = zeros(1, N_steps);
% v_rec_d = zeros(1, N_steps);
% i_ctrl_rec_d = zeros(1, N_steps);
% 
% reset_filters();
% for k = 1:N_steps - 1
%     error = x_ref(k) - x;
%     integral_error = integral_error + error * dt;
%     F_desired = Kp * error + Ki * integral_error + Kd * (x_ref(k) - x) / dt;
%     i_ctrl = F_desired / k_i;
%     i_ctrl = max(min(i_ctrl, 3), -3);
%     
%     F_em = calc_net_force(i_ctrl, x);
%     F_spring = -k_spring * x;
%     
%     a_net = (F_em + F_spring + disturbance(k)) / m;
%     v = v + a_net * dt;
%     x = x + v * dt;
%     
%     x_rec_d(k+1) = x;
%     v_rec_d(k+1) = v;
%     i_ctrl_rec_d(k+1) = i_ctrl;
% end
% 
% figure('Position', [100, 100, 800, 600]);
% subplot(2, 1, 1);
% plot(t_vec, x_rec_d * 1e3, 'r-', 'LineWidth', 1.5);
% xlabel('Time [s]', 'FontSize', 12);
% ylabel('Displacement [mm]', 'FontSize', 12);
% title('Disturbance Rejection (50N impulse at t=0.2s)', 'FontSize', 13);
% grid on;
% 
% subplot(2, 1, 2);
% plot(t_vec, i_ctrl_rec_d, 'g-', 'LineWidth', 1.5);
% xlabel('Time [s]', 'FontSize', 12);
% ylabel('Control Current [A]', 'FontSize', 12);
% title('Control Current Response', 'FontSize', 13);
% grid on;

%% ==================== 辅助函数 ====================

function [F, Phi] = calc_electromagnetic_force(i, h, a, b, mu_0, A_a, ...
                                                A_iron, l_iron, N_coil, alpha)
    % 使用 Frolich 磁饱和模型计算单侧电磁力
    %
    % 输入:
    %   i - 线圈电流 [A]
    %   h - 气隙长度 [m]
    %   a, b - Frolich 参数
    %   mu_0 - 真空磁导率
    %   A_a - 磁极有效截面积 [m^2]
    %   A_iron - 铁芯截面积 [m^2]
    %   l_iron - 铁芯磁路长度 [m]
    %   N_coil - 线圈匝数
    %   alpha - 磁极倾角 [rad]
    %
    % 输出:
    %   F - 电磁力 [N]
    %   Phi - 磁通 [Wb]
    
    if i <= 0
        F = 0;
        Phi = 0;
        return;
    end
    
    % 求解非线性磁路方程得到磁通 Phi
    % H_iron * l_iron + 2*h * Phi / (mu_0 * A_a) = N * i
    % 其中 H_iron = a * (Phi/A_iron) / (1 - b * |Phi/A_iron|)
    
    fun = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                  + (2 * h * phi) / (mu_0 * A_a) - N_coil * i;
    
    % 磁通上限：Phi_max = B_sat * A_iron
    Phi_max = (1/b) * A_iron;
    
    try
        Phi = fzero(fun, [0, Phi_max * 0.99]);
    catch
        % 如果 fzero 失败，尝试从 0 开始搜索
        Phi = fzero(fun, [0, Phi_max]);
    end
    
    % 麦克斯韦应力张量法计算电磁力
    F = Phi^2 * cos(alpha) / (2 * mu_0 * A_a);
end

function [calc_net_force, calc_net_force2, reset_filters] = create_differential_force_calculator(x_0, calc_force, dt, i_bias, using_current_dynamic)
    % 创建差动电磁铁净力计算闭包，支持两种调用形式
    %
    % 输入:
    %   x_0        - 标称气隙 [m]
    %   calc_force - 单侧电磁力计算函数句柄: force = calc_force(i, gap)
    %   dt         - 采样时间 [s]
    %   i_bias     - 偏置电流 [A] (仅对传统接口 calc_net_force 需要)
    %
    % 输出:
    %   calc_net_force  - 函数句柄: F = calc_net_force(i_ctrl, x)
    %   calc_net_force2 - 函数句柄: F = calc_net_force2(i1, i2, x)
    %   reset_filters   - 重置函数句柄: reset_filters()
    
    % ---- 闭包内部状态（滤波器上一次输出值） ----
    i1_f_prev = 0;
    i2_f_prev = 0;
    
    % ---- 低通滤波器系数（固定） ----
    f_c = 800;
    tau = 1 / (2 * pi * f_c);
    alpha = dt / (dt + tau);
    
    % ---- 核心滤波函数（内部使用） ----
    function [i1_f, i2_f] = filter_currents(i1_raw, i2_raw)
        i1_f = (1 - alpha) * i1_f_prev + alpha * i1_raw;
        i2_f = (1 - alpha) * i2_f_prev + alpha * i2_raw;
        % 更新持久状态
        i1_f_prev = i1_f;
        i2_f_prev = i2_f;
    end
    
    % ---- 传统接口（内部实现，不与输出参数冲突） ----
    function F_net = compute_with_ctrl(i_ctrl, x)
        i1 = i_bias + i_ctrl;
        i2 = i_bias - i_ctrl;
        if (using_current_dynamic)
            [i1_f, i2_f] = filter_currents(i1, i2);
        else
            [i1_f, i2_f] = deal(i1, i2);
        end
        h1 = x_0 - x;
        h2 = x_0 + x;
        F1 = calc_force(i1_f, h1);
        F2 = calc_force(i2_f, h2);
        F_net = F1 - F2;
    end
    
    % ---- 新接口（内部实现） ----
    function F_net = compute_with_currents(i1, i2, x)
        if (using_current_dynamic)
            [i1_f, i2_f] = filter_currents(i1, i2);
        else
            [i1_f, i2_f] = deal(i1, i2);
        end
        h1 = x_0 - x;
        h2 = x_0 + x;
        F1 = calc_force(i1_f, h1);
        F2 = calc_force(i2_f, h2);
        F_net = F1 - F2;
    end
    
    % ---- 重置滤波器内部状态（内部实现） ----
    function reset_impl()
        i1_f_prev = 0;
        i2_f_prev = 0;
    end
    
    % ---- 将内部函数句柄赋给输出参数 ----
    calc_net_force = @compute_with_ctrl;
    calc_net_force2 = @compute_with_currents;
    reset_filters = @reset_impl;
end
