% mag_bearing.m
% Generate magnetic field using Frolich equation
% 
% This script implements the Frolich approximation for modeling
% magnetic saturation in a magnetic bearing system.
%
% Frolich equation: B = H / (a + b * |H|)
%   where a = 1/mu_init  (inverse of initial permeability)
%         b = 1/B_sat    (inverse of saturation flux density)
%
% The model transitions from linear (B ≈ mu_init * H) at low fields
% to saturation (B → B_sat) at high fields.

clear; clc; close all;

%% ==================== Parameters ====================
% Material: oriented silicon steel (取向硅钢片)
mu_0 = 4 * pi * 1e-7;          % Vacuum permeability [H/m]
mu_r_init = 5000;               % Relative initial permeability
mu_init = mu_r_init * mu_0;     % Absolute initial permeability [H/m]
B_sat = 1.9;                    % Saturation flux density [T]

% Frolich parameters
a = 1 / mu_init;                % [m/H]
b = 1 / B_sat;                  % [1/T]

% Bearing geometry
A_a = 1e-4;                     % Pole face effective area [m^2]
A_iron = 1.2e-4;                % Iron core cross-sectional area [m^2]
l_iron = 0.15;                  % Iron core magnetic path length [m]
N_coil = 200;                   % Number of coil turns
alpha = 30 * pi / 180;          % Pole angle from vertical [rad]
x_0 = 0.5e-3;                   % Nominal air gap [m]

% Operating range
H_max = 20000;                  % Max magnetic field strength [A/m]
H_points = 500;                 % Number of evaluation points

%% ==================== Frolich Magnetization Curve ====================
H = linspace(0, H_max, H_points)';
B_frolich = H ./ (a + b * H);

% Linear approximation (for comparison)
B_linear = mu_init * H;

% Display parameters
fprintf('=== Frolich Model Parameters ===\n');
fprintf('a = 1/mu_init = %.4f [m/H]\n', a);
fprintf('b = 1/B_sat   = %.4f [1/T]\n', b);
fprintf('mu_init = %.4f [H/m] (mu_r = %d)\n', mu_init, mu_r_init);
fprintf('B_sat   = %.2f [T]\n', B_sat);
fprintf('\n');

% Verification points
test_H = [100, 1000, 10000]';
test_B = test_H ./ (a + b * test_H);
fprintf('Verification points:\n');
fprintf('  H = %6d A/m  ->  B = %.4f T\n', [test_H, test_B]');
fprintf('  H -> inf      ->  B = %.4f T (saturation limit)\n', 1/b);

%% ==================== Plot Magnetization Curve ====================
figure('Position', [100, 100, 800, 500]);
plot(H, B_frolich, 'b-', 'LineWidth', 2); hold on;
plot(H, B_linear, 'r--', 'LineWidth', 1.5);
yline(B_sat, 'k:', 'LineWidth', 1);
xlabel('Magnetic Field Strength H [A/m]', 'FontSize', 12);
ylabel('Magnetic Flux Density B [T]', 'FontSize', 12);
title('Frolich Approximation of Magnetization Curve', 'FontSize', 14);
legend('Frolich model', 'Linear (no saturation)', ...
       sprintf('B_{sat} = %.2f T', B_sat), ...
       'Location', 'southeast', 'FontSize', 11);
grid on;
xlim([0, H_max]);
ylim([0, B_sat * 1.1]);

%% ==================== Magnetic Circuit Analysis ====================
% For a given current i, compute the flux Phi and electromagnetic force F
% using the full nonlinear magnetic circuit model:
%
%   H_iron * l_iron + 2*h * Phi / (mu_0 * A_a) = N * i
%
% where H_iron = a * (Phi/A_iron) / (1 - b * |Phi/A_iron|)

i_range = linspace(0, 10, 200);        % Coil current range [A]
h_range = [0.3e-3, 0.5e-3, 0.8e-3];   % Air gap values [m]

colors = {'r', 'b', 'g', 'k'};
figure('Position', [100, 100, 1000, 800]);

% --- Subplot 1: Flux vs Current ---
subplot(2, 2, 1);
for k = 1:length(h_range)
    h = h_range(k);
    Phi = zeros(size(i_range));
    for n = 1:length(i_range)
        i = i_range(n);
        % Solve nonlinear equation for Phi using fzero
        fun = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                      + (2 * h * phi) / (mu_0 * A_a) - N_coil * i;
        Phi(n) = fzero(fun, [0, B_sat * A_iron * 0.99]);
    end
    plot(i_range, Phi * 1e6, 'Color', colors{k}, 'LineWidth', 1.5); hold on;
end
xlabel('Current i [A]', 'FontSize', 12);
ylabel('Magnetic Flux \Phi [\muWb]', 'FontSize', 12);
title('Flux vs Current (with saturation)', 'FontSize', 13);
legend(arrayfun(@(h) sprintf('h = %.2f mm', h*1e3), h_range, ...
       'UniformOutput', false), 'Location', 'southeast');
grid on;

% --- Subplot 2: Force vs Current ---
subplot(2, 2, 2);
for k = 1:length(h_range)
    h = h_range(k);
    F = zeros(size(i_range));
    for n = 1:length(i_range)
        i = i_range(n);
        fun = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                      + (2 * h * phi) / (mu_0 * A_a) - N_coil * i;
        phi_val = fzero(fun, [0, B_sat * A_iron * 0.99]);
        % Maxwell stress tensor: F = Phi^2 * cos(alpha) / (2 * mu_0 * A_a)
        F(n) = phi_val^2 * cos(alpha) / (2 * mu_0 * A_a);
    end
    plot(i_range, F, 'Color', colors{k}, 'LineWidth', 1.5); hold on;
end
xlabel('Current i [A]', 'FontSize', 12);
ylabel('Electromagnetic Force F [N]', 'FontSize', 12);
title('Force vs Current (with saturation)', 'FontSize', 13);
legend(arrayfun(@(h) sprintf('h = %.2f mm', h*1e3), h_range, ...
       'UniformOutput', false), 'Location', 'northwest');
grid on;

% --- Subplot 3: Force vs Air Gap ---
subplot(2, 2, 3);
i_fixed = [2, 5, 8];  % Fixed current values [A]
h_scan = linspace(0.2e-3, 1.0e-3, 100);
for k = 1:length(i_fixed)
    i = i_fixed(k);
    F = zeros(size(h_scan));
    for n = 1:length(h_scan)
        h = h_scan(n);
        fun = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                      + (2 * h * phi) / (mu_0 * A_a) - N_coil * i;
        phi_val = fzero(fun, [0, B_sat * A_iron * 0.99]);
        F(n) = phi_val^2 * cos(alpha) / (2 * mu_0 * A_a);
    end
    plot(h_scan * 1e3, F, 'Color', colors{k}, 'LineWidth', 1.5); hold on;
end
xlabel('Air Gap h [mm]', 'FontSize', 12);
ylabel('Electromagnetic Force F [N]', 'FontSize', 12);
title('Force vs Air Gap (with saturation)', 'FontSize', 13);
legend(arrayfun(@(i) sprintf('i = %d A', i), i_fixed, ...
       'UniformOutput', false), 'Location', 'northeast');
grid on;

% --- Subplot 4: Equivalent permeability vs H ---
subplot(2, 2, 4);
mu_equiv = B_frolich ./ H;  % Equivalent permeability mu = B/H
mu_r_equiv = mu_equiv / mu_0;
plot(H, mu_r_equiv, 'm-', 'LineWidth', 2);
xlabel('Magnetic Field Strength H [A/m]', 'FontSize', 12);
ylabel('Relative Permeability \mu_r', 'FontSize', 12);
title('Equivalent Permeability vs H', 'FontSize', 13);
grid on;
xlim([0, H_max]);
ylim([0, mu_r_init * 1.1]);
yline(mu_r_init, 'k--', sprintf('\\mu_{r,init} = %d', mu_r_init), ...
      'LabelHorizontalAlignment', 'left');

sgtitle('Magnetic Bearing Analysis with Frolich Saturation Model', ...
        'FontSize', 15, 'FontWeight', 'bold');

%% ==================== Differential Force (Push-Pull) ====================
% For a differential electromagnetic actuator (push-pull configuration):
%   F_net = F(i1, h1) - F(i2, h2)
%   where h1 = x_0 - x, h2 = x_0 + x
%   and typically i1 = i_bias + i_control, i2 = i_bias - i_control

i_bias = 3;                         % Bias current [A]
i_ctrl = linspace(-3, 3, 100);      % Control current range [A]
x_disp = [0, 0.1e-3, 0.2e-3, 0.4e-3];      % Rotor displacement [m]

figure('Position', [100, 100, 800, 500]);
for k = 1:length(x_disp)
    x = x_disp(k);
    h1 = x_0 - x;
    h2 = x_0 + x;
    F_net = zeros(size(i_ctrl));
    for n = 1:length(i_ctrl)
        i1 = i_bias + i_ctrl(n);
        i2 = i_bias - i_ctrl(n);
        
        % Force from upper electromagnet
        fun1 = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                       + (2 * h1 * phi) / (mu_0 * A_a) - N_coil * i1;
        if i1 > 0
            phi1 = fzero(fun1, [0, B_sat * A_iron * 0.99]);
        else
            phi1 = 0;
        end
        F1 = phi1^2 * cos(alpha) / (2 * mu_0 * A_a);
        
        % Force from lower electromagnet
        fun2 = @(phi) (a * l_iron * (phi / A_iron)) / (1 - b * abs(phi / A_iron)) ...
                       + (2 * h2 * phi) / (mu_0 * A_a) - N_coil * i2;
        if i2 > 0
            phi2 = fzero(fun2, [0, B_sat * A_iron * 0.99]);
        else
            phi2 = 0;
        end
        F2 = phi2^2 * cos(alpha) / (2 * mu_0 * A_a);
        
        F_net(n) = F1 - F2;
    end
    plot(i_ctrl, F_net, 'Color', colors{k}, 'LineWidth', 1.5); hold on;
end
xlabel('Control Current i_{ctrl} [A]', 'FontSize', 12);
ylabel('Net Force F_{net} [N]', 'FontSize', 12);
title('Differential Actuator Force (Push-Pull)', 'FontSize', 14);
legend(arrayfun(@(x) sprintf('x = %.2f mm', x*1e3), x_disp, ...
       'UniformOutput', false), 'Location', 'northwest');
grid on;

fprintf('\n=== Differential Actuator ===\n');
fprintf('Bias current: i_bias = %.1f A\n', i_bias);
fprintf('Nominal air gap: x_0 = %.2f mm\n', x_0 * 1e3);
fprintf('Force-displacement stiffness at zero: nonlinear (saturation-dependent)\n');
