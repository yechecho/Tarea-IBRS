%% ANÁLISIS ADICIONAL: RESPUESTA A DIFERENTES ENTRADAS Y PARÁMETROS
% Extensión de la tarea: Análisis de sensibilidad de parámetros
%
% Se analiza cómo cambia la respuesta del circuito RLC al variar
% el factor de amortiguamiento (resistencia R).

clc; clear; close all;

fprintf('=======================================================\n');
fprintf('  ANÁLISIS DE SENSIBILIDAD - VARIACIÓN DE AMORTIGUAMIENTO\n');
fprintf('=======================================================\n\n');

L = 0.1;      % H
C = 0.001;    % F
wn = 1/sqrt(L*C);

fprintf('Parámetros fijos:\n');
fprintf('  L = %.3f H,  C = %.4f F\n', L, C);
fprintf('  ωn = %.2f rad/s\n\n', wn);

% Valores de R que dan distintos factores de amortiguamiento
zeta_values = [0.2, 0.5, 0.707, 1.0, 2.0];
R_values = 2 * zeta_values * sqrt(L/C);

t = linspace(0, 0.8, 2000);

%% --- Figura 1: Respuesta al escalón para distintos zeta ---
figure(1);
colores = {'b', 'g', 'r', 'm', 'k'};
hold on;

for i = 1:length(zeta_values)
    R = R_values(i);
    num_i = [1/(L*C)];
    den_i = [1, R/L, 1/(L*C)];
    H_i   = tf(num_i, den_i);
    [y_i, t_i] = step(H_i, t);
    plot(t_i, y_i, colores{i}, 'LineWidth', 2);
end

yline(1, 'k--', 'Referencia', 'LineWidth', 1, ...
      'LabelHorizontalAlignment', 'left');
grid on;
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Vc [V]', 'FontSize', 12);
title('Respuesta al Escalón - Diferentes Factores de Amortiguamiento', 'FontSize', 13);

leyendas = arrayfun(@(z, R) sprintf('ζ = %.3f (R=%.1fΩ)', z, R), ...
                    zeta_values, R_values, 'UniformOutput', false);
legend(leyendas, 'Location', 'best', 'FontSize', 9);
hold off;

%% --- Figura 2: Lugar de las raíces para variación de R ---
figure(2);
% Usamos el sistema de lazo cerrado con K variable equivalente a R/L
% Definir sistema base y trazar lugar de raíces
num_base = [1/(L*C)];
den_base = [1, 0, 1/(L*C)];  % s^2 + wn^2 (sin amortiguamiento)
H_base   = tf(num_base, den_base);

rlocus(tf([1 0], [1, 0, 1/(L*C)]));
grid on;
title('Lugar de las Raíces (variando R/L)', 'FontSize', 13);
xlabel('Parte Real', 'FontSize', 12);
ylabel('Parte Imaginaria', 'FontSize', 12);

%% --- Figura 3: Respuesta a señal senoidal ---
R = 10;  % Valor nominal
num = [1/(L*C)];
den = [1, R/L, 1/(L*C)];
H   = tf(num, den);
sys_ss = ss(H);

% Entrada senoidal a frecuencia natural
t_sin  = linspace(0, 0.5, 5000);
f_ent  = wn / (2*pi);                % Frecuencia de la entrada [Hz]
u_sin  = sin(wn * t_sin);            % Señal senoidal en ωn
[y_sin, ~, ~] = lsim(H, u_sin, t_sin);

figure(3);
plot(t_sin, u_sin, 'b--', 'LineWidth', 1.5); hold on;
plot(t_sin, y_sin, 'r-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Amplitud [V]', 'FontSize', 12);
title(sprintf('Respuesta a Entrada Senoidal en ωn = %.1f rad/s', wn), 'FontSize', 13);
legend({'Entrada vi(t)', 'Salida Vc(t)'}, 'Location', 'best');
hold off;

%% --- Figura 4: Respuesta al impulso ---
figure(4);
impulse(H);
grid on;
title('Respuesta al Impulso - Circuito RLC Serie', 'FontSize', 13);
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Vc [V]', 'FontSize', 12);

%% Tabla de métricas por valor de R
fprintf('Tabla de métricas por factor de amortiguamiento:\n');
fprintf('%-8s %-8s %-10s %-12s %-10s %-10s\n', ...
        'R [Ω]', 'ζ', 'Tp [ms]', 'Sobrepico%', 'Ts [ms]', 'Tr [ms]');
fprintf('%s\n', repmat('-', 1, 65));

for i = 1:length(zeta_values)
    R = R_values(i);
    num_i = [1/(L*C)];
    den_i = [1, R/L, 1/(L*C)];
    H_i   = tf(num_i, den_i);
    info  = stepinfo(H_i);
    if isnan(info.PeakTime)
        tp_str = '    N/A   ';
    else
        tp_str = sprintf('%-10.2f', info.PeakTime*1000);
    end
    fprintf('%-8.2f %-8.3f %s %-12.2f %-10.2f %-10.2f\n', ...
            R, zeta_values(i), tp_str, ...
            info.Overshoot, info.SettlingTime*1000, info.RiseTime*1000);
end

fprintf('\nAnálisis completado.\n');
