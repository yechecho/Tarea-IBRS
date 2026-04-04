%% TAREA DE MODELAMIENTO EN MATLAB - INGENIERÍA ELÉCTRICA
% Alumno: Estudiante de Ingeniería Eléctrica
% Curso: Modelamiento de Sistemas
%
% PROBLEMA:
% Se tiene un circuito RLC serie con los siguientes valores:
%   R = 10 Ohm  (Resistencia)
%   L = 0.1 H   (Inductancia)
%   C = 0.001 F (Capacitancia)
%
% Se pide:
%   1. Obtener la función de transferencia H(s) = Vc(s)/Vi(s)
%   2. Determinar polos, ceros y ganancia del sistema
%   3. Determinar tipo de respuesta (subamortiguada, crítica o sobreamortiguada)
%   4. Graficar la respuesta al escalón
%   5. Graficar el diagrama de Bode
%   6. Obtener la representación en espacio de estados
%   7. Verificar estabilidad del sistema

clc; clear; close all;

%% =========================================================
%  PARTE 1: PARÁMETROS DEL CIRCUITO RLC SERIE
%  =========================================================
fprintf('=======================================================\n');
fprintf(' TAREA DE MODELAMIENTO EN MATLAB - CIRCUITO RLC SERIE \n');
fprintf('=======================================================\n\n');

R = 10;       % Resistencia [Ohm]
L = 0.1;      % Inductancia [H]
C = 0.001;    % Capacitancia [F]

fprintf('Parámetros del circuito:\n');
fprintf('  R = %.1f Ohm\n', R);
fprintf('  L = %.4f H\n', L);
fprintf('  C = %.4f F\n\n', C);

%% =========================================================
%  PARTE 2: FUNCIÓN DE TRANSFERENCIA
%  =========================================================
% Para un circuito RLC serie, la FT desde Vi hasta Vc es:
%
%         1/(LC)
% H(s) = ----------------------
%         s^2 + (R/L)s + 1/(LC)
%
% Derivación:
%   Vi(s) = (R + Ls + 1/(Cs)) * I(s)
%   Vc(s) = I(s) / (Cs)
%   H(s)  = Vc(s)/Vi(s) = 1/(LCs^2 + RCs + 1)

num = [1/(L*C)];                    % Numerador
den = [1, R/L, 1/(L*C)];           % Denominador

H = tf(num, den);

fprintf('Función de Transferencia H(s) = Vc(s)/Vi(s):\n');
fprintf('       1/(LC)              %.1f\n', 1/(L*C));
fprintf('H(s) = ─────────────── = ──────────────────────────────\n');
fprintf('       s² + (R/L)s + 1/(LC)   s² + %.1fs + %.1f\n\n', R/L, 1/(L*C));
display(H);

%% =========================================================
%  PARTE 3: POLOS, CEROS Y GANANCIA
%  =========================================================
[z, p, k] = zpkdata(H, 'v');

fprintf('\n--- Polos, Ceros y Ganancia ---\n');
fprintf('Ceros: ');
if isempty(z)
    fprintf('Ninguno\n');
else
    fprintf('%.4f\n', z);
end

fprintf('Polos:\n');
for i = 1:length(p)
    fprintf('  p%d = %.4f + %.4fj\n', i, real(p(i)), imag(p(i)));
end
fprintf('Ganancia estática (DC): k = %.4f\n\n', k);

%% =========================================================
%  PARTE 4: PARÁMETROS DEL SISTEMA DE SEGUNDO ORDEN
%  =========================================================
wn  = sqrt(1/(L*C));          % Frecuencia natural [rad/s]
zeta = R/(2) * sqrt(C/L);     % Factor de amortiguamiento

fprintf('--- Parámetros del Sistema de 2° Orden ---\n');
fprintf('  Frecuencia natural:      ωn   = %.4f rad/s\n', wn);
fprintf('  Factor de amortiguamiento: ζ  = %.4f\n', zeta);
fprintf('  Frecuencia amortiguada:  ωd   = %.4f rad/s\n', wn*sqrt(abs(1-zeta^2)));

if zeta < 1
    fprintf('  Tipo de respuesta: SUBAMORTIGUADA (0 < ζ < 1)\n\n');
elseif zeta == 1
    fprintf('  Tipo de respuesta: CRITICAMENTE AMORTIGUADA (ζ = 1)\n\n');
else
    fprintf('  Tipo de respuesta: SOBREAMORTIGUADA (ζ > 1)\n\n');
end

%% =========================================================
%  PARTE 5: ESTABILIDAD
%  =========================================================
fprintf('--- Análisis de Estabilidad ---\n');
polos_reales = real(p);
if all(polos_reales < 0)
    fprintf('  El sistema es ESTABLE (todos los polos en semiplano izquierdo)\n\n');
elseif any(polos_reales > 0)
    fprintf('  El sistema es INESTABLE (hay polos en semiplano derecho)\n\n');
else
    fprintf('  El sistema es MARGINALMENTE ESTABLE\n\n');
end

%% =========================================================
%  PARTE 6: REPRESENTACIÓN EN ESPACIO DE ESTADOS
%  =========================================================
% Definiendo estados: x1 = Vc, x2 = dVc/dt = iL/C
%
%   dx1/dt = x2
%   dx2/dt = -(1/LC)*x1 - (R/L)*x2 + (1/LC)*vi
%
%   y = x1  (salida = voltaje en el capacitor)

A = [0,          1;
     -1/(L*C), -R/L];

B = [0;
     1/(L*C)];

C_mat = [1, 0];

D = 0;

sys_ss = ss(A, B, C_mat, D);

fprintf('--- Representación en Espacio de Estados ---\n');
fprintf('x_dot = A*x + B*u\n');
fprintf('    y  = C*x + D*u\n\n');
fprintf('Matriz A:\n');
disp(A);
fprintf('Matriz B:\n');
disp(B);
fprintf('Matriz C:\n');
disp(C_mat);
fprintf('Escalar D:\n');
disp(D);

%% =========================================================
%  PARTE 7: GRÁFICAS
%  =========================================================

% --- Figura 1: Respuesta al Escalón Unitario ---
figure(1);
t_final = 2;
t = linspace(0, t_final, 1000);
[y_step, t_step] = step(H, t);

plot(t_step, y_step, 'b-', 'LineWidth', 2);
hold on;
yline(1, 'r--', 'Escalón de referencia', 'LineWidth', 1.5, ...
      'LabelHorizontalAlignment', 'left');
grid on;
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Voltaje Vc [V]', 'FontSize', 12);
title('Respuesta al Escalón - Circuito RLC Serie', 'FontSize', 14);
legend({'Respuesta Vc(t)', 'Entrada unitaria'}, 'Location', 'best');

% Marcar sobrepico si existe
info = stepinfo(H);
if info.Overshoot > 0
    [y_max, idx_max] = max(y_step);
    plot(t_step(idx_max), y_max, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(t_step(idx_max)+0.02, y_max, ...
        sprintf(' Sobrepico = %.2f%%', info.Overshoot), 'FontSize', 10);
end
hold off;

fprintf('\n--- Información de la Respuesta al Escalón ---\n');
fprintf('  Tiempo de subida  (10%%→90%%): %.4f s\n', info.RiseTime);
fprintf('  Tiempo de asentamiento:       %.4f s\n', info.SettlingTime);
fprintf('  Sobrepico:                    %.2f %%\n', info.Overshoot);
fprintf('  Tiempo de pico:               %.4f s\n', info.PeakTime);
fprintf('  Valor pico:                   %.4f V\n\n', info.Peak);

% --- Figura 2: Diagrama de Bode ---
figure(2);
bode(H);
grid on;
title('Diagrama de Bode - Circuito RLC Serie', 'FontSize', 14);

% --- Figura 3: Mapa de Polos y Ceros ---
figure(3);
pzmap(H);
grid on;
title('Mapa de Polos y Ceros', 'FontSize', 14);

% --- Figura 4: Respuesta en Frecuencia (módulo y fase manual) ---
figure(4);
w = logspace(0, 4, 1000);
[mag, phase] = bode(H, w);
mag   = squeeze(mag);
phase = squeeze(phase);

subplot(2,1,1);
semilogx(w, 20*log10(mag), 'b-', 'LineWidth', 2);
grid on;
xlabel('Frecuencia [rad/s]', 'FontSize', 11);
ylabel('Magnitud [dB]', 'FontSize', 11);
title('Respuesta en Frecuencia - Magnitud', 'FontSize', 12);
xline(wn, 'r--', {'\omega_n'}, 'LabelVerticalAlignment', 'bottom');

subplot(2,1,2);
semilogx(w, phase, 'r-', 'LineWidth', 2);
grid on;
xlabel('Frecuencia [rad/s]', 'FontSize', 11);
ylabel('Fase [°]', 'FontSize', 11);
title('Respuesta en Frecuencia - Fase', 'FontSize', 12);
xline(wn, 'b--', {'\omega_n'}, 'LabelVerticalAlignment', 'bottom');

sgtitle('Diagrama de Bode - Circuito RLC Serie', 'FontSize', 14);

% --- Figura 5: Comparación respuesta natural vs forzada ---
figure(5);
% Respuesta natural (condición inicial Vc(0) = 1V, iL(0) = 0)
x0 = [1; 0];   % [Vc(0); x2(0)]
[y_nat, t_nat] = initial(sys_ss, x0, t);

plot(t_nat, y_nat, 'g-', 'LineWidth', 2);
hold on;
plot(t_step, y_step, 'b--', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Voltaje Vc [V]', 'FontSize', 12);
title('Respuesta Natural vs Forzada (Escalón)', 'FontSize', 14);
legend({'Respuesta natural (Vc_0=1V)', 'Respuesta forzada (escalón)'}, ...
       'Location', 'best');
hold off;

fprintf('=======================================================\n');
fprintf('  ANÁLISIS COMPLETO FINALIZADO\n');
fprintf('  Se generaron 5 figuras con los resultados.\n');
fprintf('=======================================================\n');
