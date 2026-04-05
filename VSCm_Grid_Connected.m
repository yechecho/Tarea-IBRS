%% Parameters for VSC Grid-Connected - Control Vectorial Grid-Following
%
% Tarea de Simulacion II: Control Vectorial de un Convertidor VSC en
% Modo Grid-Following
%
% Sistema: Convertidor VSC trifasico conectado a la red electrica
% mediante filtro tipo L, con control vectorial en marco dq.
%
% Estructura de control:
%   - PLL (Phase Locked Loop) para sincronizacion con la red
%   - Lazo externo de potencia (Outer Loop): P*, Q* -> id*, iq*
%   - Lazo interno de corriente (Inner Loop): PI para id e iq
%   - Control Droop primario P-omega y Q-V
%
% Ejecucion: Este script es llamado automaticamente al abrir el modelo.
% Para crear el modelo completo con todos los subsistemas, ejecute:
%   >> create_VSC_GridFollowing_Complete
%
% Copyright 2020-2024

%% =========================================================
%% 1. PARAMETROS DE SIMULACION
%% =========================================================
Ts   = 1e-5;    % Tiempo de muestreo fundamental          [s]
Tsc  = 5e-4;    % Tiempo de muestreo del control          [s]
T_stop = 0.3;   % Tiempo de simulacion                    [s]

%% =========================================================
%% 2. PARAMETROS DE LA RED ELECTRICA
%% =========================================================
f_grid    = 60;                  % Frecuencia de la red              [Hz]
omega_grid = 2*pi*f_grid;        % Frecuencia angular de la red      [rad/s]
V_grid    = 220;                 % Voltaje de linea RMS de la red    [V]
V_grid_ph = V_grid / sqrt(3);    % Voltaje de fase RMS               [V]
V_grid_pk = V_grid_ph * sqrt(2); % Voltaje de fase pico              [V]

%% =========================================================
%% 3. PARAMETROS DEL CONVERTIDOR VSC
%% =========================================================
V_dc  = 400;    % Voltaje del bus DC                      [V]
fsw   = 2000;   % Frecuencia de conmutacion del inversor  [Hz]
f_pwm = fsw;    % Frecuencia PWM (alias)                  [Hz]

%% =========================================================
%% 4. PARAMETROS DEL FILTRO DE ACOPLAMIENTO (Tipo L)
%% =========================================================
L_filter = 5e-3;   % Inductancia del filtro L              [H]
R_filter = 0.05;   % Resistencia del filtro                [Ohm]

%% =========================================================
%% 5. GANANCIAS DEL PLL (Phase Locked Loop)
%% =========================================================
% El PLL estima el angulo theta y la frecuencia omega de la red.
% Ecuacion: omega = omega0 + Kp_pll*vq + Ki_pll*integral(vq)
%           dtheta/dt = omega
% Nota: vq -> 0 en estado estable indica buena sincronizacion.
Kp_pll = 150;    % Ganancia proporcional del PLL           [rad/s/V]
Ki_pll = 150;    % Ganancia integral del PLL               [rad/s^2/V]

%% =========================================================
%% 6. GANANCIAS DEL REGULADOR DE VOLTAJE (Modelo existente)
%% =========================================================
% PI controllers del lazo de voltaje (modelo original)
Kp_vd = 0.36;   % Proporcional eje d - controlador de voltaje
Ki_vd = 850;    % Integral eje d - controlador de voltaje
Kp_vq = 0.36;   % Proporcional eje q - controlador de voltaje
Ki_vq = 850;    % Integral eje q - controlador de voltaje

%% =========================================================
%% 7. GANANCIAS DEL LAZO INTERNO DE CORRIENTE (Inner Loop)
%% =========================================================
% PI controllers para regular las corrientes id e iq en el marco dq.
% Ecuaciones:
%   vd* = Kp_id*(id* - id) + Ki_id*integral(id* - id) - omega*L*iq + vgd
%   vq* = Kp_iq*(iq* - iq) + Ki_iq*integral(iq* - iq) + omega*L*id + vgq
Kp_id = 10;     % Proporcional eje d - controlador de corriente
Ki_id = 100;    % Integral eje d - controlador de corriente
Kp_iq = 10;     % Proporcional eje q - controlador de corriente
Ki_iq = 100;    % Integral eje q - controlador de corriente

%% =========================================================
%% 8. REFERENCIAS DE POTENCIA (Lazo Externo - Outer Loop)
%% =========================================================
% Mapeo: id* = P* / (1.5 * Vd),  iq* = -Q* / (1.5 * Vd)
P_ref  = 5000;  % Referencia de potencia activa inicial   [W]
Q_ref  = 0;     % Referencia de potencia reactiva inicial [VAR]
omega_ref = omega_grid; % Referencia de frecuencia angular [rad/s]
V_ref     = V_grid_ph;  % Referencia de voltaje de fase    [V]

%% Perturbaciones en referencias de potencia
P_ref_step  = 8000; % Referencia P* despues del escalon   [W]     (t=0.05s)
Q_ref_step  = 2000; % Referencia Q* despues del escalon   [VAR]   (t=0.10s)
t_step_P    = 0.05; % Tiempo del escalon en P*            [s]
t_step_Q    = 0.10; % Tiempo del escalon en Q*            [s]

%% =========================================================
%% 9. CONTROL DROOP PRIMARIO P-omega y Q-V
%% =========================================================
% Permite que el convertidor participe en regulacion de frecuencia y voltaje.
% Ecuaciones:
%   omega = omega0 - kp_droop*(P - P*)
%   V     = V0    - kq_droop*(Q - Q*)
kp_droop = 0.005;   % Ganancia droop P-omega              [rad/s/W]
kq_droop = 0.005;   % Ganancia droop Q-V                  [V/VAR]

%% =========================================================
%% 10. RESUMEN DE PARAMETROS (informacion en consola)
%% =========================================================
fprintf('\n');
fprintf('=========================================================\n');
fprintf(' Parametros VSC Grid-Following cargados correctamente\n');
fprintf('=========================================================\n');
fprintf(' Red electrica:\n');
fprintf('   f_grid    = %g Hz,  V_grid = %g V,  V_dc = %g V\n', f_grid, V_grid, V_dc);
fprintf(' Filtro L:\n');
fprintf('   L_filter  = %g mH,  R_filter = %g Ohm\n', L_filter*1e3, R_filter);
fprintf(' PLL:\n');
fprintf('   Kp_pll    = %g,     Ki_pll   = %g\n', Kp_pll, Ki_pll);
fprintf(' Lazo de corriente:\n');
fprintf('   Kp_id/iq  = %g,     Ki_id/iq = %g\n', Kp_id, Ki_id);
fprintf(' Control Droop:\n');
fprintf('   kp_droop  = %g,     kq_droop = %g\n', kp_droop, kq_droop);
fprintf(' Referencias:\n');
fprintf('   P_ref = %g W,  Q_ref = %g VAR\n', P_ref, Q_ref);
fprintf('=========================================================\n\n');
