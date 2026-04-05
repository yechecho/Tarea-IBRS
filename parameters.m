%% parameters.m
% VSC Grid-Following Control System - Parameter Initialization Script
%
% This script loads all system parameters into the MATLAB workspace
% before running or building the Simulink model.
%
% Usage:
%   >> parameters        (run in MATLAB command window)
%   >> run('parameters') (run from another script)
%
% Author: VSC Control System - Tarea IBRS
% MATLAB Version: 2024b

clear; clc;

fprintf('==============================================\n');
fprintf('  VSC Grid-Following Control - Parameters\n');
fprintf('==============================================\n\n');

%% =========================================================
%  1. GRID PARAMETERS
% =========================================================
f_grid      = 60;                    % Grid frequency [Hz]
omega_grid  = 2 * pi * f_grid;      % Grid angular frequency [rad/s]
V_grid_ll   = 220;                   % Grid voltage line-to-line RMS [VAC]
V_grid_ph   = V_grid_ll / sqrt(3);  % Grid phase voltage RMS [V]
V_grid_peak = V_grid_ph * sqrt(2);  % Grid phase voltage peak [V]

fprintf('[GRID]\n');
fprintf('  Frequency          : %g Hz\n',   f_grid);
fprintf('  Angular frequency  : %.4f rad/s\n', omega_grid);
fprintf('  Voltage (L-L RMS)  : %g VAC\n',  V_grid_ll);
fprintf('  Voltage (phase RMS): %.4f V\n',  V_grid_ph);
fprintf('  Voltage (peak)     : %.4f V\n',  V_grid_peak);
fprintf('\n');

%% =========================================================
%  2. CONVERTER POWER RATING
% =========================================================
P_rated = 10e3;    % Converter rated active power  [W]   (10 kW)
Q_rated = 5e3;     % Converter rated reactive power [VAR] (5 kVAR, nominal)

fprintf('[CONVERTER RATING]\n');
fprintf('  Rated active power  : %g W\n',  P_rated);
fprintf('  Rated reactive power: %g VAR\n', Q_rated);
fprintf('\n');

%% =========================================================
%  3. DC-LINK PARAMETERS
% =========================================================
V_dc  = 400;       % DC-link voltage     [VDC]
C_dc  = 2200e-6;   % DC-link capacitance [F]  (2200 µF)
I_dc_rated = P_rated / V_dc;   % Rated DC current [A]

fprintf('[DC-LINK]\n');
fprintf('  DC voltage   : %g VDC\n', V_dc);
fprintf('  Capacitance  : %g µF\n',  C_dc * 1e6);
fprintf('  Rated DC curr: %.4f A\n', I_dc_rated);
fprintf('\n');

%% =========================================================
%  4. AC FILTER PARAMETERS
% =========================================================
L_filter = 15e-3;  % Filter inductance  [H]   (15 mH)
R_filter = 0.1;    % Filter resistance  [Ohm] (0.1 Ω)
tau_filter = L_filter / R_filter;   % Filter time constant [s]

% Per-unit base values
S_base = P_rated;                   % Base apparent power [VA]
V_base = V_grid_ph;                 % Base voltage (phase) [V]
I_base = S_base / (1.5 * V_base);  % Base current (DQ frame) [A]
Z_base = V_base / I_base;           % Base impedance [Ohm]
L_base = Z_base / omega_grid;      % Base inductance [H]

L_filter_pu = L_filter / L_base;   % Filter inductance [pu]
R_filter_pu = R_filter / Z_base;   % Filter resistance [pu]

fprintf('[AC FILTER]\n');
fprintf('  Inductance  : %g mH  (%.4f pu)\n', L_filter*1e3, L_filter_pu);
fprintf('  Resistance  : %g Ohm (%.4f pu)\n', R_filter, R_filter_pu);
fprintf('  Time constant: %.4f ms\n', tau_filter * 1e3);
fprintf('\n');

%% =========================================================
%  5. SWITCHING / SAMPLING PARAMETERS
% =========================================================
f_pwm = 5000;              % PWM (switching) frequency [Hz]
T_pwm = 1 / f_pwm;        % PWM period                [s]
Ts    = 1e-5;              % Control sampling time     [s]  (10 µs)
T_sim = 0.5;               % Simulation stop time      [s]

fprintf('[SWITCHING / SAMPLING]\n');
fprintf('  PWM frequency    : %g kHz\n', f_pwm/1e3);
fprintf('  PWM period       : %g µs\n',  T_pwm*1e6);
fprintf('  Sampling time Ts : %g µs\n',  Ts*1e6);
fprintf('  Simulation time  : %g s\n',   T_sim);
fprintf('\n');

%% =========================================================
%  6. PLL CONTROLLER PARAMETERS
% =========================================================
% Standard synchronous-reference-frame PLL (SRF-PLL)
% Input: Vq error (force Vq → 0 to lock phase)
Kp_pll = 100;    % PLL proportional gain
Ki_pll = 5000;   % PLL integral gain

% Natural frequency and damping (analytical)
omega_n_pll = sqrt(Ki_pll);              % Natural frequency [rad/s]
zeta_pll    = Kp_pll / (2 * sqrt(Ki_pll));  % Damping ratio

fprintf('[PLL]\n');
fprintf('  Kp = %g,  Ki = %g\n', Kp_pll, Ki_pll);
fprintf('  Natural frequency: %.2f rad/s (%.2f Hz)\n', omega_n_pll, omega_n_pll/(2*pi));
fprintf('  Damping ratio   : %.4f\n', zeta_pll);
fprintf('\n');

%% =========================================================
%  7. CURRENT CONTROLLER PARAMETERS (Inner Loop)
% =========================================================
% Dual PI controllers for Id and Iq channels
Kp_id = 10;    % Proportional gain - Id loop
Ki_id = 100;   % Integral gain     - Id loop
Kp_iq = 10;    % Proportional gain - Iq loop
Ki_iq = 100;   % Integral gain     - Iq loop

% Bandwidth of current loop
omega_cc = Kp_id / L_filter;   % Current controller bandwidth [rad/s]

fprintf('[CURRENT CONTROLLER]\n');
fprintf('  Id loop: Kp = %g, Ki = %g\n', Kp_id, Ki_id);
fprintf('  Iq loop: Kp = %g, Ki = %g\n', Kp_iq, Ki_iq);
fprintf('  Bandwidth: %.2f rad/s (%.2f Hz)\n', omega_cc, omega_cc/(2*pi));
fprintf('\n');

%% =========================================================
%  8. DROOP CONTROL PARAMETERS (Outer Loop)
% =========================================================
% P-ω droop: ω = ω_ref - Kp_droop * (P - P_ref)
% Q-V droop: V = V_ref - Kq_droop * (Q - Q_ref)
Kp_droop = 0.01;   % Active power droop gain   [rad/s/W]
Kq_droop = 0.01;   % Reactive power droop gain [V/VAR]

% Reference power set-points
P_ref = 0;   % Active power reference   [W]
Q_ref = 0;   % Reactive power reference [VAR]

fprintf('[DROOP CONTROL]\n');
fprintf('  Kp_droop = %g rad/s/W\n', Kp_droop);
fprintf('  Kq_droop = %g V/VAR\n',   Kq_droop);
fprintf('  P_ref = %g W,  Q_ref = %g VAR\n', P_ref, Q_ref);
fprintf('\n');

%% =========================================================
%  9. MODULATION INDEX LIMIT
% =========================================================
Ma_max = 1.0;       % Maximum modulation index (linear range SPWM)
Ma_max_svpwm = 2 / sqrt(3);   % SVPWM maximum modulation index (~1.155)

fprintf('[MODULATION]\n');
fprintf('  Max modulation index (SPWM) : %.4f\n', Ma_max);
fprintf('  Max modulation index (SVPWM): %.4f\n', Ma_max_svpwm);
fprintf('\n');

%% =========================================================
%  10. DERIVED / USEFUL QUANTITIES
% =========================================================
% Rated phase current (peak)
I_rated_peak = (2/3) * P_rated / V_grid_peak;

% Id_ref at rated power (unity power factor, Vq=0 in SRF)
Id_ref_rated = I_rated_peak;
Iq_ref_rated = 0;   % Unity power factor

fprintf('[DERIVED QUANTITIES]\n');
fprintf('  Rated phase current (peak): %.4f A\n', I_rated_peak);
fprintf('  Id_ref at rated power     : %.4f A\n', Id_ref_rated);
fprintf('  Iq_ref at unity PF        : %.4f A\n', Iq_ref_rated);
fprintf('\n');

fprintf('==============================================\n');
fprintf('  All parameters loaded successfully!\n');
fprintf('  Next: Run create_VSC_model.m to build model\n');
fprintf('==============================================\n');
