%% create_VSC_model.m
% VSC Grid-Following Control System — Simulink Model Builder
%
% This script creates the complete, fully-functional Simulink model
% for a grid-following Voltage Source Converter (VSC) control system.
%
% REQUIREMENTS:
%   - MATLAB R2021a or newer (tested on R2024b)
%   - Simulink
%   - Control System Toolbox  (for PID Controller block)
%
% USAGE:
%   1. Open MATLAB and navigate to the project folder.
%   2. Run this script:  >> create_VSC_model
%   3. The model 'VSC_GridFollowing_Control.slx' is created and opened.
%   4. Press Ctrl+T (or Run button) to simulate for 0.5 seconds.
%   5. Open the 'Visualization' subsystem to view the six scope windows.
%
% CONTROL ARCHITECTURE (grid-following, current-mode):
%
%   GridSource → RL Filter → (current meas.)
%       ↓ Va,Vb,Vc               ↓ Ia,Ib,Ic
%   abc→dq (Park)V       abc→dq (Park)I
%       ↓ Vd,Vq                  ↓ Id,Iq
%       → PLL(θ,ω) ──────────→  DroopControl → CurrentControl
%                                                    ↓ Vd_ref,Vq_ref
%                                             dq→abc (InvPark)
%                                                    ↓ Va_conv,Vb_conv,Vc_conv
%                                                 Filter (feedback)
%
% Author: Tarea IBRS — VSC Grid-Following Control
% MATLAB Version: R2024b

clear; close all; clc;

fprintf('=============================================\n');
fprintf('  VSC Grid-Following Control — Model Builder \n');
fprintf('=============================================\n\n');

%% =========================================================
%%  STEP 1: Load parameters
%% =========================================================
run('parameters.m');
fprintf('\n');

%% =========================================================
%%  STEP 2: Create Simulink model
%% =========================================================
mdl = 'VSC_GridFollowing_Control';

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'Solver',     'ode23tb', ...
    'SolverType', 'Variable-step', ...
    'MaxStep',    num2str(Ts), ...
    'StopTime',   num2str(T_sim), ...
    'RelTol',     '1e-4', ...
    'AbsTol',     '1e-6');

fprintf('[MODEL] "%s" created (solver=ode23tb, T=%gs, Ts=%gµs)\n\n', ...
    mdl, T_sim, Ts*1e6);

%% =========================================================
%%  Phase constants used throughout
%% =========================================================
PH  = {'A','B','C'};
PHI = [0, -2*pi/3, 2*pi/3];   % positive-sequence phase angles [rad]

%% =========================================================
%%  SS-1: GRIDSOURCE — Three-phase sinusoidal AC sources
%% =========================================================
gs = [mdl '/GridSource'];
add_block('simulink/Ports & Subsystems/Subsystem', gs);
set_param(gs, 'Position', blkp(1,3,120,110));
Simulink.SubSystem.deleteContents(gs);

for k = 1:3
    sw = [gs '/V' PH{k}];
    add_block('simulink/Sources/Sine Wave', sw);
    set_param(sw, ...
        'Amplitude',  'V_grid_peak', ...
        'Frequency',  'omega_grid', ...
        'Phase',      num2str(PHI(k)), ...
        'SampleTime', '0', ...
        'Position',   blkp(1, k, 130, 30));

    op = [gs '/V' PH{k} '_out'];
    add_block('simulink/Ports & Subsystems/Out1', op);
    set_param(op, 'Port', num2str(k), 'Position', blkp(3, k, 30, 30));
    add_line(gs, ['V' PH{k} '/1'], ['V' PH{k} '_out/1'], 'autorouting','on');
end
fprintf('[1/9] GridSource  (Va,Vb,Vc  %.1fV pk  %gHz)\n', V_grid_peak, f_grid);

%% =========================================================
%%  SS-2: FILTER — Per-phase RL filter  (transfer function)
%%        Model:  I(s) = [Vgrid(s) - Vconv(s)] / (L*s + R)
%% =========================================================
fl = [mdl '/Filter'];
add_block('simulink/Ports & Subsystems/Subsystem', fl);
set_param(fl, 'Position', blkp(2,3,120,180));
Simulink.SubSystem.deleteContents(fl);

for k = 1:3
    ph = PH{k};

    ipg = [fl '/Vg' ph];   % grid voltage input   (port k)
    add_block('simulink/Ports & Subsystems/In1', ipg);
    set_param(ipg, 'Port', num2str(k), 'Position', blkp(1, k, 30, 28));

    ipc = [fl '/Vc' ph];   % converter voltage input  (port k+3)
    add_block('simulink/Ports & Subsystems/In1', ipc);
    set_param(ipc, 'Port', num2str(k+3), 'Position', blkp(1, k+3, 30, 28));

    sm = [fl '/Diff' ph];
    add_block('simulink/Math Operations/Sum', sm);
    set_param(sm, 'Inputs', '+-', 'Position', blkp(2, k, 30, 30));

    tf = [fl '/TF_' ph];
    add_block('simulink/Continuous/Transfer Fcn', tf);
    set_param(tf, ...
        'Numerator',   '[1]', ...
        'Denominator', '[L_filter, R_filter]', ...
        'Position',    blkp(3, k, 160, 30));

    op = [fl '/I' ph];
    add_block('simulink/Ports & Subsystems/Out1', op);
    set_param(op, 'Port', num2str(k), 'Position', blkp(5, k, 30, 28));

    add_line(fl, ['Vg' ph '/1'],   ['Diff' ph '/1'], 'autorouting','on');
    add_line(fl, ['Vc' ph '/1'],   ['Diff' ph '/2'], 'autorouting','on');
    add_line(fl, ['Diff' ph '/1'], ['TF_'  ph '/1'], 'autorouting','on');
    add_line(fl, ['TF_'  ph '/1'], ['I'    ph '/1'], 'autorouting','on');
end
fprintf('[2/9] Filter  (L=%.1fmH  R=%.2fΩ  TF 1/(Ls+R))\n', L_filter*1e3, R_filter);

%% =========================================================
%%  SS-3: abc2dq_V  and  abc2dq_I — Park transform
%%        (amplitude-invariant Clarke + Park)
%%
%%  α = (2/3)*( xa - xb/2 - xc/2 )
%%  β = (2/3)*(       (√3/2)*xb - (√3/2)*xc )
%%  xd =  α*cos(θ) + β*sin(θ)
%%  xq = -α*sin(θ) + β*cos(θ)
%% =========================================================
buildParkSS([mdl '/abc2dq_V'], blkp(3,1,120,200));
buildParkSS([mdl '/abc2dq_I'], blkp(3,5,120,200));
fprintf('[3/9] Park transforms (abc→dq)  voltages + currents\n');

%% =========================================================
%%  SS-4: PLL — Synchronous Reference Frame PLL
%%
%%  Forces Vq → 0.  Structure:
%%    error = -Vq
%%    Δω   = PI(error)
%%    ω    = ω_ff + Δω
%%    θ    = ∫ω dt   (wrapped to [0, 2π])
%% =========================================================
pll = [mdl '/PLL'];
add_block('simulink/Ports & Subsystems/Subsystem', pll);
set_param(pll, 'Position', blkp(4,1,130,100));
Simulink.SubSystem.deleteContents(pll);

add_block('simulink/Ports & Subsystems/In1', [pll '/Vq_in']);
set_param([pll '/Vq_in'], 'Port','1', 'Position', blkp(1,1,30,28));

add_block('simulink/Math Operations/Gain', [pll '/Neg']);
set_param([pll '/Neg'], 'Gain','-1', 'Position', blkp(2,1,50,28));

add_block('simulink/Continuous/PID Controller', [pll '/PI_PLL']);
set_param([pll '/PI_PLL'], ...
    'Controller','PI', 'P','Kp_pll', 'I','Ki_pll', ...
    'InitialConditionSource','internal', 'InitialCondition','0', ...
    'Position', blkp(3,1,130,50));

add_block('simulink/Sources/Constant', [pll '/Omega_ff']);
set_param([pll '/Omega_ff'], 'Value','omega_grid', 'Position', blkp(3,2,100,28));

add_block('simulink/Math Operations/Sum', [pll '/Sum_omega']);
set_param([pll '/Sum_omega'], 'Inputs','++', 'Position', blkp(4,1,30,30));

add_block('simulink/Continuous/Integrator', [pll '/Int_theta']);
set_param([pll '/Int_theta'], 'InitialCondition','0', 'Position', blkp(5,1,80,30));

% Wrap theta to [0, 2π]:  theta_wrapped = theta - 2π * floor(theta / 2π)
add_block('simulink/Math Operations/Gain',             [pll '/Inv2pi']);
set_param([pll '/Inv2pi'],    'Gain','1/(2*pi)',  'Position', blkp(6,2,60,28));
add_block('simulink/Math Operations/Rounding Function',[pll '/Floor_th']);
set_param([pll '/Floor_th'],  'Operator','floor', 'Position', blkp(7,2,60,28));
add_block('simulink/Math Operations/Gain',             [pll '/Scale2pi']);
set_param([pll '/Scale2pi'],  'Gain','2*pi',      'Position', blkp(8,2,60,28));
add_block('simulink/Math Operations/Sum',              [pll '/Wrap']);
set_param([pll '/Wrap'],      'Inputs','+-',      'Position', blkp(9,1,30,30));

add_block('simulink/Ports & Subsystems/Out1', [pll '/Theta_out']);
set_param([pll '/Theta_out'], 'Port','1', 'Position', blkp(10,1,30,28));
add_block('simulink/Ports & Subsystems/Out1', [pll '/Omega_out']);
set_param([pll '/Omega_out'], 'Port','2', 'Position', blkp(5,3,30,28));

add_line(pll,'Vq_in/1',   'Neg/1',        'autorouting','on');
add_line(pll,'Neg/1',     'PI_PLL/1',     'autorouting','on');
add_line(pll,'PI_PLL/1',  'Sum_omega/1',  'autorouting','on');
add_line(pll,'Omega_ff/1','Sum_omega/2',  'autorouting','on');
add_line(pll,'Sum_omega/1','Int_theta/1', 'autorouting','on');
add_line(pll,'Sum_omega/1','Omega_out/1', 'autorouting','on');
add_line(pll,'Int_theta/1','Inv2pi/1',    'autorouting','on');
add_line(pll,'Int_theta/1','Wrap/1',      'autorouting','on');
add_line(pll,'Inv2pi/1',  'Floor_th/1',  'autorouting','on');
add_line(pll,'Floor_th/1','Scale2pi/1',  'autorouting','on');
add_line(pll,'Scale2pi/1','Wrap/2',      'autorouting','on');
add_line(pll,'Wrap/1',    'Theta_out/1', 'autorouting','on');

fprintf('[4/9] PLL  (Kp=%g  Ki=%g  ζ=%.3f  ωn=%.1frad/s)\n', ...
    Kp_pll, Ki_pll, Kp_pll/(2*sqrt(Ki_pll)), sqrt(Ki_pll));

%% =========================================================
%%  SS-5: DROOPCONTROL — Outer active/reactive power loop
%%
%%  P = 1.5*(Vd*Id + Vq*Iq)
%%  Q = 1.5*(Vd*Iq - Vq*Id)
%%  Id_ref = Kp_droop*(P_ref - P_lpf)
%%  Iq_ref = Kq_droop*(Q_ref - Q_lpf)
%% =========================================================
drp = [mdl '/DroopControl'];
add_block('simulink/Ports & Subsystems/Subsystem', drp);
set_param(drp, 'Position', blkp(5,1,130,130));
Simulink.SubSystem.deleteContents(drp);

% 4 inputs: Vd(1) Vq(2) Id(3) Iq(4)
drp_in = {'Vd','Vq','Id','Iq'};
for k = 1:4
    add_block('simulink/Ports & Subsystems/In1', [drp '/' drp_in{k}]);
    set_param([drp '/' drp_in{k}], 'Port', num2str(k), 'Position', blkp(1,k,30,28));
end

% Products for power
add_block('simulink/Math Operations/Product', [drp '/VdId']); set_param([drp '/VdId'], 'Position', blkp(2,1,30,28));
add_block('simulink/Math Operations/Product', [drp '/VqIq']); set_param([drp '/VqIq'], 'Position', blkp(2,2,30,28));
add_block('simulink/Math Operations/Product', [drp '/VdIq']); set_param([drp '/VdIq'], 'Position', blkp(2,4,30,28));
add_block('simulink/Math Operations/Product', [drp '/VqId']); set_param([drp '/VqId'], 'Position', blkp(2,5,30,28));

add_block('simulink/Math Operations/Sum',  [drp '/SumP']); set_param([drp '/SumP'], 'Inputs','++', 'Position', blkp(3,1,30,30));
add_block('simulink/Math Operations/Gain', [drp '/GainP']); set_param([drp '/GainP'], 'Gain','1.5', 'Position', blkp(4,1,60,28));
add_block('simulink/Math Operations/Sum',  [drp '/SumQ']); set_param([drp '/SumQ'], 'Inputs','+-', 'Position', blkp(3,4,30,30));
add_block('simulink/Math Operations/Gain', [drp '/GainQ']); set_param([drp '/GainQ'], 'Gain','1.5', 'Position', blkp(4,4,60,28));

% 5 Hz low-pass filter  τ = 1/(2π·5)
tau_lpf = 1 / (2*pi*5);
add_block('simulink/Continuous/Transfer Fcn', [drp '/LPF_P']);
set_param([drp '/LPF_P'], 'Numerator','[1]', 'Denominator', sprintf('[%g, 1]',tau_lpf), 'Position', blkp(5,1,110,30));
add_block('simulink/Continuous/Transfer Fcn', [drp '/LPF_Q']);
set_param([drp '/LPF_Q'], 'Numerator','[1]', 'Denominator', sprintf('[%g, 1]',tau_lpf), 'Position', blkp(5,4,110,30));

add_block('simulink/Sources/Constant', [drp '/P_ref']); set_param([drp '/P_ref'], 'Value','P_ref', 'Position', blkp(5,2,80,28));
add_block('simulink/Sources/Constant', [drp '/Q_ref']); set_param([drp '/Q_ref'], 'Value','Q_ref', 'Position', blkp(5,5,80,28));

add_block('simulink/Math Operations/Sum', [drp '/ErrP']); set_param([drp '/ErrP'], 'Inputs','+-', 'Position', blkp(6,1,30,30));
add_block('simulink/Math Operations/Sum', [drp '/ErrQ']); set_param([drp '/ErrQ'], 'Inputs','+-', 'Position', blkp(6,4,30,30));

add_block('simulink/Math Operations/Gain', [drp '/KpD']); set_param([drp '/KpD'], 'Gain','Kp_droop', 'Position', blkp(7,1,80,28));
add_block('simulink/Math Operations/Gain', [drp '/KqD']); set_param([drp '/KqD'], 'Gain','Kq_droop', 'Position', blkp(7,4,80,28));

% Maximum current reference = 1.5× the rated peak phase current.
% At rated power P_rated with unity PF:  Id_peak = (2/3)*P_rated/V_grid_peak
%   (factor 2/3 from amplitude-invariant Park transform).
% The 1.5 safety margin allows transient overcurrent before clipping.
I_sat_val = 1.5 * (2/3) * P_rated / V_grid_peak;
add_block('simulink/Discontinuities/Saturation', [drp '/SatId']); set_param([drp '/SatId'], 'UpperLimit',num2str(I_sat_val), 'LowerLimit',num2str(-I_sat_val), 'Position',blkp(8,1,80,28));
add_block('simulink/Discontinuities/Saturation', [drp '/SatIq']); set_param([drp '/SatIq'], 'UpperLimit',num2str(I_sat_val), 'LowerLimit',num2str(-I_sat_val), 'Position',blkp(8,4,80,28));

% 4 outputs: Id_ref(1) Iq_ref(2) P(3) Q(4)
add_block('simulink/Ports & Subsystems/Out1', [drp '/Id_ref']); set_param([drp '/Id_ref'], 'Port','1', 'Position', blkp(9,1,30,28));
add_block('simulink/Ports & Subsystems/Out1', [drp '/Iq_ref']); set_param([drp '/Iq_ref'], 'Port','2', 'Position', blkp(9,4,30,28));
add_block('simulink/Ports & Subsystems/Out1', [drp '/P_out']);  set_param([drp '/P_out'],  'Port','3', 'Position', blkp(6,2,30,28));
add_block('simulink/Ports & Subsystems/Out1', [drp '/Q_out']);  set_param([drp '/Q_out'],  'Port','4', 'Position', blkp(6,5,30,28));

% Wiring
add_line(drp,'Vd/1','VdId/1','autorouting','on'); add_line(drp,'Id/1','VdId/2','autorouting','on');
add_line(drp,'Vq/1','VqIq/1','autorouting','on'); add_line(drp,'Iq/1','VqIq/2','autorouting','on');
add_line(drp,'Vd/1','VdIq/1','autorouting','on'); add_line(drp,'Iq/1','VdIq/2','autorouting','on');
add_line(drp,'Vq/1','VqId/1','autorouting','on'); add_line(drp,'Id/1','VqId/2','autorouting','on');
add_line(drp,'VdId/1','SumP/1','autorouting','on'); add_line(drp,'VqIq/1','SumP/2','autorouting','on');
add_line(drp,'SumP/1','GainP/1','autorouting','on'); add_line(drp,'GainP/1','LPF_P/1','autorouting','on');
add_line(drp,'VdIq/1','SumQ/1','autorouting','on'); add_line(drp,'VqId/1','SumQ/2','autorouting','on');
add_line(drp,'SumQ/1','GainQ/1','autorouting','on'); add_line(drp,'GainQ/1','LPF_Q/1','autorouting','on');
add_line(drp,'LPF_P/1','P_out/1','autorouting','on');
add_line(drp,'LPF_Q/1','Q_out/1','autorouting','on');
add_line(drp,'P_ref/1','ErrP/1','autorouting','on'); add_line(drp,'LPF_P/1','ErrP/2','autorouting','on');
add_line(drp,'Q_ref/1','ErrQ/1','autorouting','on'); add_line(drp,'LPF_Q/1','ErrQ/2','autorouting','on');
add_line(drp,'ErrP/1','KpD/1','autorouting','on'); add_line(drp,'KpD/1','SatId/1','autorouting','on'); add_line(drp,'SatId/1','Id_ref/1','autorouting','on');
add_line(drp,'ErrQ/1','KqD/1','autorouting','on'); add_line(drp,'KqD/1','SatIq/1','autorouting','on'); add_line(drp,'SatIq/1','Iq_ref/1','autorouting','on');

fprintf('[5/9] DroopControl  (Kp_droop=%g  Kq_droop=%g)\n', Kp_droop, Kq_droop);

%% =========================================================
%%  SS-6: CURRENTCONTROL — Inner dq PI loop with decoupling
%%
%%  Vd_ref = PI_Id(Id_ref - Id) - ω·L·Iq + Vd_ff
%%  Vq_ref = PI_Iq(Iq_ref - Iq) + ω·L·Id + Vq_ff
%% =========================================================
cc = [mdl '/CurrentControl'];
add_block('simulink/Ports & Subsystems/Subsystem', cc);
set_param(cc, 'Position', blkp(6,3,130,160));
Simulink.SubSystem.deleteContents(cc);

% 7 inputs
cc_in = {'Id_meas','Iq_meas','Id_ref','Iq_ref','Vd_ff','Vq_ff','Omega'};
for k = 1:7
    add_block('simulink/Ports & Subsystems/In1', [cc '/' cc_in{k}]);
    set_param([cc '/' cc_in{k}], 'Port', num2str(k), 'Position', blkp(1,k,40,28));
end

% Id error and PI
add_block('simulink/Math Operations/Sum', [cc '/ErrId']); set_param([cc '/ErrId'], 'Inputs','+-', 'Position', blkp(2,1,30,30));
add_line(cc,'Id_ref/1','ErrId/1','autorouting','on'); add_line(cc,'Id_meas/1','ErrId/2','autorouting','on');
add_block('simulink/Continuous/PID Controller', [cc '/PI_Id']);
set_param([cc '/PI_Id'], 'Controller','PI', 'P','Kp_id', 'I','Ki_id', 'Position', blkp(3,1,130,40));
add_line(cc,'ErrId/1','PI_Id/1','autorouting','on');

% Iq error and PI
add_block('simulink/Math Operations/Sum', [cc '/ErrIq']); set_param([cc '/ErrIq'], 'Inputs','+-', 'Position', blkp(2,3,30,30));
add_line(cc,'Iq_ref/1','ErrIq/1','autorouting','on'); add_line(cc,'Iq_meas/1','ErrIq/2','autorouting','on');
add_block('simulink/Continuous/PID Controller', [cc '/PI_Iq']);
set_param([cc '/PI_Iq'], 'Controller','PI', 'P','Kp_iq', 'I','Ki_iq', 'Position', blkp(3,3,130,40));
add_line(cc,'ErrIq/1','PI_Iq/1','autorouting','on');

% Cross-coupling decoupling: omega*L*Iq (subtracted from Vd_ref)
add_block('simulink/Math Operations/Gain',   [cc '/GL_d']); set_param([cc '/GL_d'], 'Gain','L_filter', 'Position', blkp(2,5,60,28));
add_block('simulink/Math Operations/Product',[cc '/oLIq']); set_param([cc '/oLIq'], 'Position', blkp(3,5,30,28));
add_line(cc,'Iq_meas/1','GL_d/1','autorouting','on');
add_line(cc,'GL_d/1','oLIq/1','autorouting','on');
add_line(cc,'Omega/1','oLIq/2','autorouting','on');

% Cross-coupling decoupling: omega*L*Id (added to Vq_ref)
add_block('simulink/Math Operations/Gain',   [cc '/GL_q']); set_param([cc '/GL_q'], 'Gain','L_filter', 'Position', blkp(2,6,60,28));
add_block('simulink/Math Operations/Product',[cc '/oLId']); set_param([cc '/oLId'], 'Position', blkp(3,6,30,28));
add_line(cc,'Id_meas/1','GL_q/1','autorouting','on');
add_line(cc,'GL_q/1','oLId/1','autorouting','on');
add_line(cc,'Omega/1','oLId/2','autorouting','on');

% Sum for Vd_ref: PI_Id + Vd_ff - omega*L*Iq
add_block('simulink/Math Operations/Sum', [cc '/SumVd']); set_param([cc '/SumVd'], 'Inputs','++-', 'Position', blkp(4,1,30,40));
add_line(cc,'PI_Id/1','SumVd/1','autorouting','on');
add_line(cc,'Vd_ff/1','SumVd/2','autorouting','on');
add_line(cc,'oLIq/1', 'SumVd/3','autorouting','on');

% Sum for Vq_ref: PI_Iq + Vq_ff + omega*L*Id
add_block('simulink/Math Operations/Sum', [cc '/SumVq']); set_param([cc '/SumVq'], 'Inputs','+++', 'Position', blkp(4,3,30,40));
add_line(cc,'PI_Iq/1','SumVq/1','autorouting','on');
add_line(cc,'Vq_ff/1','SumVq/2','autorouting','on');
add_line(cc,'oLId/1', 'SumVq/3','autorouting','on');

% Output voltage saturation ±Vdc/2
V_sat = V_dc / 2;
add_block('simulink/Discontinuities/Saturation', [cc '/SatVd']); set_param([cc '/SatVd'], 'UpperLimit',num2str(V_sat), 'LowerLimit',num2str(-V_sat), 'Position',blkp(5,1,80,30));
add_block('simulink/Discontinuities/Saturation', [cc '/SatVq']); set_param([cc '/SatVq'], 'UpperLimit',num2str(V_sat), 'LowerLimit',num2str(-V_sat), 'Position',blkp(5,3,80,30));
add_line(cc,'SumVd/1','SatVd/1','autorouting','on');
add_line(cc,'SumVq/1','SatVq/1','autorouting','on');

% 2 outputs: Vd_ref, Vq_ref
add_block('simulink/Ports & Subsystems/Out1', [cc '/Vd_ref']); set_param([cc '/Vd_ref'], 'Port','1', 'Position', blkp(6,1,30,28));
add_block('simulink/Ports & Subsystems/Out1', [cc '/Vq_ref']); set_param([cc '/Vq_ref'], 'Port','2', 'Position', blkp(6,3,30,28));
add_line(cc,'SatVd/1','Vd_ref/1','autorouting','on');
add_line(cc,'SatVq/1','Vq_ref/1','autorouting','on');

fprintf('[6/9] CurrentControl  (Kp=%g  Ki=%g  BW=%.0frad/s  decoupled)\n', ...
    Kp_id, Ki_id, Kp_id/L_filter);

%% =========================================================
%%  SS-7: dq2abc — Inverse Park transform
%%
%%  α = xd*cos(θ) - xq*sin(θ)
%%  β = xd*sin(θ) + xq*cos(θ)
%%  xA =  α
%%  xB = -0.5*α + (√3/2)*β
%%  xC = -0.5*α - (√3/2)*β
%% =========================================================
ipr = [mdl '/dq2abc'];
add_block('simulink/Ports & Subsystems/Subsystem', ipr);
set_param(ipr, 'Position', blkp(7,3,120,180));
Simulink.SubSystem.deleteContents(ipr);

add_block('simulink/Ports & Subsystems/In1', [ipr '/xd']);    set_param([ipr '/xd'],    'Port','1', 'Position', blkp(1,1,30,28));
add_block('simulink/Ports & Subsystems/In1', [ipr '/xq']);    set_param([ipr '/xq'],    'Port','2', 'Position', blkp(1,2,30,28));
add_block('simulink/Ports & Subsystems/In1', [ipr '/Theta']); set_param([ipr '/Theta'], 'Port','3', 'Position', blkp(1,3,30,28));

add_block('simulink/Math Operations/Trigonometric Function', [ipr '/CosT']); set_param([ipr '/CosT'], 'Operator','cos', 'Position', blkp(2,3,60,28));
add_block('simulink/Math Operations/Trigonometric Function', [ipr '/SinT']); set_param([ipr '/SinT'], 'Operator','sin', 'Position', blkp(2,4,60,28));
add_line(ipr,'Theta/1','CosT/1','autorouting','on');
add_line(ipr,'Theta/1','SinT/1','autorouting','on');

% alpha = xd*cos - xq*sin
add_block('simulink/Math Operations/Product', [ipr '/Pdcos']); set_param([ipr '/Pdcos'], 'Position', blkp(3,1,30,28));
add_block('simulink/Math Operations/Product', [ipr '/Pqsin']); set_param([ipr '/Pqsin'], 'Position', blkp(3,2,30,28));
add_block('simulink/Math Operations/Sum',     [ipr '/SumAlp']); set_param([ipr '/SumAlp'], 'Inputs','+-', 'Position', blkp(4,1,30,30));
add_line(ipr,'xd/1',   'Pdcos/1','autorouting','on'); add_line(ipr,'CosT/1','Pdcos/2','autorouting','on');
add_line(ipr,'xq/1',   'Pqsin/1','autorouting','on'); add_line(ipr,'SinT/1','Pqsin/2','autorouting','on');
add_line(ipr,'Pdcos/1','SumAlp/1','autorouting','on'); add_line(ipr,'Pqsin/1','SumAlp/2','autorouting','on');

% beta = xd*sin + xq*cos
add_block('simulink/Math Operations/Product', [ipr '/Pdsin']); set_param([ipr '/Pdsin'], 'Position', blkp(3,4,30,28));
add_block('simulink/Math Operations/Product', [ipr '/Pqcos']); set_param([ipr '/Pqcos'], 'Position', blkp(3,5,30,28));
add_block('simulink/Math Operations/Sum',     [ipr '/SumBet']); set_param([ipr '/SumBet'], 'Inputs','++', 'Position', blkp(4,4,30,30));
add_line(ipr,'xd/1',   'Pdsin/1','autorouting','on'); add_line(ipr,'SinT/1','Pdsin/2','autorouting','on');
add_line(ipr,'xq/1',   'Pqcos/1','autorouting','on'); add_line(ipr,'CosT/1','Pqcos/2','autorouting','on');
add_line(ipr,'Pdsin/1','SumBet/1','autorouting','on'); add_line(ipr,'Pqcos/1','SumBet/2','autorouting','on');

% xA = alpha
add_block('simulink/Ports & Subsystems/Out1', [ipr '/xA']); set_param([ipr '/xA'], 'Port','1', 'Position', blkp(5,1,30,28));
add_line(ipr,'SumAlp/1','xA/1','autorouting','on');

% xB = -0.5*alpha + (sqrt(3)/2)*beta
add_block('simulink/Math Operations/Gain', [ipr '/KbA']); set_param([ipr '/KbA'], 'Gain','-0.5',      'Position', blkp(5,3,50,28));
add_block('simulink/Math Operations/Gain', [ipr '/KbB']); set_param([ipr '/KbB'], 'Gain','sqrt(3)/2', 'Position', blkp(5,4,70,28));
add_block('simulink/Math Operations/Sum',  [ipr '/SumVb']); set_param([ipr '/SumVb'], 'Inputs','++',  'Position', blkp(6,3,30,30));
add_block('simulink/Ports & Subsystems/Out1', [ipr '/xB']); set_param([ipr '/xB'], 'Port','2', 'Position', blkp(7,3,30,28));
add_line(ipr,'SumAlp/1','KbA/1','autorouting','on');
add_line(ipr,'SumBet/1','KbB/1','autorouting','on');
add_line(ipr,'KbA/1',   'SumVb/1','autorouting','on');
add_line(ipr,'KbB/1',   'SumVb/2','autorouting','on');
add_line(ipr,'SumVb/1', 'xB/1',   'autorouting','on');

% xC = -0.5*alpha - (sqrt(3)/2)*beta
add_block('simulink/Math Operations/Gain', [ipr '/KcA']); set_param([ipr '/KcA'], 'Gain','-0.5',       'Position', blkp(5,6,50,28));
add_block('simulink/Math Operations/Gain', [ipr '/KcB']); set_param([ipr '/KcB'], 'Gain','-sqrt(3)/2', 'Position', blkp(5,7,70,28));
add_block('simulink/Math Operations/Sum',  [ipr '/SumVc']); set_param([ipr '/SumVc'], 'Inputs','++',   'Position', blkp(6,6,30,30));
add_block('simulink/Ports & Subsystems/Out1', [ipr '/xC']); set_param([ipr '/xC'], 'Port','3', 'Position', blkp(7,6,30,28));
add_line(ipr,'SumAlp/1','KcA/1','autorouting','on');
add_line(ipr,'SumBet/1','KcB/1','autorouting','on');
add_line(ipr,'KcA/1',   'SumVc/1','autorouting','on');
add_line(ipr,'KcB/1',   'SumVc/2','autorouting','on');
add_line(ipr,'SumVc/1', 'xC/1',   'autorouting','on');

fprintf('[7/9] dq2abc  (Inverse Park)\n');

%% =========================================================
%%  SS-8: SVPWM — Sinusoidal carrier-based PWM at f_pwm
%%
%%  m_x = V_x_ref / (Vdc/2)       normalization
%%  compare m_x with triangular carrier → gate signals
%%  Upper switch = (m_x > carrier),  Lower = NOT(upper)
%% =========================================================
pwm = [mdl '/SVPWM'];
add_block('simulink/Ports & Subsystems/Subsystem', pwm);
set_param(pwm, 'Position', blkp(8,3,120,180));
Simulink.SubSystem.deleteContents(pwm);

for k = 1:3
    add_block('simulink/Ports & Subsystems/In1', [pwm '/V' PH{k} '_ref']);
    set_param([pwm '/V' PH{k} '_ref'], 'Port', num2str(k), 'Position', blkp(1,k,40,28));
end

add_block('simulink/Sources/Constant', [pwm '/Vdc2']);
set_param([pwm '/Vdc2'], 'Value','V_dc/2', 'Position', blkp(1,5,80,28));

add_block('simulink/Sources/Repeating Sequence', [pwm '/Carrier']);
set_param([pwm '/Carrier'], ...
    'rep_seq_t', '[0, 0.5/f_pwm, 1/f_pwm]', ...
    'rep_seq_y', '[-1, 1, -1]', ...
    'Position',  blkp(1,6,140,30));

port_n = 1;
for k = 1:3
    ph = PH{k};

    div = [pwm '/Div_' ph];
    add_block('simulink/Math Operations/Divide', div);
    set_param(div, 'Position', blkp(2,k,30,28));
    add_line(pwm, ['V' ph '_ref/1'], ['Div_' ph '/1'], 'autorouting','on');
    add_line(pwm, 'Vdc2/1',         ['Div_' ph '/2'], 'autorouting','on');

    sat = [pwm '/Sat_' ph];
    add_block('simulink/Discontinuities/Saturation', sat);
    set_param(sat, 'UpperLimit','1','LowerLimit','-1','Position', blkp(3,k,70,28));
    add_line(pwm, ['Div_' ph '/1'], ['Sat_' ph '/1'], 'autorouting','on');

    dif = [pwm '/Dif_' ph];
    add_block('simulink/Math Operations/Sum', dif);
    set_param(dif, 'Inputs','+-', 'Position', blkp(4,k,30,28));
    add_line(pwm, ['Sat_' ph '/1'], ['Dif_' ph '/1'], 'autorouting','on');
    add_line(pwm, 'Carrier/1',     ['Dif_' ph '/2'], 'autorouting','on');

    add_block('simulink/Logic and Bit Operations/Compare To Constant', [pwm '/Cmp_' ph]);
    set_param([pwm '/Cmp_' ph], 'relop','>','const','0','Position', blkp(5,k,100,28));
    add_line(pwm, ['Dif_' ph '/1'], ['Cmp_' ph '/1'], 'autorouting','on');

    ou = [pwm '/G' ph 'U'];
    add_block('simulink/Ports & Subsystems/Out1', ou);
    set_param(ou, 'Port', num2str(port_n), 'Position', blkp(6,k*2-1,30,28));
    add_line(pwm, ['Cmp_' ph '/1'], ['G' ph 'U/1'], 'autorouting','on');
    port_n = port_n + 1;

    nt = [pwm '/NOT_' ph];
    add_block('simulink/Logic and Bit Operations/Logical Operator', nt);
    set_param(nt, 'Operator','NOT', 'Position', blkp(6,k*2,50,28));
    add_line(pwm, ['Cmp_' ph '/1'], ['NOT_' ph '/1'], 'autorouting','on');

    ol = [pwm '/G' ph 'L'];
    add_block('simulink/Ports & Subsystems/Out1', ol);
    set_param(ol, 'Port', num2str(port_n), 'Position', blkp(7,k*2,30,28));
    add_line(pwm, ['NOT_' ph '/1'], ['G' ph 'L/1'], 'autorouting','on');
    port_n = port_n + 1;
end
fprintf('[8/9] SVPWM  (%g kHz  6 gate signals)\n', f_pwm/1e3);

%% =========================================================
%%  SS-9: VISUALIZATION — Six oscilloscope blocks
%% =========================================================
viz = [mdl '/Visualization'];
add_block('simulink/Ports & Subsystems/Subsystem', viz);
set_param(viz, 'Position', blkp(9,3,130,280));
Simulink.SubSystem.deleteContents(viz);

viz_lbl = {'Va','Vb','Vc','Ia','Ib','Ic','Vd','Vq', ...
           'Id','Iq','Id_ref','Iq_ref','Theta','Omega','P','Q'};
for k = 1:length(viz_lbl)
    ip = [viz '/' viz_lbl{k}];
    add_block('simulink/Ports & Subsystems/In1', ip);
    set_param(ip, 'Port', num2str(k), 'Position', blkp(1,k,40,28));
end

scope_defs = { ...
    'Sc_Vgrid',  [1 2 3],       'Grid Voltages Va,Vb,Vc [V]'; ...
    'Sc_Iabc',   [4 5 6],       'Filter Currents Ia,Ib,Ic [A]'; ...
    'Sc_VdVq',   [7 8],         'DQ Voltages Vd,Vq [V]'; ...
    'Sc_IdIq',   [9 10 11 12],  'DQ Currents: Measured vs Reference [A]'; ...
    'Sc_PLL',    [13 14],       'PLL: Theta [rad] and Omega [rad/s]'; ...
    'Sc_Power',  [15 16],       'Active Power P [W] and Reactive Power Q [VAR]' ...
};

for s = 1:size(scope_defs,1)
    sc_name = scope_defs{s,1};
    sc_port = scope_defs{s,2};
    sc_titl = scope_defs{s,3};

    add_block('simulink/Sinks/Scope', [viz '/' sc_name]);
    set_param([viz '/' sc_name], ...
        'NumInputPorts', num2str(length(sc_port)), ...
        'Title', sc_titl, ...
        'Position', blkp(3, s*3-1, 140, 60));

    add_block('simulink/Signal Routing/Mux', [viz '/Mux_' sc_name]);
    set_param([viz '/Mux_' sc_name], ...
        'Inputs', num2str(length(sc_port)), ...
        'Position', blkp(2, s*3-1, 30, length(sc_port)*20));

    for j = 1:length(sc_port)
        add_line(viz, [viz_lbl{sc_port(j)} '/1'], ...
                      ['Mux_' sc_name '/' num2str(j)], 'autorouting','on');
    end
    add_line(viz, ['Mux_' sc_name '/1'], [sc_name '/1'], 'autorouting','on');
end
fprintf('[9/9] Visualization  (6 scopes)\n\n');

%% =========================================================
%%  TOP-LEVEL SIGNAL ROUTING
%% =========================================================
fprintf('[WIRING] Connecting top-level subsystems...\n');

% GridSource → Filter (grid ports 1-3) and abc2dq_V (ports 1-3)
for k = 1:3
    add_line(mdl, ['GridSource/' num2str(k)], ['Filter/' num2str(k)],   'autorouting','on');
    add_line(mdl, ['GridSource/' num2str(k)], ['abc2dq_V/' num2str(k)],'autorouting','on');
end

% Filter currents → abc2dq_I (ports 1-3)
for k = 1:3
    add_line(mdl, ['Filter/' num2str(k)], ['abc2dq_I/' num2str(k)], 'autorouting','on');
end

% PLL: Vq from voltage Park transform (port 2)
add_line(mdl, 'abc2dq_V/2', 'PLL/1', 'autorouting','on');

% PLL θ → all Park/InvPark angle inputs
add_line(mdl, 'PLL/1', 'abc2dq_V/4', 'autorouting','on');
add_line(mdl, 'PLL/1', 'abc2dq_I/4', 'autorouting','on');
add_line(mdl, 'PLL/1', 'dq2abc/3',   'autorouting','on');

% DroopControl: Vd, Vq, Id, Iq
add_line(mdl, 'abc2dq_V/1', 'DroopControl/1', 'autorouting','on');
add_line(mdl, 'abc2dq_V/2', 'DroopControl/2', 'autorouting','on');
add_line(mdl, 'abc2dq_I/1', 'DroopControl/3', 'autorouting','on');
add_line(mdl, 'abc2dq_I/2', 'DroopControl/4', 'autorouting','on');

% CurrentControl inputs
add_line(mdl, 'abc2dq_I/1',    'CurrentControl/1', 'autorouting','on'); % Id_meas
add_line(mdl, 'abc2dq_I/2',    'CurrentControl/2', 'autorouting','on'); % Iq_meas
add_line(mdl, 'DroopControl/1','CurrentControl/3', 'autorouting','on'); % Id_ref
add_line(mdl, 'DroopControl/2','CurrentControl/4', 'autorouting','on'); % Iq_ref
add_line(mdl, 'abc2dq_V/1',    'CurrentControl/5', 'autorouting','on'); % Vd_ff
add_line(mdl, 'abc2dq_V/2',    'CurrentControl/6', 'autorouting','on'); % Vq_ff
add_line(mdl, 'PLL/2',         'CurrentControl/7', 'autorouting','on'); % Omega

% CurrentControl → dq2abc
add_line(mdl, 'CurrentControl/1', 'dq2abc/1', 'autorouting','on'); % Vd_ref
add_line(mdl, 'CurrentControl/2', 'dq2abc/2', 'autorouting','on'); % Vq_ref

% dq2abc output voltages → Filter converter inputs (ports 4-6)
% This closes the main control loop: converter output → filter
for k = 1:3
    add_line(mdl, ['dq2abc/' num2str(k)], ['Filter/' num2str(k+3)], 'autorouting','on');
end

% dq2abc → SVPWM (gate signal generation — monitors reference voltages)
for k = 1:3
    add_line(mdl, ['dq2abc/' num2str(k)], ['SVPWM/' num2str(k)], 'autorouting','on');
end

% Visualization wiring
viz_wires = { ...
    'GridSource/1', 'Visualization/1';   % Va
    'GridSource/2', 'Visualization/2';   % Vb
    'GridSource/3', 'Visualization/3';   % Vc
    'Filter/1',     'Visualization/4';   % Ia
    'Filter/2',     'Visualization/5';   % Ib
    'Filter/3',     'Visualization/6';   % Ic
    'abc2dq_V/1',   'Visualization/7';   % Vd
    'abc2dq_V/2',   'Visualization/8';   % Vq
    'abc2dq_I/1',   'Visualization/9';   % Id
    'abc2dq_I/2',   'Visualization/10';  % Iq
    'DroopControl/1','Visualization/11'; % Id_ref
    'DroopControl/2','Visualization/12'; % Iq_ref
    'PLL/1',        'Visualization/13';  % Theta
    'PLL/2',        'Visualization/14';  % Omega
    'DroopControl/3','Visualization/15'; % P
    'DroopControl/4','Visualization/16'; % Q
};

for k = 1:size(viz_wires,1)
    try
        add_line(mdl, viz_wires{k,1}, viz_wires{k,2}, 'autorouting','on');
    catch ME
        warning('Skipped wire %s → %s: %s', viz_wires{k,1}, viz_wires{k,2}, ME.message);
    end
end

fprintf('   ✓ All connections established\n\n');

%% =========================================================
%%  SAVE
%% =========================================================
save_system(mdl, 'VSC_GridFollowing_Control.slx');

fprintf('=============================================\n');
fprintf('  MODEL SAVED: VSC_GridFollowing_Control.slx\n');
fprintf('=============================================\n');
fprintf('  Solver  : ode23tb  MaxStep=%gµs  T=%gs\n', Ts*1e6, T_sim);
fprintf('  Grid    : %.1fV pk  %gHz  →  %.1fV L-L RMS\n', V_grid_peak, f_grid, V_grid_ll);
fprintf('  Filter  : L=%.1fmH  R=%.2fΩ\n', L_filter*1e3, R_filter);
fprintf('  PLL     : Kp=%g  Ki=%g  (ζ=%.2f)\n', Kp_pll, Ki_pll, Kp_pll/(2*sqrt(Ki_pll)));
fprintf('  Droop   : Kp=%g  Kq=%g\n', Kp_droop, Kq_droop);
fprintf('  Current : Kp=%g  Ki=%g  BW=%.0frad/s\n', Kp_id, Ki_id, Kp_id/L_filter);
fprintf('  SVPWM   : %gkHz  6-gate\n', f_pwm/1e3);
fprintf('\n  To run:  sim(''%s'')\n', mdl);
fprintf('=============================================\n');

%% =========================================================
%%  LOCAL FUNCTIONS  (must be at end of script file)
%% =========================================================

function p = blkp(col, row, w, h)
%BLKP  Return a Simulink block position vector [x1, y1, x2, y2].
%
%   p = blkp(col, row)       returns a 100×40 block at grid (col,row)
%   p = blkp(col, row, w, h) returns a w×h block at grid (col,row)
%
%   Grid spacing: 180 px per column, 90 px per row.
    if nargin < 3; w = 100; end
    if nargin < 4; h = 40;  end
    x1 = (col - 1) * 180 + 30;
    y1 = (row - 1) * 90  + 30;
    p  = [x1, y1, x1 + w, y1 + h];
end

function buildParkSS(ss_path, bpos)
%BUILDPARKSS  Build a reusable abc→dq Park-transform subsystem.
%
%   ss_path : full Simulink path, e.g. 'myModel/abc2dq_V'
%   bpos    : block position [x1 y1 x2 y2] in parent diagram
%
%   Ports:
%     In  1: xA       Out 1: xd
%     In  2: xB       Out 2: xq
%     In  3: xC
%     In  4: Theta
%
%   Transform (amplitude-invariant):
%     alpha = (2/3)*(xA - xB/2 - xC/2)
%     beta  = (2/3)*(sqrt(3)/2*xB - sqrt(3)/2*xC)
%     xd    =  alpha*cos(theta) + beta*sin(theta)
%     xq    = -alpha*sin(theta) + beta*cos(theta)

    PH_local = {'A','B','C'};

    add_block('simulink/Ports & Subsystems/Subsystem', ss_path);
    set_param(ss_path, 'Position', bpos);
    Simulink.SubSystem.deleteContents(ss_path);

    % --- Inputs ---
    for k = 1:3
        add_block('simulink/Ports & Subsystems/In1', [ss_path '/x' PH_local{k}]);
        set_param([ss_path '/x' PH_local{k}], 'Port', num2str(k), 'Position', blkp(1,k,30,28));
    end
    add_block('simulink/Ports & Subsystems/In1', [ss_path '/Theta']);
    set_param([ss_path '/Theta'], 'Port','4', 'Position', blkp(1,4,30,28));

    % --- Clarke: alpha = (2/3)*(xA - xB/2 - xC/2) ---
    add_block('simulink/Math Operations/Gain', [ss_path '/KaA']); set_param([ss_path '/KaA'], 'Gain', '2/3',  'Position', blkp(2,1,60,28));
    add_block('simulink/Math Operations/Gain', [ss_path '/KaB']); set_param([ss_path '/KaB'], 'Gain', '-1/3', 'Position', blkp(2,2,60,28));
    add_block('simulink/Math Operations/Gain', [ss_path '/KaC']); set_param([ss_path '/KaC'], 'Gain', '-1/3', 'Position', blkp(2,3,60,28));
    add_block('simulink/Math Operations/Sum',  [ss_path '/SumA']); set_param([ss_path '/SumA'], 'Inputs','+++', 'Position', blkp(3,2,30,60));
    add_line(ss_path,'xA/1','KaA/1','autorouting','on'); add_line(ss_path,'KaA/1','SumA/1','autorouting','on');
    add_line(ss_path,'xB/1','KaB/1','autorouting','on'); add_line(ss_path,'KaB/1','SumA/2','autorouting','on');
    add_line(ss_path,'xC/1','KaC/1','autorouting','on'); add_line(ss_path,'KaC/1','SumA/3','autorouting','on');

    % --- Clarke: beta = sqrt(3)/3*xB - sqrt(3)/3*xC ---
    add_block('simulink/Math Operations/Gain', [ss_path '/KbB']); set_param([ss_path '/KbB'], 'Gain', 'sqrt(3)/3',  'Position', blkp(2,5,70,28));
    add_block('simulink/Math Operations/Gain', [ss_path '/KbC']); set_param([ss_path '/KbC'], 'Gain', '-sqrt(3)/3', 'Position', blkp(2,6,70,28));
    add_block('simulink/Math Operations/Sum',  [ss_path '/SumB']); set_param([ss_path '/SumB'], 'Inputs','++', 'Position', blkp(3,5,30,40));
    add_line(ss_path,'xB/1','KbB/1','autorouting','on'); add_line(ss_path,'KbB/1','SumB/1','autorouting','on');
    add_line(ss_path,'xC/1','KbC/1','autorouting','on'); add_line(ss_path,'KbC/1','SumB/2','autorouting','on');

    % --- Trig: cos(theta), sin(theta) ---
    add_block('simulink/Math Operations/Trigonometric Function', [ss_path '/CosT']); set_param([ss_path '/CosT'], 'Operator','cos', 'Position', blkp(3,3,60,28));
    add_block('simulink/Math Operations/Trigonometric Function', [ss_path '/SinT']); set_param([ss_path '/SinT'], 'Operator','sin', 'Position', blkp(3,4,60,28));
    add_line(ss_path,'Theta/1','CosT/1','autorouting','on');
    add_line(ss_path,'Theta/1','SinT/1','autorouting','on');

    % --- Park: xd = alpha*cos + beta*sin ---
    add_block('simulink/Math Operations/Product', [ss_path '/Pac']); set_param([ss_path '/Pac'], 'Position', blkp(4,2,30,28));
    add_block('simulink/Math Operations/Product', [ss_path '/Pbs']); set_param([ss_path '/Pbs'], 'Position', blkp(4,3,30,28));
    add_block('simulink/Math Operations/Sum',     [ss_path '/Sxd']); set_param([ss_path '/Sxd'], 'Inputs','++', 'Position', blkp(5,2,30,30));
    add_line(ss_path,'SumA/1','Pac/1','autorouting','on'); add_line(ss_path,'CosT/1','Pac/2','autorouting','on');
    add_line(ss_path,'SumB/1','Pbs/1','autorouting','on'); add_line(ss_path,'SinT/1','Pbs/2','autorouting','on');
    add_line(ss_path,'Pac/1','Sxd/1','autorouting','on'); add_line(ss_path,'Pbs/1','Sxd/2','autorouting','on');

    % --- Park: xq = -alpha*sin + beta*cos ---
    add_block('simulink/Math Operations/Product', [ss_path '/Pas']); set_param([ss_path '/Pas'], 'Position', blkp(4,5,30,28));
    add_block('simulink/Math Operations/Product', [ss_path '/Pbc']); set_param([ss_path '/Pbc'], 'Position', blkp(4,6,30,28));
    add_block('simulink/Math Operations/Gain',    [ss_path '/NegS']); set_param([ss_path '/NegS'], 'Gain','-1', 'Position', blkp(5,5,40,28));
    add_block('simulink/Math Operations/Sum',     [ss_path '/Sxq']); set_param([ss_path '/Sxq'], 'Inputs','++', 'Position', blkp(6,5,30,30));
    add_line(ss_path,'SumA/1','Pas/1','autorouting','on'); add_line(ss_path,'SinT/1','Pas/2','autorouting','on');
    add_line(ss_path,'SumB/1','Pbc/1','autorouting','on'); add_line(ss_path,'CosT/1','Pbc/2','autorouting','on');
    add_line(ss_path,'Pas/1','NegS/1','autorouting','on');
    add_line(ss_path,'NegS/1','Sxq/1','autorouting','on');
    add_line(ss_path,'Pbc/1','Sxq/2','autorouting','on');

    % --- Outputs ---
    add_block('simulink/Ports & Subsystems/Out1', [ss_path '/xd']); set_param([ss_path '/xd'], 'Port','1', 'Position', blkp(6,2,30,28));
    add_block('simulink/Ports & Subsystems/Out1', [ss_path '/xq']); set_param([ss_path '/xq'], 'Port','2', 'Position', blkp(7,5,30,28));
    add_line(ss_path,'Sxd/1','xd/1','autorouting','on');
    add_line(ss_path,'Sxq/1','xq/1','autorouting','on');
end
