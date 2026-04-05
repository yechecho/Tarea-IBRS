%% ========================================================================
%% create_VSC_GridFollowing_Complete.m
%% ========================================================================
%% Crea modelo Simulink: VSC Grid-Following con Control Vectorial completo
%%
%% Tarea de Simulacion II - Curso: Modelizado de sistemas electricos
%%
%% ESTRUCTURA DEL MODELO (9 subsistemas + 8 scopes):
%%   GridSource       - Fuente trifasica senoidal 60 Hz
%%   PLL              - Phase Locked Loop explicito (PI visible, theta/omega)
%%   ParkTransform    - Transformacion Park abc->dq  (MATLAB Function)
%%   InvParkTransform - Transformacion Park inversa dq->abc
%%   OuterLoop        - Lazo externo: P*,Q* -> id*,iq*  (division por 1.5*Vd)
%%   InnerLoop        - Lazo interno: PI corriente id, iq con desacoplamiento
%%   FilterModel      - Ecuaciones filtro RL en marco sincrono dq
%%   PowerCalc        - Calculo P=1.5(vd*id+vq*iq), Q=1.5(vd*iq-vq*id)
%%   DroopControl     - Control droop P-omega y Q-V
%%
%% VARIABLES WORKSPACE requeridas (cargadas por VSCm_Grid_Connected.m):
%%   f_grid, V_grid_pk, omega_grid, V_dc
%%   L_filter, R_filter
%%   Kp_pll, Ki_pll
%%   Kp_id, Ki_id, Kp_iq, Ki_iq
%%   kp_droop, kq_droop, V_ref
%%   P_ref, Q_ref, P_ref_step, Q_ref_step, t_step_P, t_step_Q
%%
%% INSTRUCCIONES:
%%   1. Ejecute VSCm_Grid_Connected primero (carga parametros)
%%   2. Ejecute: >> create_VSC_GridFollowing_Complete
%%   3. Se abrira VSC_GridFollowing_Control.slx en Simulink
%%   4. Presione Ctrl+T para simular 0.3 segundos
%% ========================================================================

clear all; close all; clc;

%% ---- Cargar parametros ----
VSCm_Grid_Connected;

%% Alias necesario para DroopControl (voltaje de referencia nominal)
V_ref_nom = V_ref;

%% ========================================================================
%% CREAR MODELO
%% ========================================================================
model_name = 'VSC_GridFollowing_Control';

if bdIsLoaded(model_name)
    bdclose(model_name);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'Solver', 'ode45', 'RelTol', '1e-4', ...
    'MaxStep', '1e-4', 'StopTime', '0.3', 'SimulationMode', 'normal');

fprintf('Modelo "%s" creado. Creando subsistemas...\n\n', model_name);

%% ========================================================================
%% SUBSISTEMA 1: GridSource - Fuente trifasica
%% ========================================================================
gs = [model_name '/GridSource'];
add_block('simulink/Ports & Subsystems/Subsystem', gs, ...
    'Position', [50,200,150,260], 'BackgroundColor', '[0.0,0.45,0.74]');
Simulink.SubSystem.deleteContents(gs);

add_block('simulink/Sources/Sine Wave', [gs '/Va'], ...
    'Amplitude','V_grid_pk','Frequency','f_grid','Phase','0', ...
    'Position',[30,30,85,60]);
add_block('simulink/Sources/Sine Wave', [gs '/Vb'], ...
    'Amplitude','V_grid_pk','Frequency','f_grid','Phase','-2*pi/3', ...
    'Position',[30,90,85,120]);
add_block('simulink/Sources/Sine Wave', [gs '/Vc'], ...
    'Amplitude','V_grid_pk','Frequency','f_grid','Phase','2*pi/3', ...
    'Position',[30,150,85,180]);
add_block('simulink/Signal Routing/Mux', [gs '/Mux'], ...
    'Inputs','3','Position',[130,55,135,155]);
add_block('simulink/Ports & Subsystems/Out1', [gs '/Vabc'], ...
    'Position',[170,95,200,115]);

add_line(gs,'Va/1','Mux/1'); add_line(gs,'Vb/1','Mux/2');
add_line(gs,'Vc/1','Mux/3'); add_line(gs,'Mux/1','Vabc/1');
fprintf('  [OK] GridSource\n');

%% ========================================================================
%% SUBSISTEMA 2: PLL - Phase Locked Loop (SRF-PLL explicito)
%% ========================================================================
%% Algoritmo:
%%   1. Calcular vq usando transformacion Park con theta estimado
%%   2. PI: omega = omega0 + Kp_pll*vq + Ki_pll*INT(vq)
%%   3. theta = INT(omega) mod 2*pi
%%
%% Nota: vq->0 en estado estable indica sincronizacion correcta

pll = [model_name '/PLL'];
add_block('simulink/Ports & Subsystems/Subsystem', pll, ...
    'Position', [200,200,340,260], 'BackgroundColor', '[0.93,0.69,0.13]');
Simulink.SubSystem.deleteContents(pll);

%% Puerto de entrada
add_block('simulink/Ports & Subsystems/In1', [pll '/Vabc'], ...
    'Position',[20,130,50,150],'Port','1');

%% MATLAB Function para calcular vq (solo usa entradas y pi, no vars workspace)
add_block('simulink/User-Defined Functions/MATLAB Function', [pll '/calc_vq'], ...
    'Position',[100,110,230,170]);

%% Controlador PI del PLL (bloques estandar - pueden referenciar vars workspace)
add_block('simulink/Math Operations/Gain', [pll '/Kp_PLL'], ...
    'Gain','Kp_pll','Position',[280,108,330,132]);
add_block('simulink/Continuous/Integrator', [pll '/Int_vq'], ...
    'InitialCondition','0','Position',[280,145,315,175]);
add_block('simulink/Math Operations/Gain', [pll '/Ki_PLL'], ...
    'Gain','Ki_pll','Position',[330,145,375,175]);

%% omega = omega0 + Kp*vq + Ki*INT(vq)
add_block('simulink/Sources/Constant', [pll '/omega0'], ...
    'Value','omega_grid','Position',[285,65,330,90]);
add_block('simulink/Math Operations/Add', [pll '/Sum_omega'], ...
    'Inputs','+++','Position',[395,73,420,175]);

%% theta = INT(omega) mod 2*pi
add_block('simulink/Continuous/Integrator', [pll '/Int_theta'], ...
    'InitialCondition','0','Position',[445,105,480,140]);
add_block('simulink/Math Operations/Math Function', [pll '/mod2pi'], ...
    'Operator','mod','Position',[505,100,545,145]);
add_block('simulink/Sources/Constant', [pll '/twopi'], ...
    'Value','2*pi','Position',[455,155,490,175]);

%% Outports
add_block('simulink/Ports & Subsystems/Out1', [pll '/theta'], ...
    'Position',[575,108,605,128],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [pll '/omega'], ...
    'Position',[575,148,605,168],'Port','2');

%% Conexiones del PLL
add_line(pll,'Vabc/1',       'calc_vq/1');
add_line(pll,'Int_theta/1',  'calc_vq/2');   % theta (feedback)
add_line(pll,'calc_vq/1',    'Kp_PLL/1');
add_line(pll,'calc_vq/1',    'Int_vq/1');
add_line(pll,'Int_vq/1',     'Ki_PLL/1');
add_line(pll,'omega0/1',     'Sum_omega/1');
add_line(pll,'Kp_PLL/1',     'Sum_omega/2');
add_line(pll,'Ki_PLL/1',     'Sum_omega/3');
add_line(pll,'Sum_omega/1',  'Int_theta/1');
add_line(pll,'Int_theta/1',  'mod2pi/1');
add_line(pll,'twopi/1',      'mod2pi/2');
add_line(pll,'mod2pi/1',     'theta/1');
add_line(pll,'Sum_omega/1',  'omega/1');

fprintf('  [OK] PLL (Kp_pll=%g, Ki_pll=%g)\n', Kp_pll, Ki_pll);

%% ========================================================================
%% SUBSISTEMA 3: ParkTransform - Transformacion abc -> dq
%% ========================================================================
pt = [model_name '/ParkTransform'];
add_block('simulink/Ports & Subsystems/Subsystem', pt, ...
    'Position', [200,320,340,380], 'BackgroundColor', '[0.47,0.67,0.19]');
Simulink.SubSystem.deleteContents(pt);

add_block('simulink/Ports & Subsystems/In1', [pt '/Xabc'], ...
    'Position',[20,60,50,80],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [pt '/theta'], ...
    'Position',[20,110,50,130],'Port','2');
add_block('simulink/User-Defined Functions/MATLAB Function', [pt '/Park_abc2dq'], ...
    'Position',[100,70,230,120]);
add_block('simulink/Ports & Subsystems/Out1', [pt '/Xdq'], ...
    'Position',[270,82,300,102]);

add_line(pt,'Xabc/1','Park_abc2dq/1');
add_line(pt,'theta/1','Park_abc2dq/2');
add_line(pt,'Park_abc2dq/1','Xdq/1');
fprintf('  [OK] ParkTransform (abc->dq)\n');

%% ========================================================================
%% SUBSISTEMA 4: InvParkTransform - Transformacion dq -> abc
%% ========================================================================
ipt = [model_name '/InvParkTransform'];
add_block('simulink/Ports & Subsystems/Subsystem', ipt, ...
    'Position', [200,430,340,490], 'BackgroundColor', '[0.47,0.67,0.19]');
Simulink.SubSystem.deleteContents(ipt);

add_block('simulink/Ports & Subsystems/In1', [ipt '/Xdq'], ...
    'Position',[20,60,50,80],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [ipt '/theta'], ...
    'Position',[20,110,50,130],'Port','2');
add_block('simulink/User-Defined Functions/MATLAB Function', [ipt '/InvPark_dq2abc'], ...
    'Position',[100,70,230,120]);
add_block('simulink/Ports & Subsystems/Out1', [ipt '/Xabc'], ...
    'Position',[270,82,300,102]);

add_line(ipt,'Xdq/1','InvPark_dq2abc/1');
add_line(ipt,'theta/1','InvPark_dq2abc/2');
add_line(ipt,'InvPark_dq2abc/1','Xabc/1');
fprintf('  [OK] InvParkTransform (dq->abc)\n');

%% ========================================================================
%% SUBSISTEMA 5: OuterLoop - Lazo Externo de Potencia
%% ========================================================================
%% id* =  P* / (1.5 * Vd)
%% iq* = -Q* / (1.5 * Vd)

ol = [model_name '/OuterLoop'];
add_block('simulink/Ports & Subsystems/Subsystem', ol, ...
    'Position', [400,200,540,300], 'BackgroundColor', '[0.47,0.67,0.19]');
Simulink.SubSystem.deleteContents(ol);

add_block('simulink/Ports & Subsystems/In1', [ol '/P_ref'], ...
    'Position',[20,20,50,40],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [ol '/Q_ref'], ...
    'Position',[20,70,50,90],'Port','2');
add_block('simulink/Ports & Subsystems/In1', [ol '/vd'], ...
    'Position',[20,120,50,140],'Port','3');

%% Calcular 1.5*Vd (denominador), protegido contra division por cero
add_block('simulink/Math Operations/Abs', [ol '/abs_vd'], ...
    'Position',[85,120,115,150]);
add_block('simulink/Sources/Constant', [ol '/min_vd'], ...
    'Value','1','Position',[120,155,155,175]);
add_block('simulink/Math Operations/MinMax', [ol '/max_vd'], ...
    'Function','max','Inputs','2','Position',[175,122,215,158]);
add_block('simulink/Math Operations/Gain', [ol '/gain_15'], ...
    'Gain','1.5','Position',[235,122,265,158]);

%% id* = P* / (1.5*Vd)
add_block('simulink/Math Operations/Divide', [ol '/div_id'], ...
    'Position',[295,20,325,60]);
%% iq* = -Q* / (1.5*Vd)
add_block('simulink/Math Operations/Gain', [ol '/neg1'], ...
    'Gain','-1','Position',[85,68,120,92]);
add_block('simulink/Math Operations/Divide', [ol '/div_iq'], ...
    'Position',[295,70,325,110]);

%% Saturaciones de corriente +/-100A
add_block('simulink/Discontinuities/Saturation', [ol '/sat_id'], ...
    'UpperLimit','100','LowerLimit','-100','Position',[345,25,380,55]);
add_block('simulink/Discontinuities/Saturation', [ol '/sat_iq'], ...
    'UpperLimit','100','LowerLimit','-100','Position',[345,75,380,105]);

add_block('simulink/Ports & Subsystems/Out1', [ol '/id_ref'], ...
    'Position',[405,30,435,50],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [ol '/iq_ref'], ...
    'Position',[405,80,435,100],'Port','2');

add_line(ol,'vd/1','abs_vd/1');
add_line(ol,'abs_vd/1','max_vd/1');
add_line(ol,'min_vd/1','max_vd/2');
add_line(ol,'max_vd/1','gain_15/1');
add_line(ol,'gain_15/1','div_id/2');
add_line(ol,'gain_15/1','div_iq/2');
add_line(ol,'P_ref/1','div_id/1');
add_line(ol,'Q_ref/1','neg1/1');
add_line(ol,'neg1/1','div_iq/1');
add_line(ol,'div_id/1','sat_id/1');
add_line(ol,'div_iq/1','sat_iq/1');
add_line(ol,'sat_id/1','id_ref/1');
add_line(ol,'sat_iq/1','iq_ref/1');
fprintf('  [OK] OuterLoop (P*,Q* -> id*,iq*)\n');

%% ========================================================================
%% SUBSISTEMA 6: InnerLoop - Lazo Interno de Corriente (PI id, iq)
%% ========================================================================
%% vd* = Kp_id*(id*-id) + Ki_id*INT(id*-id) - omega*L*iq + vgd
%% vq* = Kp_iq*(iq*-iq) + Ki_iq*INT(iq*-iq) + omega*L*id + vgq

il = [model_name '/InnerLoop'];
add_block('simulink/Ports & Subsystems/Subsystem', il, ...
    'Position', [580,200,720,300], 'BackgroundColor', '[0.93,0.69,0.13]');
Simulink.SubSystem.deleteContents(il);

%% Inports
add_block('simulink/Ports & Subsystems/In1', [il '/id_ref'], 'Position',[20,15,50,35],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [il '/iq_ref'], 'Position',[20,55,50,75],'Port','2');
add_block('simulink/Ports & Subsystems/In1', [il '/id'],     'Position',[20,95,50,115],'Port','3');
add_block('simulink/Ports & Subsystems/In1', [il '/iq'],     'Position',[20,135,50,155],'Port','4');
add_block('simulink/Ports & Subsystems/In1', [il '/vgd'],    'Position',[20,175,50,195],'Port','5');
add_block('simulink/Ports & Subsystems/In1', [il '/vgq'],    'Position',[20,215,50,235],'Port','6');
add_block('simulink/Ports & Subsystems/In1', [il '/omega'],  'Position',[20,255,50,275],'Port','7');

%% Errores d y q
add_block('simulink/Math Operations/Add', [il '/err_d'], ...
    'Inputs','+-','Position',[90,18,115,52]);
add_block('simulink/Math Operations/Add', [il '/err_q'], ...
    'Inputs','+-','Position',[90,118,115,152]);

%% PI eje d
add_block('simulink/Math Operations/Gain',       [il '/Kp_d'], 'Gain','Kp_id','Position',[145,18,185,42]);
add_block('simulink/Continuous/Integrator',      [il '/Int_d'],'InitialCondition','0','Position',[145,55,185,85]);
add_block('simulink/Math Operations/Gain',       [il '/Ki_d'], 'Gain','Ki_id','Position',[200,55,240,85]);
add_block('simulink/Math Operations/Add',        [il '/PI_d'], 'Inputs','++','Position',[255,22,280,60]);

%% PI eje q
add_block('simulink/Math Operations/Gain',       [il '/Kp_q'], 'Gain','Kp_iq','Position',[145,118,185,142]);
add_block('simulink/Continuous/Integrator',      [il '/Int_q'],'InitialCondition','0','Position',[145,155,185,185]);
add_block('simulink/Math Operations/Gain',       [il '/Ki_q'], 'Gain','Ki_iq','Position',[200,155,240,185]);
add_block('simulink/Math Operations/Add',        [il '/PI_q'], 'Inputs','++','Position',[255,122,280,160]);

%% Desacoplamiento: -omega*L*iq (para eje d), +omega*L*id (para eje q)
add_block('simulink/Math Operations/Product', [il '/oL_iq'],'Position',[90,205,125,235]);
add_block('simulink/Math Operations/Gain',    [il '/neg_L'],'Gain','-L_filter','Position',[140,205,185,235]);
add_block('simulink/Math Operations/Product', [il '/oL_id'],'Position',[90,255,125,285]);
add_block('simulink/Math Operations/Gain',    [il '/pos_L'],'Gain','L_filter','Position',[140,255,185,285]);

%% Suma total: PI + desacoplamiento + feedforward de red
add_block('simulink/Math Operations/Add', [il '/Sum_vd'], 'Inputs','+++','Position',[310,15,335,230]);
add_block('simulink/Math Operations/Add', [il '/Sum_vq'], 'Inputs','+++','Position',[310,115,335,290]);

%% Saturacion de salida: +/- Vdc/2
add_block('simulink/Discontinuities/Saturation', [il '/Sat_vd'], ...
    'UpperLimit','V_dc/2','LowerLimit','-V_dc/2','Position',[355,90,395,130]);
add_block('simulink/Discontinuities/Saturation', [il '/Sat_vq'], ...
    'UpperLimit','V_dc/2','LowerLimit','-V_dc/2','Position',[355,180,395,220]);

add_block('simulink/Ports & Subsystems/Out1', [il '/vd_ref'], 'Position',[420,98,450,118],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [il '/vq_ref'], 'Position',[420,188,450,208],'Port','2');

%% Conexiones InnerLoop
add_line(il,'id_ref/1','err_d/1'); add_line(il,'id/1','err_d/2');
add_line(il,'iq_ref/1','err_q/1'); add_line(il,'iq/1','err_q/2');
add_line(il,'err_d/1','Kp_d/1');   add_line(il,'err_d/1','Int_d/1');
add_line(il,'err_q/1','Kp_q/1');   add_line(il,'err_q/1','Int_q/1');
add_line(il,'Int_d/1','Ki_d/1');   add_line(il,'Int_q/1','Ki_q/1');
add_line(il,'Kp_d/1','PI_d/1');    add_line(il,'Ki_d/1','PI_d/2');
add_line(il,'Kp_q/1','PI_q/1');    add_line(il,'Ki_q/1','PI_q/2');
add_line(il,'omega/1','oL_iq/1');  add_line(il,'iq/1','oL_iq/2');
add_line(il,'oL_iq/1','neg_L/1');
add_line(il,'omega/1','oL_id/1');  add_line(il,'id/1','oL_id/2');
add_line(il,'oL_id/1','pos_L/1');
add_line(il,'PI_d/1',   'Sum_vd/1'); add_line(il,'neg_L/1','Sum_vd/2'); add_line(il,'vgd/1','Sum_vd/3');
add_line(il,'PI_q/1',   'Sum_vq/1'); add_line(il,'pos_L/1','Sum_vq/2'); add_line(il,'vgq/1','Sum_vq/3');
add_line(il,'Sum_vd/1','Sat_vd/1'); add_line(il,'Sat_vd/1','vd_ref/1');
add_line(il,'Sum_vq/1','Sat_vq/1'); add_line(il,'Sat_vq/1','vq_ref/1');
fprintf('  [OK] InnerLoop (Kp_id=%g Ki_id=%g, Kp_iq=%g Ki_iq=%g)\n', Kp_id,Ki_id,Kp_iq,Ki_iq);

%% ========================================================================
%% SUBSISTEMA 7: FilterModel - Filtro RL en marco sincrono dq
%% ========================================================================
%% L*did/dt = vd* - vgd - R*id + omega*L*iq
%% L*diq/dt = vq* - vgq - R*iq - omega*L*id

fm = [model_name '/FilterModel'];
add_block('simulink/Ports & Subsystems/Subsystem', fm, ...
    'Position', [760,200,880,300], 'BackgroundColor', '[0.85,0.33,0.10]');
Simulink.SubSystem.deleteContents(fm);

add_block('simulink/Ports & Subsystems/In1', [fm '/vd_ref'], 'Position',[20,15,50,35],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [fm '/vq_ref'], 'Position',[20,55,50,75],'Port','2');
add_block('simulink/Ports & Subsystems/In1', [fm '/vgd'],    'Position',[20,95,50,115],'Port','3');
add_block('simulink/Ports & Subsystems/In1', [fm '/vgq'],    'Position',[20,135,50,155],'Port','4');
add_block('simulink/Ports & Subsystems/In1', [fm '/omega'],  'Position',[20,175,50,195],'Port','5');

%% Ecuacion eje d: L*did/dt = vd* - vgd - R*id + omega*L*iq
%% Sum_d ports: (+)vd* (--)vgd (--)R*id (+)omega*L*iq  -> '+--+'
add_block('simulink/Math Operations/Product', [fm '/oL_iq'], 'Position',[70,185,105,215]);
add_block('simulink/Math Operations/Gain',    [fm '/L_iq'],  'Gain','L_filter','Position',[115,185,155,215]);
add_block('simulink/Math Operations/Add',     [fm '/Sum_d'], 'Inputs','+--+','Position',[195,10,220,55]);
add_block('simulink/Math Operations/Gain',    [fm '/L_inv_d'],'Gain','1/L_filter','Position',[235,20,275,45]);
add_block('simulink/Continuous/Integrator',   [fm '/Int_id'],'InitialCondition','0','Position',[295,20,330,50]);
add_block('simulink/Math Operations/Gain',    [fm '/R_id'],   'Gain','R_filter','Position',[120,55,155,80]);

%% Ecuacion eje q: L*diq/dt = vq* - vgq - R*iq - omega*L*id
%% Sum_q ports: (+)vq* (--)vgq (--)R*iq (--)omega*L*id -> '+---'
add_block('simulink/Math Operations/Product', [fm '/oL_id'], 'Position',[70,235,105,265]);
add_block('simulink/Math Operations/Gain',    [fm '/L_id'],  'Gain','L_filter','Position',[115,235,155,265]);
add_block('simulink/Math Operations/Add',     [fm '/Sum_q'], 'Inputs','+---','Position',[195,110,220,155]);
add_block('simulink/Math Operations/Gain',    [fm '/L_inv_q'],'Gain','1/L_filter','Position',[235,120,275,145]);
add_block('simulink/Continuous/Integrator',   [fm '/Int_iq'],'InitialCondition','0','Position',[295,120,330,150]);
add_block('simulink/Math Operations/Gain',    [fm '/R_iq'],   'Gain','R_filter','Position',[120,155,155,180]);

%% Outports
add_block('simulink/Ports & Subsystems/Out1', [fm '/id'], 'Position',[365,28,395,48],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [fm '/iq'], 'Position',[365,128,395,148],'Port','2');

%% Conexiones FilterModel eje d
add_line(fm,'omega/1',  'oL_iq/1'); add_line(fm,'Int_iq/1','oL_iq/2');
add_line(fm,'oL_iq/1',  'L_iq/1');
add_line(fm,'Int_id/1', 'R_id/1');
add_line(fm,'vd_ref/1', 'Sum_d/1'); add_line(fm,'vgd/1',  'Sum_d/2');
add_line(fm,'R_id/1',   'Sum_d/3'); add_line(fm,'L_iq/1', 'Sum_d/4');
add_line(fm,'Sum_d/1',  'L_inv_d/1');
add_line(fm,'L_inv_d/1','Int_id/1');
add_line(fm,'Int_id/1', 'id/1');

%% Conexiones FilterModel eje q
add_line(fm,'omega/1',  'oL_id/1'); add_line(fm,'Int_id/1','oL_id/2');
add_line(fm,'oL_id/1',  'L_id/1');
add_line(fm,'Int_iq/1', 'R_iq/1');
add_line(fm,'vq_ref/1', 'Sum_q/1'); add_line(fm,'vgq/1',  'Sum_q/2');
add_line(fm,'R_iq/1',   'Sum_q/3'); add_line(fm,'L_id/1', 'Sum_q/4');
add_line(fm,'Sum_q/1',  'L_inv_q/1');
add_line(fm,'L_inv_q/1','Int_iq/1');
add_line(fm,'Int_iq/1', 'iq/1');
fprintf('  [OK] FilterModel (L=%gmH, R=%gOhm)\n', L_filter*1e3, R_filter);

%% ========================================================================
%% SUBSISTEMA 8: PowerCalc - Calculo de potencia P y Q
%% ========================================================================
%% P = 1.5*(vd*id + vq*iq)
%% Q = 1.5*(vd*iq - vq*id)

pw = [model_name '/PowerCalc'];
add_block('simulink/Ports & Subsystems/Subsystem', pw, ...
    'Position', [920,200,1040,300], 'BackgroundColor', '[0.49,0.18,0.56]');
Simulink.SubSystem.deleteContents(pw);

add_block('simulink/Ports & Subsystems/In1', [pw '/vd'], 'Position',[20,20,50,40],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [pw '/vq'], 'Position',[20,60,50,80],'Port','2');
add_block('simulink/Ports & Subsystems/In1', [pw '/id'], 'Position',[20,100,50,120],'Port','3');
add_block('simulink/Ports & Subsystems/In1', [pw '/iq'], 'Position',[20,140,50,160],'Port','4');

add_block('simulink/Math Operations/Product', [pw '/vd_id'], 'Position',[85,20,110,45]);
add_block('simulink/Math Operations/Product', [pw '/vq_iq'], 'Position',[85,58,110,83]);
add_block('simulink/Math Operations/Add',     [pw '/Sum_P'], 'Inputs','++','Position',[135,30,160,68]);
add_block('simulink/Math Operations/Gain',    [pw '/G_P'],   'Gain','1.5','Position',[175,35,210,63]);

add_block('simulink/Math Operations/Product', [pw '/vd_iq'], 'Position',[85,98,110,123]);
add_block('simulink/Math Operations/Product', [pw '/vq_id'], 'Position',[85,136,110,161]);
add_block('simulink/Math Operations/Add',     [pw '/Sum_Q'], 'Inputs','+-','Position',[135,108,160,148]);
add_block('simulink/Math Operations/Gain',    [pw '/G_Q'],   'Gain','1.5','Position',[175,113,210,143]);

add_block('simulink/Ports & Subsystems/Out1', [pw '/P'], 'Position',[235,40,265,60],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [pw '/Q'], 'Position',[235,118,265,138],'Port','2');

add_line(pw,'vd/1','vd_id/1'); add_line(pw,'id/1','vd_id/2');
add_line(pw,'vq/1','vq_iq/1'); add_line(pw,'iq/1','vq_iq/2');
add_line(pw,'vd_id/1','Sum_P/1'); add_line(pw,'vq_iq/1','Sum_P/2');
add_line(pw,'Sum_P/1','G_P/1'); add_line(pw,'G_P/1','P/1');

add_line(pw,'vd/1','vd_iq/1'); add_line(pw,'iq/1','vd_iq/2');
add_line(pw,'vq/1','vq_id/1'); add_line(pw,'id/1','vq_id/2');
add_line(pw,'vd_iq/1','Sum_Q/1'); add_line(pw,'vq_id/1','Sum_Q/2');
add_line(pw,'Sum_Q/1','G_Q/1'); add_line(pw,'G_Q/1','Q/1');
fprintf('  [OK] PowerCalc (P=1.5(vd*id+vq*iq), Q)\n');

%% ========================================================================
%% SUBSISTEMA 9: DroopControl - Control Droop P-omega y Q-V
%% ========================================================================
%% omega_ref = omega0 - kp_droop*(P - P*)
%% V_ref     = V0    - kq_droop*(Q - Q*)

dr = [model_name '/DroopControl'];
add_block('simulink/Ports & Subsystems/Subsystem', dr, ...
    'Position', [1100,200,1220,300], 'BackgroundColor', '[0.30,0.75,0.93]');
Simulink.SubSystem.deleteContents(dr);

add_block('simulink/Ports & Subsystems/In1', [dr '/P'],     'Position',[20,20,50,40],'Port','1');
add_block('simulink/Ports & Subsystems/In1', [dr '/Q'],     'Position',[20,65,50,85],'Port','2');
add_block('simulink/Ports & Subsystems/In1', [dr '/P_ref'], 'Position',[20,110,50,130],'Port','3');
add_block('simulink/Ports & Subsystems/In1', [dr '/Q_ref'], 'Position',[20,155,50,175],'Port','4');

%% Droop P-omega: omega_ref = omega0 - kp_droop*(P - P*)
add_block('simulink/Math Operations/Add',  [dr '/dP'],       'Inputs','+-','Position',[90,22,115,58]);
add_block('simulink/Math Operations/Gain', [dr '/kp_d'],     'Gain','kp_droop','Position',[135,28,180,52]);
add_block('simulink/Sources/Constant',     [dr '/w0'],       'Value','omega_grid','Position',[135,65,180,90]);
add_block('simulink/Math Operations/Add',  [dr '/Sum_wref'], 'Inputs','+-','Position',[210,40,240,85]);

%% Droop Q-V: V_ref = V0 - kq_droop*(Q - Q*)
add_block('simulink/Math Operations/Add',  [dr '/dQ'],       'Inputs','+-','Position',[90,112,115,148]);
add_block('simulink/Math Operations/Gain', [dr '/kq_d'],     'Gain','kq_droop','Position',[135,118,180,142]);
add_block('simulink/Sources/Constant',     [dr '/V0'],       'Value','V_ref_nom','Position',[135,158,180,183]);
add_block('simulink/Math Operations/Add',  [dr '/Sum_Vref'], 'Inputs','+-','Position',[210,135,240,180]);

add_block('simulink/Ports & Subsystems/Out1', [dr '/omega_ref'], 'Position',[265,52,295,72],'Port','1');
add_block('simulink/Ports & Subsystems/Out1', [dr '/V_ref'],     'Position',[265,147,295,167],'Port','2');

add_line(dr,'P/1','dP/1');     add_line(dr,'P_ref/1','dP/2');
add_line(dr,'dP/1','kp_d/1');
add_line(dr,'w0/1','Sum_wref/1'); add_line(dr,'kp_d/1','Sum_wref/2');
add_line(dr,'Sum_wref/1','omega_ref/1');

add_line(dr,'Q/1','dQ/1');     add_line(dr,'Q_ref/1','dQ/2');
add_line(dr,'dQ/1','kq_d/1');
add_line(dr,'V0/1','Sum_Vref/1'); add_line(dr,'kq_d/1','Sum_Vref/2');
add_line(dr,'Sum_Vref/1','V_ref/1');
fprintf('  [OK] DroopControl (kp=%g, kq=%g)\n', kp_droop, kq_droop);

%% ========================================================================
%% BLOQUES DE PERTURBACION: escalones en P* y Q*
%% ========================================================================
add_block('simulink/Sources/Step', [model_name '/Step_Pref'], ...
    'Time',   num2str(t_step_P), ...
    'Before', num2str(P_ref), ...
    'After',  num2str(P_ref_step), ...
    'Position', [50,500,120,540]);
add_block('simulink/Sources/Step', [model_name '/Step_Qref'], ...
    'Time',   num2str(t_step_Q), ...
    'Before', num2str(Q_ref), ...
    'After',  num2str(Q_ref_step), ...
    'Position', [50,570,120,610]);
fprintf('  [OK] Perturbaciones (P*: %g->%g @t=%gs, Q*: %g->%g @t=%gs)\n', ...
    P_ref,P_ref_step,t_step_P, Q_ref,Q_ref_step,t_step_Q);

%% ========================================================================
%% BLOQUES AUXILIARES EN EL NIVEL SUPERIOR
%% ========================================================================
%% Demux salida de ParkTransform -> vd, vq separados
add_block('simulink/Signal Routing/Demux', [model_name '/Demux_Vdq'], ...
    'Outputs','2','Position',[390,330,395,380]);

%% Mux vd_ref y vq_ref para InvParkTransform
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_vdq_ref'], ...
    'Inputs','2','Position',[50,640,55,710]);

%% ========================================================================
%% SCOPES PARA ANALISIS
%% ========================================================================
add_block('simulink/Sinks/Scope', [model_name '/Scope_PLL'], ...
    'NumInputPorts','2','Position',[1300,50,1360,110]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_CurrentsDQ'], ...
    'NumInputPorts','4','Position',[1300,140,1360,200]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_VoltagesDQ'], ...
    'NumInputPorts','2','Position',[1300,230,1360,290]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Power'], ...
    'NumInputPorts','2','Position',[1300,320,1360,380]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Vabc_Grid'], ...
    'NumInputPorts','1','Position',[1300,410,1360,470]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Vref_abc'], ...
    'NumInputPorts','1','Position',[1300,500,1360,560]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_PowerRefs'], ...
    'NumInputPorts','2','Position',[1300,590,1360,650]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Droop'], ...
    'NumInputPorts','2','Position',[1300,680,1360,740]);
fprintf('  [OK] Scopes (8 total)\n');

%% ========================================================================
%% INTERCONEXIONES EN EL NIVEL SUPERIOR
%% ========================================================================
fprintf('\nConectando subsistemas...\n');

%% 1. GridSource -> PLL, ParkTransform (voltajes), Scope_Vabc_Grid
add_line(model_name,'GridSource/1', 'PLL/1',              'autorouting','on');
add_line(model_name,'GridSource/1', 'ParkTransform/1',    'autorouting','on');
add_line(model_name,'GridSource/1', 'Scope_Vabc_Grid/1',  'autorouting','on');

%% 2. PLL: theta -> ParkTransform, InvParkTransform
%%         omega -> InnerLoop, FilterModel
add_line(model_name,'PLL/1', 'ParkTransform/2',    'autorouting','on');
add_line(model_name,'PLL/1', 'InvParkTransform/2', 'autorouting','on');
add_line(model_name,'PLL/2', 'InnerLoop/7',        'autorouting','on');
add_line(model_name,'PLL/2', 'FilterModel/5',      'autorouting','on');

%% 3. ParkTransform -> Demux_Vdq
add_line(model_name,'ParkTransform/1', 'Demux_Vdq/1', 'autorouting','on');

%% 4. Demux_Vdq(vd) -> OuterLoop(vd), InnerLoop(vgd), FilterModel(vgd)
%%    Demux_Vdq(vq) -> InnerLoop(vgq), FilterModel(vgq)
add_line(model_name,'Demux_Vdq/1', 'OuterLoop/3',    'autorouting','on');
add_line(model_name,'Demux_Vdq/1', 'InnerLoop/5',    'autorouting','on');
add_line(model_name,'Demux_Vdq/2', 'InnerLoop/6',    'autorouting','on');
add_line(model_name,'Demux_Vdq/1', 'FilterModel/3',  'autorouting','on');
add_line(model_name,'Demux_Vdq/2', 'FilterModel/4',  'autorouting','on');
add_line(model_name,'Demux_Vdq/1', 'PowerCalc/1',    'autorouting','on');
add_line(model_name,'Demux_Vdq/2', 'PowerCalc/2',    'autorouting','on');

%% 5. Steps P*, Q* -> OuterLoop, DroopControl, Scopes
add_line(model_name,'Step_Pref/1', 'OuterLoop/1',    'autorouting','on');
add_line(model_name,'Step_Qref/1', 'OuterLoop/2',    'autorouting','on');
add_line(model_name,'Step_Pref/1', 'DroopControl/3', 'autorouting','on');
add_line(model_name,'Step_Qref/1', 'DroopControl/4', 'autorouting','on');
add_line(model_name,'Step_Pref/1', 'Scope_PowerRefs/1', 'autorouting','on');
add_line(model_name,'Step_Qref/1', 'Scope_PowerRefs/2', 'autorouting','on');

%% 6. OuterLoop(id*, iq*) -> InnerLoop
add_line(model_name,'OuterLoop/1', 'InnerLoop/1', 'autorouting','on');
add_line(model_name,'OuterLoop/2', 'InnerLoop/2', 'autorouting','on');

%% 7. InnerLoop(vd*, vq*) -> FilterModel, Mux_vdq_ref
add_line(model_name,'InnerLoop/1', 'FilterModel/1',  'autorouting','on');
add_line(model_name,'InnerLoop/2', 'FilterModel/2',  'autorouting','on');
add_line(model_name,'InnerLoop/1', 'Mux_vdq_ref/1',  'autorouting','on');
add_line(model_name,'InnerLoop/2', 'Mux_vdq_ref/2',  'autorouting','on');

%% 8. FilterModel(id, iq) -> InnerLoop(feedback), PowerCalc
add_line(model_name,'FilterModel/1', 'InnerLoop/3',  'autorouting','on');
add_line(model_name,'FilterModel/2', 'InnerLoop/4',  'autorouting','on');
add_line(model_name,'FilterModel/1', 'PowerCalc/3',  'autorouting','on');
add_line(model_name,'FilterModel/2', 'PowerCalc/4',  'autorouting','on');

%% 9. InvParkTransform: Mux_vdq_ref -> Xabc_ref
add_line(model_name,'Mux_vdq_ref/1', 'InvParkTransform/1', 'autorouting','on');

%% 10. PowerCalc(P, Q) -> DroopControl
add_line(model_name,'PowerCalc/1', 'DroopControl/1', 'autorouting','on');
add_line(model_name,'PowerCalc/2', 'DroopControl/2', 'autorouting','on');

%% 11. Conexiones a Scopes
add_line(model_name,'PLL/1',             'Scope_PLL/1',          'autorouting','on'); % theta
add_line(model_name,'PLL/2',             'Scope_PLL/2',          'autorouting','on'); % omega
add_line(model_name,'OuterLoop/1',       'Scope_CurrentsDQ/1',   'autorouting','on'); % id*
add_line(model_name,'OuterLoop/2',       'Scope_CurrentsDQ/2',   'autorouting','on'); % iq*
add_line(model_name,'FilterModel/1',     'Scope_CurrentsDQ/3',   'autorouting','on'); % id
add_line(model_name,'FilterModel/2',     'Scope_CurrentsDQ/4',   'autorouting','on'); % iq
add_line(model_name,'Demux_Vdq/1',       'Scope_VoltagesDQ/1',   'autorouting','on'); % vd
add_line(model_name,'Demux_Vdq/2',       'Scope_VoltagesDQ/2',   'autorouting','on'); % vq
add_line(model_name,'PowerCalc/1',       'Scope_Power/1',        'autorouting','on'); % P
add_line(model_name,'PowerCalc/2',       'Scope_Power/2',        'autorouting','on'); % Q
add_line(model_name,'InvParkTransform/1','Scope_Vref_abc/1',     'autorouting','on'); % Vabc_ref
add_line(model_name,'DroopControl/1',    'Scope_Droop/1',        'autorouting','on'); % omega_ref
add_line(model_name,'DroopControl/2',    'Scope_Droop/2',        'autorouting','on'); % V_ref
fprintf('  [OK] Todas las conexiones completadas\n');

%% ========================================================================
%% CONFIGURAR CODIGO DE LAS MATLAB FUNCTION BLOCKS
%% ========================================================================
%% Nota: si la configuracion via sfroot() no funciono, los codigos de las
%% MATLAB Function blocks deben editarse manualmente en Simulink.
%% Abra cada bloque MATLAB Function y copie el codigo correspondiente.

fprintf('\nConfigurando MATLAB Function blocks...\n');

%% PLL/calc_vq: calcula vq para el SRF-PLL
pll_vq_script = sprintf([...
    'function vq = fcn(Vabc, theta)\n'...
    '%% SRF-PLL: calcula componente vq en marco sincrono\n'...
    '%% vq = -2/3*(Va*sin(t) + Vb*sin(t-2pi/3) + Vc*sin(t+2pi/3))\n'...
    'Va = Vabc(1); Vb = Vabc(2); Vc = Vabc(3);\n'...
    'vq = -2/3*(Va*sin(theta) + ...\n'...
    '           Vb*sin(theta - 2*pi/3) + ...\n'...
    '           Vc*sin(theta + 2*pi/3));\n'...
    'end\n']);

%% ParkTransform/Park_abc2dq
park_script = sprintf([...
    'function Xdq = fcn(Xabc, theta)\n'...
    '%% Transformacion de Park: abc -> dq\n'...
    '%% Xd =  2/3*(Xa*cos(t) + Xb*cos(t-2pi/3) + Xc*cos(t+2pi/3))\n'...
    '%% Xq = -2/3*(Xa*sin(t) + Xb*sin(t-2pi/3) + Xc*sin(t+2pi/3))\n'...
    'Xa = Xabc(1); Xb = Xabc(2); Xc = Xabc(3);\n'...
    'Xd =  2/3*(Xa*cos(theta) + ...\n'...
    '           Xb*cos(theta-2*pi/3) + ...\n'...
    '           Xc*cos(theta+2*pi/3));\n'...
    'Xq = -2/3*(Xa*sin(theta) + ...\n'...
    '           Xb*sin(theta-2*pi/3) + ...\n'...
    '           Xc*sin(theta+2*pi/3));\n'...
    'Xdq = [Xd; Xq];\n'...
    'end\n']);

%% InvParkTransform/InvPark_dq2abc
invpark_script = sprintf([...
    'function Xabc = fcn(Xdq, theta)\n'...
    '%% Transformacion de Park Inversa: dq -> abc\n'...
    'Xd = Xdq(1); Xq = Xdq(2);\n'...
    'Xa = Xd*cos(theta)        - Xq*sin(theta);\n'...
    'Xb = Xd*cos(theta-2*pi/3) - Xq*sin(theta-2*pi/3);\n'...
    'Xc = Xd*cos(theta+2*pi/3) - Xq*sin(theta+2*pi/3);\n'...
    'Xabc = [Xa; Xb; Xc];\n'...
    'end\n']);

%% Intentar configurar via Stateflow API
try
    rt  = sfroot();
    mdl = rt.find('-isa','Simulink.BlockDiagram','Name', model_name);

    charts = mdl.find('-isa','Stateflow.EMChart');
    for k = 1:length(charts)
        cpath = charts(k).Path;
        if contains(cpath, 'PLL/calc_vq')
            charts(k).Script = pll_vq_script;
            fprintf('  [OK] PLL/calc_vq configurado\n');
        elseif contains(cpath, 'ParkTransform/Park_abc2dq')
            charts(k).Script = park_script;
            fprintf('  [OK] ParkTransform/Park_abc2dq configurado\n');
        elseif contains(cpath, 'InvParkTransform/InvPark_dq2abc')
            charts(k).Script = invpark_script;
            fprintf('  [OK] InvParkTransform/InvPark_dq2abc configurado\n');
        end
    end
catch ME
    fprintf('  [AVISO] No se pudo configurar MATLAB Functions via API: %s\n', ME.message);
    fprintf('          Configure manualmente copiando los codigos del README.\n');
end

%% ========================================================================
%% GUARDAR MODELO
%% ========================================================================
save_system(model_name);

fprintf('\n');
fprintf('========================================================\n');
fprintf(' MODELO VSC GRID-FOLLOWING CREADO EXITOSAMENTE\n');
fprintf('========================================================\n');
fprintf(' Archivo : %s.slx\n', model_name);
fprintf(' Ruta    : %s\n', pwd);
fprintf('\n SUBSISTEMAS:\n');
fprintf('   GridSource        Fuente 3ph %gHz (Vp=%.1fV)\n', f_grid, V_grid_pk);
fprintf('   PLL               SRF-PLL (Kp=%g, Ki=%g)\n', Kp_pll, Ki_pll);
fprintf('   ParkTransform     abc->dq (MATLAB Function)\n');
fprintf('   InvParkTransform  dq->abc (MATLAB Function)\n');
fprintf('   OuterLoop         P*,Q* -> id*,iq* (lazo potencia)\n');
fprintf('   InnerLoop         PI corriente id,iq (Kp=%g Ki=%g)\n', Kp_id, Ki_id);
fprintf('   FilterModel       RL dq (L=%gmH, R=%g Ohm)\n', L_filter*1e3, R_filter);
fprintf('   PowerCalc         P=1.5(vd*id+vq*iq), Q\n');
fprintf('   DroopControl      P-omega (kp=%g), Q-V (kq=%g)\n', kp_droop, kq_droop);
fprintf('\n PERTURBACIONES:\n');
fprintf('   t=%.2fs: P* %g -> %g W\n', t_step_P, P_ref, P_ref_step);
fprintf('   t=%.2fs: Q* %g -> %g VAR\n', t_step_Q, Q_ref, Q_ref_step);
fprintf('\n SCOPES (8 para analisis del informe):\n');
fprintf('   Scope_PLL         theta [rad], omega [rad/s]\n');
fprintf('   Scope_CurrentsDQ  id*,iq*,id,iq [A]\n');
fprintf('   Scope_VoltagesDQ  vd,vq [V]\n');
fprintf('   Scope_Power       P [W], Q [VAR]\n');
fprintf('   Scope_Vabc_Grid   Voltajes 3ph de la red\n');
fprintf('   Scope_Vref_abc    Voltajes ref convertidor 3ph\n');
fprintf('   Scope_PowerRefs   P*,Q* con escalones\n');
fprintf('   Scope_Droop       omega_ref,V_ref del droop\n');
fprintf('\n Stop time=0.3s, Solver=ode45\n');
fprintf('========================================================\n');

%% Abrir el modelo para visualizacion inmediata
open_system(model_name);
