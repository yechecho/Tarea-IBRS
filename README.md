# VSC Grid-Following Control - Tarea de Simulación II

## Descripción

Este repositorio contiene los archivos de simulación para la **Tarea de Simulación II: Control Vectorial de un Convertidor VSC en Modo Grid-Following** del curso *Modelizado de Sistemas Eléctricos de Potencia*.

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `VSC_Grid_Connected.slx` | Modelo Simulink original (VSC Simscape con filtro L, red 25kV/60Hz) |
| `VSCm_Grid_Connected.m` | **Parámetros del sistema** (PLL, control corriente, droop, referencias) |
| `create_VSC_GridFollowing_Complete.m` | **Script que crea el modelo completo** de control vectorial grid-following |
| `Tarea2_IBRs.pdf` | Enunciado de la tarea |

---

## Cómo Usar

### Opción A: Modelo Original (VSC_Grid_Connected.slx)
1. Abra MATLAB y navegue a la carpeta del repositorio
2. Ejecute `VSCm_Grid_Connected` para cargar parámetros
3. Abra `VSC_Grid_Connected.slx` en Simulink
4. Simule con Ctrl+T

### Opción B: Modelo Completo de Control Vectorial (Recomendado para la Tarea)
1. Abra MATLAB y navegue a la carpeta del repositorio
2. Ejecute el script de creación:
   ```matlab
   create_VSC_GridFollowing_Complete
   ```
3. El modelo `VSC_GridFollowing_Control.slx` se creará y abrirá automáticamente
4. Simule con Ctrl+T (tiempo de simulación: 0.3 segundos)

---

## Estructura del Modelo Completo (VSC_GridFollowing_Control)

```
VSC_GridFollowing_Control
├── GridSource          → Fuente trifásica 60 Hz (Va, Vb, Vc)
├── PLL                 → Phase Locked Loop (explicito con PI visible)
│   ├── calc_vq         → Calcula vq para SRF-PLL
│   ├── Kp_PLL          → Ganancia proporcional (Kp_pll)
│   ├── Ki_PLL + Int    → Ganancia integral (Ki_pll)
│   ├── Sum_omega       → ω = ω₀ + Kp·vq + Ki·∫vq dt
│   └── Int_theta       → θ = ∫ω dt (mod 2π)
├── ParkTransform       → Transformación abc→dq (Park)
├── InvParkTransform    → Transformación dq→abc (Park inversa)
├── OuterLoop           → Lazo externo (Potencia → Corriente)
│   ├── id* = P* / (1.5·Vd)
│   └── iq* = -Q* / (1.5·Vd)
├── InnerLoop           → Lazo interno (Control de corriente PI)
│   ├── PI eje d: vd* = Kp_id·(id*-id) + Ki_id·∫(id*-id)dt - ωL·iq + vgd
│   └── PI eje q: vq* = Kp_iq·(iq*-iq) + Ki_iq·∫(iq*-iq)dt + ωL·id + vgq
├── FilterModel         → Modelo filtro RL en marco dq
│   ├── L·did/dt = vd* - vgd - R·id + ωL·iq
│   └── L·diq/dt = vq* - vgq - R·iq - ωL·id
├── PowerCalc           → P = 1.5(vd·id + vq·iq), Q = 1.5(vd·iq - vq·id)
├── DroopControl        → Control droop primario
│   ├── P-ω: ωref = ω₀ - kp_droop·(P - P*)
│   └── Q-V: Vref = V₀ - kq_droop·(Q - Q*)
├── Step_Pref           → Escalón en P* (t=0.05s: 5000W → 8000W)
├── Step_Qref           → Escalón en Q* (t=0.10s: 0 → 2000 VAR)
└── Scopes (8 total)
    ├── Scope_PLL         → θ(t), ω(t)
    ├── Scope_CurrentsDQ  → id*, iq*, id, iq
    ├── Scope_VoltagesDQ  → vd, vq
    ├── Scope_Power       → P(t), Q(t)
    ├── Scope_Vabc_Grid   → Voltajes trifásicos de la red
    ├── Scope_Vref_abc    → Voltajes de referencia del convertidor
    ├── Scope_PowerRefs   → P*(t), Q*(t) con escalones
    └── Scope_Droop       → ωref(t), Vref(t) del droop
```

---

## Parámetros del Sistema (VSCm_Grid_Connected.m)

### Red Eléctrica
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `f_grid` | 60 Hz | Frecuencia de la red |
| `omega_grid` | 2π·60 rad/s | Frecuencia angular nominal |
| `V_grid` | 220 V | Voltaje de línea RMS |
| `V_dc` | 400 V | Voltaje del bus DC |

### Filtro de Acoplamiento (Tipo L)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `L_filter` | 5 mH | Inductancia del filtro |
| `R_filter` | 0.05 Ω | Resistencia del filtro |

### PLL (Phase Locked Loop)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `Kp_pll` | 150 | Ganancia proporcional |
| `Ki_pll` | 150 | Ganancia integral |

### Lazo Interno de Corriente (Inner Loop)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `Kp_id` | 10 | Proporcional eje d |
| `Ki_id` | 100 | Integral eje d |
| `Kp_iq` | 10 | Proporcional eje q |
| `Ki_iq` | 100 | Integral eje q |

### Control Droop Primario
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `kp_droop` | 0.005 | Ganancia droop P-ω [rad/s/W] |
| `kq_droop` | 0.005 | Ganancia droop Q-V [V/VAR] |

### Referencias de Potencia
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `P_ref` | 5000 W | Potencia activa inicial |
| `Q_ref` | 0 VAR | Potencia reactiva inicial |
| `P_ref_step` | 8000 W | Potencia activa después del escalón (t=0.05s) |
| `Q_ref_step` | 2000 VAR | Potencia reactiva después del escalón (t=0.10s) |

---

## Código de los Bloques MATLAB Function

Si los bloques MATLAB Function no se configuraron automáticamente, copie el siguiente código manualmente:

### PLL/calc_vq
```matlab
function vq = fcn(Vabc, theta)
% SRF-PLL: calcula componente vq en marco síncrono
% vq = -2/3*(Va*sin(t) + Vb*sin(t-2pi/3) + Vc*sin(t+2pi/3))
Va = Vabc(1); Vb = Vabc(2); Vc = Vabc(3);
vq = -2/3*(Va*sin(theta) + ...
           Vb*sin(theta - 2*pi/3) + ...
           Vc*sin(theta + 2*pi/3));
end
```

### ParkTransform/Park_abc2dq
```matlab
function Xdq = fcn(Xabc, theta)
% Transformación de Park: abc -> dq
% Xd =  2/3*(Xa*cos(t) + Xb*cos(t-2pi/3) + Xc*cos(t+2pi/3))
% Xq = -2/3*(Xa*sin(t) + Xb*sin(t-2pi/3) + Xc*sin(t+2pi/3))
Xa = Xabc(1); Xb = Xabc(2); Xc = Xabc(3);
Xd =  2/3*(Xa*cos(theta) + ...
           Xb*cos(theta-2*pi/3) + ...
           Xc*cos(theta+2*pi/3));
Xq = -2/3*(Xa*sin(theta) + ...
           Xb*sin(theta-2*pi/3) + ...
           Xc*sin(theta+2*pi/3));
Xdq = [Xd; Xq];
end
```

### InvParkTransform/InvPark_dq2abc
```matlab
function Xabc = fcn(Xdq, theta)
% Transformación de Park Inversa: dq -> abc
Xd = Xdq(1); Xq = Xdq(2);
Xa = Xd*cos(theta)        - Xq*sin(theta);
Xb = Xd*cos(theta-2*pi/3) - Xq*sin(theta-2*pi/3);
Xc = Xd*cos(theta+2*pi/3) - Xq*sin(theta+2*pi/3);
Xabc = [Xa; Xb; Xc];
end
```

---

## Variables Clave de Análisis

Según el enunciado, el informe debe analizar:

| Variable | Scope | Descripción |
|----------|-------|-------------|
| θ (theta) | `Scope_PLL` (ch.1) | Ángulo estimado por el PLL |
| ω (omega) | `Scope_PLL` (ch.2) | Frecuencia estimada por el PLL |
| id, iq | `Scope_CurrentsDQ` (ch.3,4) | Corrientes en marco dq |
| id*, iq* | `Scope_CurrentsDQ` (ch.1,2) | Referencias de corriente |
| vd, vq | `Scope_VoltagesDQ` | Voltajes de red en marco dq |
| P | `Scope_Power` (ch.1) | Potencia activa |
| Q | `Scope_Power` (ch.2) | Potencia reactiva |

**Nota:** En estado estable con PLL sincronizado: vq ≈ 0. Esto confirma que el eje d está alineado con el vector de tensión de la red.

---

## Interpretación Física

1. **PLL**: Sincroniza el marco dq con la red ajustando ω hasta que vq → 0
2. **id controla P**: Cuando vq ≈ 0, P ≈ 1.5·vd·id
3. **iq controla Q**: Cuando vq ≈ 0, Q ≈ -1.5·vd·iq
4. **Droop P-ω**: Permite regulación primaria de frecuencia compartida entre convertidores
5. **Droop Q-V**: Permite regulación de voltaje en el punto de conexión

---

## Configuración de Simulación

- **Solver**: ode45 (paso variable)
- **Tolerancia relativa**: 1e-4
- **Tiempo de simulación**: 0.3 segundos
- **Perturbaciones**: Escalón P* en t=0.05s, escalón Q* en t=0.10s
