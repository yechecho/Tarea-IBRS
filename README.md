# Tarea-IBRS — Modelamiento en MATLAB

Tarea de Modelamiento en MATLAB para el curso de Ingeniería Eléctrica.

---

## Problema

Se tiene un **circuito RLC serie** con los siguientes valores de parámetros:

| Componente | Símbolo | Valor |
|------------|---------|-------|
| Resistencia | R | 10 Ω |
| Inductancia | L | 0.1 H |
| Capacitancia | C | 0.001 F |

La entrada del circuito es el voltaje de la fuente `vi(t)` y la salida es el voltaje en el capacitor `Vc(t)`.

Se pide realizar:

1. Obtener la **función de transferencia** `H(s) = Vc(s)/Vi(s)`
2. Determinar **polos, ceros y ganancia** del sistema
3. Calcular la **frecuencia natural** `ωn` y el **factor de amortiguamiento** `ζ`
4. Determinar el **tipo de respuesta** (subamortiguada, críticamente amortiguada o sobreamortiguada)
5. Verificar la **estabilidad** del sistema
6. Obtener la **representación en espacio de estados**
7. Graficar la **respuesta al escalón unitario**
8. Graficar el **diagrama de Bode**
9. Graficar el **mapa de polos y ceros**
10. Analizar la **sensibilidad** de la respuesta ante variaciones en la resistencia R

---

## Solución Teórica

### Función de Transferencia

Aplicando la Ley de Kirchhoff de voltajes (KVL) en el circuito RLC serie:

$$V_i(s) = \left(R + Ls + \frac{1}{Cs}\right) I(s)$$

$$V_c(s) = \frac{1}{Cs} I(s)$$

Por tanto:

$$H(s) = \frac{V_c(s)}{V_i(s)} = \frac{\frac{1}{LC}}{s^2 + \frac{R}{L}s + \frac{1}{LC}}$$

Con los valores dados:

$$H(s) = \frac{10000}{s^2 + 100s + 10000}$$

### Parámetros del Sistema de Segundo Orden

- **Frecuencia natural:** $\omega_n = \sqrt{\frac{1}{LC}} = 100\ \text{rad/s}$
- **Factor de amortiguamiento:** $\zeta = \frac{R}{2}\sqrt{\frac{C}{L}} = 0.5$
- **Tipo de respuesta:** Subamortiguada ($0 < \zeta < 1$)

### Representación en Espacio de Estados

Definiendo los estados $x_1 = V_c$ y $x_2 = \dot{V}_c$:

$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ -\frac{1}{LC} & -\frac{R}{L} \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ \frac{1}{LC} \end{bmatrix} u$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \mathbf{x}$$

---

## Archivos MATLAB

| Archivo | Descripción |
|---------|-------------|
| `tarea_modelamiento.m` | Script principal: FT, polos/ceros, espacio de estados, respuesta al escalón, Bode, mapa de polos y ceros |
| `analisis_sensibilidad.m` | Análisis de sensibilidad ante variación de R (distintos valores de ζ), respuesta senoidal e impulsiva |

---

## Resultados Esperados

- El sistema es **estable** (ambos polos en el semiplano izquierdo)
- La respuesta al escalón presenta un **sobrepico** de aproximadamente 16.3% (ζ = 0.5)
- El sistema alcanza el valor estacionario de 1 V (ganancia DC unitaria)
- El diagrama de Bode muestra una resonancia a ωn = 100 rad/s

---

## Cómo Ejecutar

1. Abrir MATLAB (versión R2018a o superior recomendada)
2. Navegar a la carpeta del proyecto
3. Ejecutar el script principal:
   ```matlab
   run('tarea_modelamiento.m')
   ```
4. Para el análisis adicional:
   ```matlab
   run('analisis_sensibilidad.m')
   ```
