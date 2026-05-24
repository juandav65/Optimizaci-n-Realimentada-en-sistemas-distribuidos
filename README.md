# Optimización Realimentada en Sistemas Distribuidos

**Universidad Nacional de Colombia — Facultad de Ingeniería**  
**Departamento de Ingeniería Mecánica y Mecatrónica**  
**Curso:** Optimización y Control en Sistemas Distribuidos en Red | **Semestre:** 2025-II

---

## Autores

| Nombre | 
|--------|
| Jorge Emilio Melo Guevara |
| Juan David Medina Pérez |

---

## Descripción

Este proyecto es un estudio académico sobre **Optimización Realimentada Distribuida (DFO)** aplicada a la coordinación de enjambres robóticos, basado en el paper de referencia:

> Terpin, A., Fricker, S., Perez, M., de Badyn, M. H., & Dörfler, F. (2022, June). *Distributed Feedback Optimisation for Robotic Coordination*. In 2022 American Control Conference (ACC) (pp. 3710-3715). IEEE.

El objetivo central es demostrar cómo un grupo de robots puede organizarse en **formaciones geométricas bidimensionales** alrededor de puntos objetivo, sin coordinación central, usando únicamente información local entre agentes vecinos.

---

## Conceptos Clave

**Optimización Realimentada (FO):** paradigma de control que combina optimización en línea con regulación realimentada. A diferencia del control por optimización clásico, no requiere modelos extremadamente precisos ni observabilidad completa de todos los estados — se basa en mediciones de salida en tiempo real y sensibilidades de estado estable, resultando en implementaciones ligeras y robustas ante perturbaciones.

**Optimización Realimentada Distribuida (DFO):** extensión del paradigma anterior a sistemas multiagente, donde múltiples agentes cooperan para minimizar un objetivo global intercambiando información únicamente de forma local, sin coordinador central.

---

## Objetivos

- Exponer el método de optimización retroalimentada aplicado sobre sistemas distribuidos.
- Implementar un algoritmo DFO para posicionar grupos de robots en formaciones geométricas deseadas alrededor de puntos objetivo.

---

## Formulación del Problema

### Estructura del sistema

Se considera un enjambre de $N$ agentes en un plano $a$-$b$ con marco de referencia cartesiano global compartido. Las conexiones entre agentes se modelan como un grafo no dirigido $G(\nu, \mathcal{E})$, donde $\nu$ es el conjunto de agentes y $\mathcal{E}$ el conjunto de enlaces entre nodos.

### Estado de cada agente

El estado del nodo $i$ es:

$$x_i = \begin{bmatrix} r_i \\ \theta_i \end{bmatrix}, \quad r_i = \begin{bmatrix} a_i \\ b_i \end{bmatrix}, \quad \theta_i \in (-\pi, \pi]$$

donde $r_i$ es la posición y $\theta_i$ es la orientación respecto al eje $a$.

### Dinámica de la planta (unicycle model)

$$\dot{a}_i = v_i \cos(\theta_i), \qquad \dot{b}_i = v_i \sin(\theta_i), \qquad \dot{\theta}_i = \omega_i$$

### Errores de seguimiento

Dada una referencia de posición $u_i = [u_{ai}, u_{bi}]^T$:

$$\xi_{ai} = u_{ai} - a_i, \quad \xi_{bi} = u_{bi} - b_i, \quad \xi_i = \sqrt{\xi_{ai}^2 + \xi_{bi}^2}$$

$$\phi_i = \text{atan2}(\xi_{bi}, \xi_{ai}) - \theta_i$$

### Low Level Control (LLC)

Ley de control propuesta por los autores del paper guía:

$$v_i = k_i \xi_i \cos(\phi_i)$$

$$\omega_i = k_i (\cos(\phi_i) + 1)\sin(\phi_i)$$

### Función de costo

La función objetivo combina dos términos: mantener la formación y converger al punto objetivo $\tau$:

$$\Phi(r) = \frac{\gamma_1}{2} \sum_{(i,j) \in \mathcal{E}} \|r_i - r_j - d_{ij}\|^2 + \frac{\gamma_2}{2} \sum_{i \in \nu} \|r_i - \tau\|^2$$

El primer término penaliza el incumplimiento de las distancias deseadas $d_{ij}$ entre agentes vecinos; el segundo atrae el enjambre hacia $\tau$. Los pesos $\gamma_1$ y $\gamma_2$ permiten priorizar uno u otro comportamiento.

---

## Esquema de Control

El esquema de lazo cerrado se define a partir de:

$$\dot{x} = f(x, u), \qquad y = g(x, u) = r, \qquad \dot{u} = -\epsilon \, \mathbb{J}h(u)^T \nabla\Phi(r)$$

Para el problema planteado, el mapa de entrada-salida en estado estacionario y su sensibilidad son:

$$h(u) = u, \qquad \mathbb{J}h(u) = I_{2N}$$

### Regla de actualización distribuida de cada agente

Cada robot $i$ actualiza su referencia local $u_i$ mediante:

$$\dot{u}_i = -\varepsilon \left[ \gamma_1 \sum_{j \in \mathcal{N}(i)} (r_i - r_j - d_{ij}) + \gamma_2(r_i - \tau) \right]$$

Esta expresión es puramente **local**: el agente $i$ solo necesita su posición, la de sus vecinos $\mathcal{N}(i)$ y el objetivo global $\tau$.

### Parámetros de sintonización utilizados

| Parámetro | Valor | Justificación |
|-----------|-------|---------------|
| $\epsilon$ | $-1$ | Aporta el signo negativo sin escalar la señal |
| $\gamma_1$ | $1$ | Igual peso a la formación y al objetivo |
| $\gamma_2$ | $1$ | Igual peso a la formación y al objetivo |
| $k$ | $0.1$ | Ganancia del LLC para cada componente |

---

## Implementación

La simulación se desarrolló en **MATLAB**, partiendo del código fuente original disponible en el repositorio del paper de referencia:

> [https://github.com/antonioterpin/feedback-optimization-swarm-robotics](https://github.com/antonioterpin/feedback-optimization-swarm-robotics)

Como parte del trabajo de estudio, se elaboró una **versión simplificada y condensada** del código original (`OptimizacionRealimetada_RedesDistribuidas.m`), con el objetivo de facilitar la comprensión del algoritmo. Las principales decisiones de simplificación fueron:

- Se eliminaron las funcionalidades de evasión de obstáculos y saturación de velocidad (parámetros deshabilitados: `params = [0,0,0,0,0]`).
- Todo el sistema se consolida en un **único script de MATLAB** con funciones auxiliares al final del mismo archivo, en lugar de múltiples archivos separados.
- Se añadieron comentarios explícitos que vinculan cada bloque de código con las ecuaciones y secciones del paper de referencia.
- Se simplificó el ciclo de simulación usando integración de Euler con paso fijo `dt = 0.1`.

### Parámetros de simulación

```matlab
simTime  = 100;       % Tiempo de simulación
dt       = 0.1;       % Paso de integración (Euler)
N_agents = 4;         % Número de agentes
epsilon  = 0.1;       % Ganancia del optimizador
gamma    = [1,1,10,10]; % Pesos γ₁, γ₂ de la función de costo
```

### Estructura del script

El script principal se organiza en tres bloques:

**1. Configuración de la formación**
- **Matriz B** (incidencia): codifica la topología del grafo de la formación.
- **Matriz A** (adyacencia): indica qué agentes están conectados.
- **Vector d**: define las posiciones relativas objetivo entre agentes conectados.

**2. Configuración del objetivo e inicialización**
- Se define el punto objetivo $\tau = [3,\ 3]$.
- Se establecen condiciones iniciales aleatorias cerca del origen: $x, y \in [-0.1, 0.1]$, $\theta \in [-\pi/2, \pi/2]$.

**3. Ciclo de simulación** — implementa el diagrama de control con tres funciones internas:

| Función | Rol |
|---------|-----|
| `ctrl_implemented` | Calcula $\dot{u} = -\varepsilon \nabla\Phi(r)$ — ley de control distribuida (ec. 7 del paper) |
| `dyn_implemented_velocity` | Simula la dinámica unicycle con el LLC (ecs. 1, 2, 3 del paper) |
| `out_implemented` | Mapa de salida $y = g(x,u) = r$ — extrae posiciones del estado completo |

Adicionalmente se incluyen funciones de visualización:

| Función | Descripción |
|---------|-------------|
| `plotFormation` | Grafica las trayectorias de los agentes con codificación temporal por color |
| `formation_plot` | Dibuja el grafo de la formación en un instante de tiempo específico |
| `compareTuning` | Grafica la evolución de los costos de formación y de objetivo |

---

## Caso Simulado: Formación Cuadrada

**Punto objetivo:** $\tau = [1.5,\ 1.5]$

**Matrices de configuración:**

$$\mathbf{B} = \begin{bmatrix} 1 & 0 & 0 & -1 \\ -1 & 1 & 0 & 0 \\ 0 & -1 & 1 & 0 \\ 0 & 0 & -1 & 1 \end{bmatrix}, \qquad \mathbf{A} = \begin{bmatrix} 0 & 1 & 0 & 1 \\ 1 & 0 & 1 & 0 \\ 0 & 1 & 0 & 1 \\ 1 & 0 & 1 & 0 \end{bmatrix}$$

$$\mathbf{d} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ 1 \\ -1 \\ 0 \\ 0 \\ -1 \end{bmatrix}$$

### Resultados

**Formación en el tiempo:** los cuatro robots parten de condiciones iniciales aleatorias cerca del origen y convergen progresivamente hacia la configuración cuadrada, desplazándose coordinadamente hacia $\tau$.

**Distancia al objetivo:** se observa una disminución monótona que se estabiliza por debajo de 0.5 unidades. La distancia residual refleja el balance inherente entre los términos $\gamma_1$ (formación) y $\gamma_2$ (objetivo) en la función de costo.

**Costo de objetivo normalizado** $\Phi_\tau(r)/\gamma_2$: disminuye de forma acelerada, confirmando la convergencia del enjambre hacia la vecindad de $\tau$.

**Costo de formación** $\Phi_d(r) = \frac{\gamma_1}{2}\|\mathbf{B}_2^T r - d\|^2$: disminución rápida y monótona, evidenciando que los agentes adoptan eficientemente las separaciones relativas deseadas.

---

## Conclusiones

- El esquema DFO permite coordinar un enjambre de robots en formaciones geométricas sin controlador central, usando únicamente información local entre vecinos.
- La convergencia asintótica hacia la formación y el objetivo fue validada numéricamente, consistente con los resultados teóricos del paper de referencia.
- La distancia residual al objetivo es una consecuencia matemática del balance entre $\gamma_1$ y $\gamma_2$, y puede ajustarse según la prioridad de la aplicación.
- El enfoque es **escalable** (se puede agregar más agentes extendiendo $B$, $A$ y $d$) y **robusto** (no requiere modelo exacto de la planta en el esquema FO puro).

---

## Referencias

1. Terpin, A., Fricker, S., Perez, M., de Badyn, M. H., & Dörfler, F. (2022, June). *Distributed Feedback Optimisation for Robotic Coordination*. In 2022 American Control Conference (ACC) (pp. 3710-3715). IEEE.

2. Ortmann, L., Hauswirth, A., Caduff, I., Dörfler, F., & Bolognani, S. (2020). *Experimental validation of feedback optimization in power distribution grids*. Electric Power Systems Research, 189, 106782.
