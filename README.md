# Laboratorio 2 — Navegación Reactiva con Filtrado y Fusión de Sensores en Webots

**Asignatura:** ICI 4150 — Robótica y Sistemas Autónomos 2026-01  
**Integrantes:** [Nombre 1] · [Nombre 2] · [Nombre 3]

---

## Objetivo

Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial, aplicando filtrado sobre las mediciones de sensores de distancia y un filtro de Kalman para estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

---

## Robot y Sensores Utilizados

**Robot:** e-puck (Webots) — diferencial de dos ruedas.

| Sensor | ID Webots | Posición | Uso |
|--------|-----------|----------|-----|
| Proximidad frontal izquierdo | `ps7` | Frontal izq. | Detección de obstáculos al frente |
| Proximidad frontal derecho   | `ps0` | Frontal der. | Detección de obstáculos al frente |
| Proximidad lateral derecho   | `ps2` | Lateral der. | Decisión de dirección de giro |
| Proximidad lateral izquierdo | `ps5` | Lateral izq. | Decisión de dirección de giro |
| Encoder rueda izquierda      | `left wheel sensor`  | Rueda izq. | Estimación de avance |
| Encoder rueda derecha        | `right wheel sensor` | Rueda der. | Estimación de avance |

**Radio de rueda:** r = 0.0205 m

---

## Frecuencia de Muestreo

| Parámetro | Valor |
|-----------|-------|
| Tiempo de muestreo Ts | 0.064 s (64 ms, paso base del e-puck) |
| Frecuencia de muestreo fs | ≈ 15.6 Hz |
| Muestras registradas | [completar tras experimento] |

> Todas las señales registradas (crudas, filtradas y estimadas) fueron adquiridas con esta misma frecuencia.

---

## Estimación del Avance mediante Encoders

Los encoders entregan medidas angulares en radianes. El desplazamiento lineal de cada rueda se calcula con:

```
s = r · θ
```

El avance lineal del robot en cada ciclo es el promedio de ambas ruedas, excepto durante giros en el lugar (velocidades opuestas), en cuyo caso se considera avance nulo.

---

## Filtro Simple Aplicado

Se implementó un **filtro de media móvil** de ventana N = 5 muestras sobre la lectura frontal convertida a metros. Esto suaviza los picos de ruido del sensor antes de pasarla como medición al filtro de Kalman.

```python
def media_movil(buffer, nuevo_valor, ventana):
    buffer.append(nuevo_valor)
    if len(buffer) > ventana:
        buffer.pop(0)
    return sum(buffer) / len(buffer)
```

---

## Filtro de Kalman — Implementación

### Variable estimada
Distancia frontal al obstáculo más cercano: **d̂ₖ** (metros).

### Parámetros

| Parámetro | Símbolo | Valor |
|-----------|---------|-------|
| Estado inicial | d̂₀ | 0.5 m |
| Covarianza inicial | P₀ | 1.0 |
| Varianza del sensor | R | 0.05 |
| Varianza del proceso | Q | 0.02 |

### Etapa de Predicción (encoders)

```
d̂⁻ₖ = d̂ₖ₋₁ − Δdₖ
P⁻ₖ = Pₖ₋₁ + Q
```

donde Δdₖ es el avance estimado del robot en el ciclo k.

### Etapa de Corrección (sensor frontal)

```
Kₖ = P⁻ₖ / (P⁻ₖ + R)
d̂ₖ = d̂⁻ₖ + Kₖ · (zₖ − d̂⁻ₖ)
Pₖ = (1 − Kₖ) · P⁻ₖ
```

donde zₖ es la distancia en metros calculada a partir del sensor frontal.

---

## Lógica de Navegación Reactiva

| Condición | Acción |
|-----------|--------|
| d̂ₖ < 0.16 m **o** sensor frontal crudo > 100 | Girar (mínimo 25 ciclos) |
| Sensor lateral izquierdo > 80 | Curvar suavemente a la derecha |
| Sensor lateral derecho > 80 | Curvar suavemente a la izquierda |
| Vía libre | Avanzar a velocidad base |

**Elección del lado de giro:** si el lateral izquierdo (ps5) detecta más obstáculo que el derecho (ps2), el robot gira a la derecha, y viceversa.

---

## Gráficos de Señales

### Señales crudas de sensores
![Señales crudas](grafico_señales_crudas.png)

### Comparación: crudo vs filtro simple vs Kalman
![Comparación filtros](grafico_comparacion_filtros.png)

### Evolución de la ganancia de Kalman
![Ganancia Kalman](grafico_ganancia_kalman.png)

### Desplazamiento acumulado (encoders)
![Encoders](grafico_encoders.png)

---

## Resultados por Escenario

### Escenario 1 — Entorno simple (pocos obstáculos)
[Describir comportamiento, capturas de pantalla, muestras registradas]

### Escenario 2 — Entorno complejo (pasillos o múltiples obstáculos)
[Describir comportamiento, capturas de pantalla, muestras registradas]

---

## Análisis y Conclusiones

[Completar con observaciones del grupo sobre:]
- Diferencias entre señal cruda, filtrada y estimada con Kalman
- Estabilidad del movimiento con cada enfoque
- Giros innecesarios observados
- Efectividad en la evasión de obstáculos

---

## Instrucciones para Ejecutar la Simulación

1. Abrir Webots y cargar el mundo `[nombre_mundo].wbt`.
2. Asignar `laboratorio2_mejorado.py` como controlador del robot e-puck.
3. Iniciar la simulación.
4. Al finalizar, se genera `señales_robot.csv` en la carpeta del controlador.
5. Ejecutar el script de graficación:
   ```bash
   pip install pandas matplotlib
   python graficar_señales.py
   ```
6. Los gráficos PNG se guardan en la misma carpeta.
