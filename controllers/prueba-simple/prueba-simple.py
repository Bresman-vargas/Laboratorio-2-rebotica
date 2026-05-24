from controller import Robot, DistanceSensor, Motor
import csv
import math

# =============================================================
# INICIALIZACIÓN DEL ROBOT
# =============================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Frecuencia de muestreo (reportar en README)
Ts = timestep / 1000.0   # En segundos (e.g., 64ms → 0.064 s)
fs = 1.0 / Ts            # Hz

print(f"[INFO] Ts = {Ts:.4f} s | fs = {fs:.2f} Hz")

# =============================================================
# 1. CONFIGURAR SENSORES DE DISTANCIA
# =============================================================
sensores_requeridos = ['ps7', 'ps0', 'ps2', 'ps5']
sensores = {}
for nombre in sensores_requeridos:
    sensores[nombre] = robot.getDevice(nombre)
    sensores[nombre].enable(timestep)

# =============================================================
# 2. CONFIGURAR MOTORES
# =============================================================
motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')
motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

# =============================================================
# 3. CONFIGURAR ENCODERS
# =============================================================
encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(timestep)
encoder_derecho.enable(timestep)

# =============================================================
# PARÁMETROS GLOBALES
# =============================================================
RADIO_RUEDA   = 0.0205    # metros (e-puck)
VELOCIDAD_BASE = 4.0
UMBRAL_FRONTAL_METROS = 0.16
UMBRAL_FRONTAL_CRUDO  = 100.0
UMBRAL_LATERAL_CRUDO  = 80.0
DURACION_GIRO_MINIMO  = 25  # ciclos de giro obligatorio

# =============================================================
# VARIABLES DE ESTADO - NAVEGACIÓN
# =============================================================
estoy_bloqueado      = False
direccion_giro       = "IZQUIERDA"
vel_izquierda        = 0.0
vel_derecha          = 0.0
pasos_giro_restantes = 0

# =============================================================
# VARIABLES - ENCODERS
# =============================================================
pos_anterior_izq = 0.0
pos_anterior_der = 0.0

# =============================================================
# FILTRO DE KALMAN - PARÁMETROS
# =============================================================
distancia_estimada = 0.5   # Estado inicial (metros)
P = 1.0                    # Covarianza inicial
R = 0.05                   # Varianza del sensor (ruido de medición)
Q = 0.02                   # Varianza del proceso (ruido del modelo)

# =============================================================
# FILTRO SIMPLE (MEDIA MÓVIL) - PARÁMETROS
# =============================================================
VENTANA_FILTRO = 5         # Cantidad de muestras para la media móvil
buffer_frontal = []        # Buffer circular para el filtro simple
distancia_filtrada_simple = 3.0  # Valor inicial

# =============================================================
# ALMACENAMIENTO DE SEÑALES (para análisis y graficación)
# =============================================================
registro = []   # Lista de dicts con una fila por ciclo
paso_actual = 0

# =============================================================
# FUNCIÓN: convertir lectura cruda del sensor a metros
# =============================================================
def sensor_a_metros(lectura_cruda):
    """Convierte la lectura cruda del sensor de proximidad a distancia en metros."""
    if lectura_cruda > 50.0:
        return 250.0 / lectura_cruda
    return 3.0  # Sin obstáculo detectado → distancia máxima nominal

# =============================================================
# FUNCIÓN: filtro de media móvil
# =============================================================
def media_movil(buffer, nuevo_valor, ventana):
    """Actualiza el buffer y devuelve la media de los últimos 'ventana' valores."""
    buffer.append(nuevo_valor)
    if len(buffer) > ventana:
        buffer.pop(0)
    return sum(buffer) / len(buffer)

# =============================================================
# BUCLE PRINCIPAL
# =============================================================
while robot.step(timestep) != -1:
    paso_actual += 1
    tiempo_actual = paso_actual * Ts  # Tiempo simulado en segundos

    # ---------------------------------------------------------
    # LECTURA CRUDA DE SENSORES
    # ---------------------------------------------------------
    valor_ps7 = sensores['ps7'].getValue()  # Frontal izquierdo
    valor_ps0 = sensores['ps0'].getValue()  # Frontal derecho
    valor_ps2 = sensores['ps2'].getValue()  # Lateral derecho
    valor_ps5 = sensores['ps5'].getValue()  # Lateral izquierdo

    lectura_frontal_cruda = max(valor_ps7, valor_ps0)

    # ---------------------------------------------------------
    # FILTRO SIMPLE (MEDIA MÓVIL) sobre sensores frontales
    # ---------------------------------------------------------
    medicion_metros_cruda = sensor_a_metros(lectura_frontal_cruda)
    distancia_filtrada_simple = media_movil(
        buffer_frontal,
        medicion_metros_cruda,
        VENTANA_FILTRO
    )

    # ---------------------------------------------------------
    # ETAPA 1 - KALMAN: PREDICCIÓN MEDIANTE ENCODERS
    # ---------------------------------------------------------
    pos_actual_izq = encoder_izquierdo.getValue()  # radianes
    pos_actual_der = encoder_derecho.getValue()    # radianes

    delta_izq = pos_actual_izq - pos_anterior_izq  # Δθ izquierda
    delta_der = pos_actual_der - pos_anterior_der  # Δθ derecha

    distancia_izq = RADIO_RUEDA * delta_izq        # s = r·θ
    distancia_der = RADIO_RUEDA * delta_der

    # Avance lineal: 0 si el robot está girando en el lugar
    if (vel_izquierda > 0 and vel_derecha < 0) or (vel_izquierda < 0 and vel_derecha > 0):
        avance_robot = 0.0
    else:
        avance_robot = (distancia_izq + distancia_der) / 2.0

    pos_anterior_izq = pos_actual_izq
    pos_anterior_der = pos_actual_der

    # Predicción de la distancia frontal
    distancia_predicha = distancia_estimada - avance_robot
    P_predicha = P + Q                              # Covarianza de predicción

    # ---------------------------------------------------------
    # ETAPA 2 - KALMAN: CORRECCIÓN MEDIANTE SENSORES FRONTALES
    # ---------------------------------------------------------
    medicion_sensor_metros = sensor_a_metros(lectura_frontal_cruda)

    K = P_predicha / (P_predicha + R)              # Ganancia de Kalman
    distancia_estimada = distancia_predicha + K * (medicion_sensor_metros - distancia_predicha)
    P = (1.0 - K) * P_predicha                     # Covarianza actualizada

    # ---------------------------------------------------------
    # ETAPA 3 - NAVEGACIÓN REACTIVA
    # ---------------------------------------------------------

    # Decrementar giro obligatorio
    if pasos_giro_restantes > 0:
        pasos_giro_restantes -= 1

    # Salir del estado bloqueado solo si el giro terminó Y la vía está libre
    if estoy_bloqueado and pasos_giro_restantes == 0 and lectura_frontal_cruda < UMBRAL_FRONTAL_CRUDO:
        estoy_bloqueado = False
        distancia_estimada = 0.5  # Reiniciar estimación

    # CASO 1: Obstáculo al frente (giro activo o detección nueva)
    if pasos_giro_restantes > 0 or distancia_estimada < UMBRAL_FRONTAL_METROS or lectura_frontal_cruda > UMBRAL_FRONTAL_CRUDO:

        if not estoy_bloqueado:
            estoy_bloqueado = True
            pasos_giro_restantes = DURACION_GIRO_MINIMO

            # Elegir dirección hacia el lado más libre
            if valor_ps5 > valor_ps2:
                direccion_giro = "DERECHA"
            else:
                direccion_giro = "IZQUIERDA"

        if direccion_giro == "DERECHA":
            vel_izquierda =  VELOCIDAD_BASE
            vel_derecha   = -VELOCIDAD_BASE
        else:
            vel_izquierda = -VELOCIDAD_BASE
            vel_derecha   =  VELOCIDAD_BASE

    # CASO 2: Obstáculo lateral izquierdo → curvar a la derecha
    elif valor_ps5 > UMBRAL_LATERAL_CRUDO:
        estoy_bloqueado = False
        vel_izquierda = VELOCIDAD_BASE
        vel_derecha   = VELOCIDAD_BASE * 0.3

    # CASO 3: Obstáculo lateral derecho → curvar a la izquierda
    elif valor_ps2 > UMBRAL_LATERAL_CRUDO:
        estoy_bloqueado = False
        vel_izquierda = VELOCIDAD_BASE * 0.3
        vel_derecha   = VELOCIDAD_BASE

    # CASO 4: Camino libre → avanzar
    else:
        vel_izquierda = VELOCIDAD_BASE
        vel_derecha   = VELOCIDAD_BASE
        estoy_bloqueado = False

    # Aplicar velocidades
    motor_izquierdo.setVelocity(vel_izquierda)
    motor_derecho.setVelocity(vel_derecha)

    # ---------------------------------------------------------
    # REGISTRO DE SEÑALES (para graficación posterior)
    # ---------------------------------------------------------
    registro.append({
        'paso':                  paso_actual,
        'tiempo_s':              round(tiempo_actual, 4),
        'ps7_crudo':             round(valor_ps7, 4),
        'ps0_crudo':             round(valor_ps0, 4),
        'ps2_lateral':           round(valor_ps2, 4),
        'ps5_lateral':           round(valor_ps5, 4),
        'frontal_crudo':         round(lectura_frontal_cruda, 4),
        'frontal_metros_crudo':  round(medicion_metros_cruda, 4),
        'frontal_filtrado':      round(distancia_filtrada_simple, 4),
        'distancia_kalman':      round(distancia_estimada, 4),
        'avance_robot_m':        round(avance_robot, 6),
        'ganancia_kalman':       round(K, 4),
        'vel_izq':               round(vel_izquierda, 2),
        'vel_der':               round(vel_derecha, 2),
        'estado_bloqueado':      int(estoy_bloqueado),
    })

    # Guardar CSV cada 500 pasos para no perder datos si se detiene la sim
    if paso_actual % 500 == 0:
        with open('señales_robot.csv', 'w', newline='') as f:
            campos = list(registro[0].keys())
            writer = csv.DictWriter(f, fieldnames=campos)
            writer.writeheader()
            writer.writerows(registro)
        print(f"[LOG] Paso {paso_actual} | d_kalman={distancia_estimada:.3f}m | "
              f"d_filtrada={distancia_filtrada_simple:.3f}m | K={K:.3f}")

# =============================================================
# GUARDAR CSV FINAL al terminar la simulación
# =============================================================
if registro:
    with open('señales_robot.csv', 'w', newline='') as f:
        campos = list(registro[0].keys())
        writer = csv.DictWriter(f, fieldnames=campos)
        writer.writeheader()
        writer.writerows(registro)
    print(f"[DONE] {len(registro)} muestras guardadas en 'señales_robot.csv'")
