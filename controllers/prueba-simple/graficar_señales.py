"""
graficar_señales.py
Ejecutar DESPUÉS de la simulación para generar los gráficos del README.
Requiere: señales_robot.csv (generado por el controlador)
Instalar dependencias: pip install pandas matplotlib
"""

import csv
import os

# ── Intentar importar dependencias ────────────────────────────────────────────
try:
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("Instala las dependencias: pip install pandas matplotlib")
    raise

# ── Cargar datos ──────────────────────────────────────────────────────────────
CSV_PATH = "señales_robot.csv"
if not os.path.exists(CSV_PATH):
    raise FileNotFoundError(f"No se encontró '{CSV_PATH}'. Ejecuta primero la simulación.")

df = pd.read_csv(CSV_PATH)
t  = df['tiempo_s']

print(f"Muestras cargadas: {len(df)}")
print(f"Duración simulada: {t.iloc[-1]:.2f} s")

# ── FIGURA 1: Señales crudas de todos los sensores ───────────────────────────
fig1, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig1.suptitle('Señales Crudas de Sensores', fontsize=14, fontweight='bold')

axes[0].plot(t, df['ps7_crudo'],  label='ps7 (frontal izq)', color='royalblue')
axes[0].plot(t, df['ps0_crudo'],  label='ps0 (frontal der)', color='steelblue', linestyle='--')
axes[0].set_ylabel('Valor crudo'); axes[0].legend(); axes[0].grid(True, alpha=0.4)
axes[0].set_title('Sensores Frontales (ps7 y ps0)')

axes[1].plot(t, df['ps5_lateral'], label='ps5 (lateral izq)', color='darkorange')
axes[1].plot(t, df['ps2_lateral'], label='ps2 (lateral der)', color='tomato', linestyle='--')
axes[1].set_ylabel('Valor crudo'); axes[1].legend(); axes[1].grid(True, alpha=0.4)
axes[1].set_title('Sensores Laterales (ps5 y ps2)')

axes[2].plot(t, df['vel_izq'], label='Velocidad izquierda', color='mediumseagreen')
axes[2].plot(t, df['vel_der'], label='Velocidad derecha',   color='crimson', linestyle='--')
axes[2].set_ylabel('rad/s'); axes[2].set_xlabel('Tiempo (s)')
axes[2].legend(); axes[2].grid(True, alpha=0.4)
axes[2].set_title('Velocidades de Motores')

plt.tight_layout()
fig1.savefig('grafico_señales_crudas.png', dpi=150)
print("✓ grafico_señales_crudas.png guardado")

# ── FIGURA 2: Comparación crudo vs filtrado vs Kalman ────────────────────────
fig2, ax = plt.subplots(figsize=(13, 5))
ax.plot(t, df['frontal_metros_crudo'],  label='Medición cruda (convertida a m)',
        color='tomato', alpha=0.6, linewidth=1)
ax.plot(t, df['frontal_filtrado'],      label='Filtro simple (media móvil)',
        color='darkorange', linewidth=1.5)
ax.plot(t, df['distancia_kalman'],      label='Estimación Kalman',
        color='royalblue', linewidth=2)
ax.axhline(y=0.16, color='gray', linestyle=':', linewidth=1.2, label='Umbral seguridad (0.16 m)')
ax.set_title('Comparación: Señal Cruda vs Filtro Simple vs Filtro de Kalman',
             fontsize=13, fontweight='bold')
ax.set_xlabel('Tiempo (s)')
ax.set_ylabel('Distancia frontal estimada (m)')
ax.legend()
ax.grid(True, alpha=0.4)
ax.set_ylim(0, 3.2)
plt.tight_layout()
fig2.savefig('grafico_comparacion_filtros.png', dpi=150)
print("✓ grafico_comparacion_filtros.png guardado")

# ── FIGURA 3: Ganancia de Kalman y covarianza implícita ──────────────────────
fig3, ax3 = plt.subplots(figsize=(11, 4))
ax3.plot(t, df['ganancia_kalman'], color='purple', linewidth=1.5, label='Ganancia Kalman (K)')
ax3.set_title('Evolución de la Ganancia de Kalman', fontsize=13, fontweight='bold')
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('K')
ax3.legend()
ax3.grid(True, alpha=0.4)
ax3.set_ylim(0, 1.05)
plt.tight_layout()
fig3.savefig('grafico_ganancia_kalman.png', dpi=150)
print("✓ grafico_ganancia_kalman.png guardado")

# ── FIGURA 4: Avance del robot por encoders ──────────────────────────────────
fig4, ax4 = plt.subplots(figsize=(11, 4))
ax4.plot(t, df['avance_robot_m'].cumsum(), color='teal', linewidth=1.5,
         label='Desplazamiento acumulado (encoders)')
ax4.set_title('Desplazamiento Lineal Acumulado (estimado con encoders)', fontsize=13, fontweight='bold')
ax4.set_xlabel('Tiempo (s)')
ax4.set_ylabel('Distancia acumulada (m)')
ax4.legend()
ax4.grid(True, alpha=0.4)
plt.tight_layout()
fig4.savefig('grafico_encoders.png', dpi=150)
print("✓ grafico_encoders.png guardado")

print("\n¡Listo! Copia los 4 PNG al repositorio GitHub y referencíalos en el README.md")
