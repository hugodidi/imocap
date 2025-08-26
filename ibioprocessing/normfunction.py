# Remuestrear ángulos (grados) desde CSV:  -> 100 instancias (0..99)
# Pasos:
# 1) Leer CSV (columna de ángulos en grados)
# 2) Graficar datos crudos
# 3) Imprimir len(datos_crudos)
# 4) Normalizar a 100 instancias (unwrap→interp→wrap)
# 5) Graficar datos normalizados
# 6) Imprimir len(datos_normalizados)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

datos_procesados = None
N = len(datos_procesados)
x_old = np.linspace(0, 99, N)              # dominio normalizado del original
x_new = np.arange(100)                     # 0..99

angles_rad = np.deg2rad(datos_procesados)
u = np.unwrap(angles_rad)                  # quitar saltos ±π
u_new = np.interp(x_new, x_old, u)         # interpolación en dominio 0..99
y_rad = (u_new + np.pi) % (2*np.pi) - np.pi  # reenvolver a [-π, π)
datos_normalizados = np.rad2deg(y_rad)

plt.figure()
plt.plot(x_new, datos_normalizados)
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("Normalized")
print("len(datos_normalizados) =", len(datos_normalizados))
plt.legend(); plt.grid(True); plt.show()

