import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

T = 100
num_points = 500

# Radio del círculo
r = 70
# Parametros de la trayectoria deseada.
w = np.pi / 12
p = 10
h = 60
dt = 0.001

# Inicializar matrices
xdg = np.zeros(num_points)
ydg = np.zeros(num_points)
zdg = np.zeros(num_points)
xdpg = np.zeros(num_points)
ydpg = np.zeros(num_points)
zdpg = np.zeros(num_points)

# Inicializar matrices para las trayectorias en función del tiempo
t_values = np.linspace(0, T, num_points)
xd_values = np.zeros(num_points)
yd_values = np.zeros(num_points)
zd_values = np.zeros(num_points)
xdp_values = np.zeros(num_points)
ydp_values = np.zeros(num_points)
zdp_values = np.zeros(num_points)

for i in range(num_points):
    t = T * i / num_points
    
    xd = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t)  # + xinih
    yd = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)  # + yinih
    zd = (h / 2) * (1 + np.tanh(t - 3.5))  # + zinih
    
    xdp = r * (p / (1 + p ** 2) + (t - p) / (1 + (t - p) ** 2)) * (-w) * np.sin(w * t)
    ydp = r * (p / (1 + p ** 2) + (t - p) / (1 + (t - p) ** 2)) * w * np.cos(w * t)
    zdp = (h / 2) * np.tanh(t - 3.5)
    
    xdg[i] = xd
    ydg[i] = yd
    zdg[i] = zd

    xdpg[i] = xdp
    ydpg[i] = ydp
    zdpg[i] = zdp

    # Guardar los valores para las trayectorias en función del tiempo
    xd_values[i] = xd
    yd_values[i] = yd
    zd_values[i] = zd
    xdp_values[i] = xdp
    ydp_values[i] = ydp
    zdp_values[i] = zdp

# Crear la figura 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Graficar la trayectoria en 3D
ax.plot3D(xdg, ydg, zdg, label='Trayectoria deseada', color='blue')

# Etiquetas de los ejes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Título de la gráfica
ax.set_title('Trayectorias en 3D')

# Mostrar la leyenda
ax.legend()

# Mostrar la gráfica
plt.show()

# Gráficas en función del tiempo
plt.figure(figsize=(8, 6))
plt.plot(t_values, xd_values, label='xd')
plt.xlabel('t')
plt.ylabel('xd')
plt.title('xd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(t_values, yd_values, label='yd')
plt.xlabel('t')
plt.ylabel('yd')
plt.title('yd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(t_values, zd_values, label='zd')
plt.xlabel('t')
plt.ylabel('zd')
plt.title('zd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(t_values, xdp_values, label='xdp')
plt.xlabel('t')
plt.ylabel('xdp')
plt.title('xdp vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(t_values, ydp_values, label='ydp')
plt.xlabel('t')
plt.ylabel('ydp')
plt.title('ydp vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(t_values, zdp_values, label='zdp')
plt.xlabel('t')
plt.ylabel('zdp')
plt.title('zdp vs t')
plt.legend()
plt.grid(True)
plt.show()