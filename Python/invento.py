import numpy as np
from math import cos, sin, radians, atan2, atan, tanh, sqrt, degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parámetros del sistema
T = 100
num_points = 500

# Posición y orientación de la herramienta
rho = -22
dhx, dhy, dhz = -49.8, 0, 504

# Localización de la estación de trabajo
ex, ey, ez = 0, 0, 0

# Parámetros de emplazamiento de tarea
tx, ty, tz, phi = 804.5, 499.9, 663.1, -9.9

# Parámetros del robot
d2, d3, d4, r4 = 150, 600, 200, 640
bR, eo = 450, 100

# Limites articulares
joint_limits = {
    1: (-170, 170),
    2: (-90, 160),
    3: (-180, 267),
    4: (-190, 190),
    5: (-270, 270),
    6: (-360, 360)
}

# Velocidades máximas
max_speeds = {
    1: 230,
    2: 225,
    3: 230,
    4: 430,
    5: 430,
    6: 630
}

# Variables articulares
thg = np.zeros((6, num_points))
thp = np.zeros((6, num_points))

# Trayectoria deseada
x_center, y_center = 100, 1000
r = 70
w = np.pi / 12
p, h = 10, 60
dt = 0.001

# Funciones auxiliares
def rotation_matrix(axis, theta):
    return np.diag([cos(theta)] * 3) + (1 - cos(theta)) * np.outer(axis, axis) + sin(theta) * np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])

# Matriz de rotación
rho_rad = np.radians(rho)
T6h = rotation_matrix([0, 1, 0], rho_rad)
Th6 = np.linalg.inv(T6h)

# Posición inicial
TE0 = np.array([
    [1, 0, 0, ex],
    [0, 1, 0, ey],
    [0, 0, 1, ez + bR],
    [0, 0, 0, 1]
])

TEt = np.array([
    [0, -1, 0, tx],
    [0, 0, -1, ty],
    [1, 0, 0, tz],
    [0, 0, 0, 1]
])

T0t = np.dot(np.linalg.inv(TE0), TEt)
R0T = T0t[:3, :3]

# Trayectoria
t = np.linspace(0, T, num_points)

# Trayectoria circular
x_desired = x_center + r * np.cos(w * t + p)
y_desired = y_center + r * np.sin(w * t + p)
z_desired = np.linspace(100, 100 + h, num_points)

# Visualización
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_desired, y_desired, z_desired)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()