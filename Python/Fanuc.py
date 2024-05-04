import numpy as np
from math import cos, sin, atan2, sqrt, radians, degrees

# Tiempo
T = 10

# Numero de puntos de la tarea
np = 50

# Localizacion de la estacion de trabajo
ex, ey, ez = 0, 0, 0

# Parametros del emplazamiento de la tarea
tx, ty, tz = 804.5, 499.9, 663.2
phi = -9.9

# Parametros del robot
d2, d3, d4, r4 = 150, 600, 200, 640
bR = 450
eo = 100

# Parametros del organo terminal
rho = -22  # Grados
dhx = -49.8  # mm
dhy = 0  # mm
dhz = 504  # mm

# Limites articulares
th1min, th1max = -170, 170  # deg
th2min, th2max = -90, 160  # deg
th3min, th3max = -180, 267  # deg
th4min, th4max = -190, 190  # deg
th5min, th5max = -270, 270  # deg
th6min, th6max = -360, 360  # deg

# Velocidades maximas
th1pmax, th2pmax, th3pmax = 230, 225, 230  # deg/s
th4pmax, th5pmax, th6pmax = 430, 430, 630  # deg/s

th1g = np.zeros(np)
th2g = np.zeros(np)
th3g = np.zeros(np)
th4g = np.zeros(np)
th5g = np.zeros(np)
th6g = np.zeros(np)
time = np.zeros(np)

x0g = np.zeros(np)
y0g = np.zeros(np)
z0g = np.zeros(np)

x1g = np.zeros(np)
y1g = np.zeros(np)
z1g = np.zeros(np)

x2g = np.zeros(np)
y2g = np.zeros(np)
z2g = np.zeros(np)

x3g = np.zeros(np)
y3g = np.zeros(np)
z3g = np.zeros(np)

x4g = np.zeros(np)
y4g = np.zeros(np)
z4g = np.zeros(np)

x5g = np.zeros(np)
y5g = np.zeros(np)
z5g = np.zeros(np)

x6g = np.zeros(np)
y6g = np.zeros(np)
z6g = np.zeros(np)

xhg = np.zeros(np)
yhg = np.zeros(np)
zhg = np.zeros(np)

# Epsilon
e1, e2, e4 = 1, 1, -1

# Matriz del marco 6 al marco de la herramienta
T6h = np.array([[cos(radians(rho)), 0, -sin(radians(rho)), dhx],
                [0, 1, 0, dhy],
                [sin(radians(rho)), 0, cos(radians(rho)), dhz],
                [0, 0, 0, 1]])
Th6 = np.linalg.inv(T6h)

# Emplazamiento en el piso
TE0 = np.array([[1, 0, 0, ex],
                [0, 1, 0, ey],
                [0, 0, 1, ez + bR],
                [0, 0, 0, 1]])

T0E = np.linalg.inv(TE0)

# Emplazamiento de la tarea
TEt = np.array([[ 0,-1, 0, tx],
                [ 0, 0,-1, ty],
                [-1, 0, 0, tz],
                [ 0, 0, 0, 1]])

T0t = T0E * TEt

for i in range(np):

    t = T * i / (np -1)
    time[i] = t
    
    f = (t / T) - (1 / (2 * np.pi)) * (sin(2 * np.pi * t / T))

