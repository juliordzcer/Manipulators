import numpy as np
from math import cos, sin, radians, atan2, atan, tanh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Tiempo
T = 100
num_points = 500

# Localizacion de la estacion de trabajo
ex, ey, ez = 0, 0, 0  # mm

# Parametros de emplazamiento de tarea
tx, ty, tz, phi = 804.5, 499.9, 663.1, -9.9  # mm, grados

# Parametros del robot
d2, d3, d4, r4 = 150, 600, 200, 640  # mm

bR, eo = 450, 100

# Parametros del organo terminal
rho, dhx, dhy, dhz = -22, -49.8, 0, 504  # Grados, mm

# Limites articulares
joint_limits = {
    'th1': (-170, 170),
    'th2': (-90, 160),
    'th3': (-180, 267),
    'th4': (-190, 190),
    'th5': (-270, 270),
    'th6': (-360, 360)
}

# Velocidades maximas
thp_max = {'th1': 230, 'th2': 225, 'th3': 230, 'th4': 430, 'th5': 430, 'th6': 630}

# Variables articulares en grados
thg = {f'th{i}g': np.zeros(num_points) for i in range(1, 7)}
xg = {f'x{i}g': np.zeros(num_points) for i in range(7)}
yg = {f'y{i}g': np.zeros(num_points) for i in range(7)}
zg = {f'z{i}g': np.zeros(num_points) for i in range(7)}

xdg = np.zeros(num_points)
ydg = np.zeros(num_points)
zdg = np.zeros(num_points)

xdpg = np.zeros(num_points)
ydpg = np.zeros(num_points)
zdpg = np.zeros(num_points)

tg = np.linspace(0,T,num_points)

# Parametros del movimiento del robot
e = {'e1': 1, 'e2': 1, 'e4': -1}

# Matriz del marco 6 al marco de la herramienta
rho_rad = radians(rho)
T6h = np.array([
    [cos(rho_rad), 0, -sin(rho_rad), dhx],
    [0, 1, 0, dhy],
    [sin(rho_rad), 0, cos(rho_rad), dhz],
    [0, 0, 0, 1]
])

Th6 = np.linalg.inv(T6h)

# Emplazamiento en el piso
TE0 = np.array([
    [1, 0, 0, ex],
    [0, 1, 0, ey],
    [0, 0, 1, ez + bR],
    [0, 0, 0, 1]
])

# Emplazamiento de la tarea
TEt = np.array([
    [0, -1, 0, tx],
    [0, 0, -1, ty],
    [1, 0, 0, tz],
    [0, 0, 0, 1]
])

T0t = np.dot(np.linalg.inv(TE0), TEt)
R0T = T0t[:3, :3]

# Especificacion de los parametros de la ruta deseada de la herramienta
xinih = 300
# delx = 0

yinih = 10
# dely = 150

zinih = 100
delz = 10

# Orientacion en angulos de Euler
alphaini = 0  # rad
delalpha = 0  # rad

betaini = 0  # rad
delbeta = 0  # rad

gammaini = 0  # rad
delgamma = 0  # rad

# Parametros de la trayectoria deseada 
# Coordenadas del centro del círculo
x_center = 100  # Coordenada x del centro del círculo
y_center = 1000  # Coordenada y del centro del círculo

# Radio del círculo
r = 70  # Radio del círculo
# Parametros de la trayectoria deseada.
w = np.pi/12
p = 10
h = 60
dt = 0.001


for i in range(num_points):
    t = T * i / num_points
    
    # Calcular la función cicloidal del tiempo
    funct = (t/T) - (1/(2*np.pi)) * (sin(2*np.pi*t/T))
    
    # Calcular los ángulos alpha, beta y gamma
    alpha = alphaini + delalpha * funct
    beta = betaini + delbeta * funct
    gamma = gammaini + delgamma * funct
    
    # Calcular las coordenadas x, y, z en función del tiempo para seguir un círculo
    # xph1 = x_center + r * cos(2 * np.pi * funct)
    # yph1 = y_center + r * sin(2 * np.pi * funct)
    # zph1 = zinih  

    xd = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) + xinih
    yd = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t) + yinih
    zd = (h/2) * (1 + np.tanh(t-3.5)) + zinih
    
    xdp = r * (p/(1 + p**2) + (t-p)/(1+(t-p)**2)) * (-w) * sin(w * t)
    ydp = r * (p/(1 + p**2) + (t-p)/(1+(t-p)**2)) * w * cos(w * t)
    zdp = (h/2) * tanh(t - 3.5)
    
    xph = zd
    yph = xd
    zph = yd

    # Cálculo de las variaciones en las coordenadas x, y, z y en los ángulos de orientación alpha, beta y gamma
    delx = 0      #(2*pi*radius/T) * (-sin(2*pi*t/T));
    dely = 0      #(2*pi*radius/T) * (cos(2*pi*t/T));
    delz = 0
    delalpha = 0
    delbeta = 0
    delgamma = 0

    Rthd11 = cos(alpha)*cos(beta)
    Rthd21 = sin(alpha)*cos(beta)
    Rthd31 =-sin(beta)
    Rthd12 = cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma)
    Rthd22 = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)
    Rthd32 = cos(beta)*sin(gamma)
    Rthd13 = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma)
    Rthd23 = sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma)
    Rthd33 =  cos(beta)*cos(gamma)

    Rthd=np.array([
            [Rthd11, Rthd12, Rthd13],
            [Rthd21, Rthd22, Rthd23],
            [Rthd31, Rthd32, Rthd33],
            ])
    
    xdg[i] = xd
    ydg[i] = yd
    zdg[i] = zd

    xdpg[i] = xdp
    ydpg[i] = ydp
    zdpg[i] = zdp






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
plt.plot(tg, xdg, label='xd')
plt.xlabel('t')
plt.ylabel('xd')
plt.title('xd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(tg, ydg, label='yd')
plt.xlabel('t')
plt.ylabel('yd')
plt.title('yd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(tg, zdg, label='zd')
plt.xlabel('t')
plt.ylabel('zd')
plt.title('zd vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(tg, xdpg, label='xdp')
plt.xlabel('t')
plt.ylabel('xdp')
plt.title('xdp vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(tg, ydpg, label='ydp')
plt.xlabel('t')
plt.ylabel('ydp')
plt.title('ydp vs t')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(tg, zdpg, label='zdp')
plt.xlabel('t')
plt.ylabel('zdp')
plt.title('zdp vs t')
plt.legend()
plt.grid(True)
plt.show()