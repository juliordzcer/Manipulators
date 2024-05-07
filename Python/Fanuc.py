import numpy as np
from math import cos, sin, radians, atan2, atan, tanh, sqrt, degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Tiempo
T = 100
num_pun= 600

# Localizacion de la estacion de trabajo 
ex,ey,ez = 0, 0, 0

# Parametros del emplazamiento de la tarea
tx, ty, tz = 804.5, 500, 663.1

# Parametros del robot
d2, d3, d4, r4 = 150, 600, 200, 640 # mm

# Base del robot
bR = 450 #mm

# Parametros del organo terminal
dhx, dhy, dhz, rho = -49.8, 0, 504,-22

# Limites articulares
limth1 = [-170, 170]
limth2 = [- 90, 160]
limth3 = [-180, 267]
limth4 = [-190, 190]
limth5 = [-270, 270]
limth6 = [-360, 360]

th1mean = (max(limth1)+min(limth1))/2
th2mean = (max(limth2)+min(limth2))/2
th3mean = (max(limth3)+min(limth3))/2
th4mean = (max(limth4)+min(limth4))/2
th5mean = (max(limth5)+min(limth5))/2
th6mean = (max(limth6)+min(limth6))/2

delth1mx = abs(max(limth1)-th1mean)
delth2mx = abs(max(limth2)-th2mean)
delth3mx = abs(max(limth3)-th3mean)
delth4mx = abs(max(limth4)-th4mean)
delth5mx = abs(max(limth5)-th5mean)
delth6mx = abs(max(limth6)-th6mean)

th1pmax = 230
th2pmax = 225
th3pmax = 230
th4pmax = 430
th5pmax = 430
th6pmax = 630

th1g = np.zeros(num_pun)
th2g = np.zeros(num_pun)
th3g = np.zeros(num_pun)
th4g = np.zeros(num_pun)
th5g = np.zeros(num_pun)
th6g = np.zeros(num_pun)

th1p = np.zeros(num_pun)
th2p = np.zeros(num_pun)
th3p = np.zeros(num_pun)
th4p = np.zeros(num_pun)
th5p = np.zeros(num_pun)
th6p = np.zeros(num_pun)

x0g = np.zeros(num_pun)
x1g = np.zeros(num_pun)
x2g = np.zeros(num_pun)
x3g = np.zeros(num_pun)
x4g = np.zeros(num_pun)
x5g = np.zeros(num_pun)
x6g = np.zeros(num_pun)
xhg = np.zeros(num_pun)
xdg = np.zeros(num_pun)
xdpg = np.zeros(num_pun)

y0g = np.zeros(num_pun)
y1g = np.zeros(num_pun)
y2g = np.zeros(num_pun)
y3g = np.zeros(num_pun)
y4g = np.zeros(num_pun)
y5g = np.zeros(num_pun)
y6g = np.zeros(num_pun)
yhg = np.zeros(num_pun)
ydg = np.zeros(num_pun)
ydpg = np.zeros(num_pun)

z0g = np.zeros(num_pun)
z1g = np.zeros(num_pun)
z2g = np.zeros(num_pun)
z3g = np.zeros(num_pun)
z4g = np.zeros(num_pun)
z5g = np.zeros(num_pun)
z6g = np.zeros(num_pun)
zhg = np.zeros(num_pun)
zdg = np.zeros(num_pun)
zdpg = np.zeros(num_pun)

tg = np.zeros(num_pun)

r1xg = [np.zeros((2, 1)) for _ in range(num_pun)]
r2xg = [np.zeros((2, 1)) for _ in range(num_pun)]
r3xg = [np.zeros((2, 1)) for _ in range(num_pun)]
r4xg = [np.zeros((2, 1)) for _ in range(num_pun)]
r5xg = [np.zeros((2, 1)) for _ in range(num_pun)]
r6xg = [np.zeros((2, 1)) for _ in range(num_pun)]
rhxg = [np.zeros((2, 1)) for _ in range(num_pun)]

r1yg = [np.zeros((2, 1)) for _ in range(num_pun)]
r2yg = [np.zeros((2, 1)) for _ in range(num_pun)]
r3yg = [np.zeros((2, 1)) for _ in range(num_pun)]
r4yg = [np.zeros((2, 1)) for _ in range(num_pun)]
r5yg = [np.zeros((2, 1)) for _ in range(num_pun)]
r6yg = [np.zeros((2, 1)) for _ in range(num_pun)]
rhyg = [np.zeros((2, 1)) for _ in range(num_pun)]

r1zg = [np.zeros((2, 1)) for _ in range(num_pun)]
r2zg = [np.zeros((2, 1)) for _ in range(num_pun)]
r3zg = [np.zeros((2, 1)) for _ in range(num_pun)]
r4zg = [np.zeros((2, 1)) for _ in range(num_pun)]
r5zg = [np.zeros((2, 1)) for _ in range(num_pun)]
r6zg = [np.zeros((2, 1)) for _ in range(num_pun)]
rhzg = [np.zeros((2, 1)) for _ in range(num_pun)]

# Parametros del movimiento del robot
# solucion del modelo inverso de posicion del robot

# Epsilon
e1, e2, e4 = 1, 1, 1

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

# Especificacion de la ruta deseada de la herramienta
xinih = 300
delx = 0

yinih = 10
dely = 150

zinih = 10
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

for i in range(num_pun):
    t = T*i/num_pun

    # Calcular la función cicloidal del tiempo
    funct = (t/T) - (1/(2 * np.pi)) * (sin(2 * np.pi * t / T))
    funtp = (1/T) * (1 - cos(2 * np.pi * t / T))
    
    # Calcular los ángulos alpha, beta y gamma
    alpha = alphaini + delalpha * funct
    beta = betaini + delbeta * funct
    gamma = gammaini + delgamma * funct

    alphahpun=delalpha*funtp
    betahpun=delbeta*funtp
    gammahpun=delgamma*funtp

    wxh=gammahpun
    wyh=betahpun
    wzh=alphahpun

    # Calcular las coordenadas x, y, z en función del tiempo para seguir un círculo
    # xd = x_center + r * cos(2 * np.pi * funct)
    # yd = y_center + r * sin(2 * np.pi * funct)
    # zd = zinih  

    # xdp = -2 * np.pi * r * sin(2 * np.pi * funct)
    # ydp = 2 * np.pi * r * cos(2 * np.pi * funct)
    # zdp = 0

    xd = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) + xinih
    yd = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t) + yinih
    zd = (h/2) * (1 + np.tanh(t-3.5)) + zinih
    
    xdp = r * (p/(1 + p**2) + (t-p)/(1+(t-p)**2)) * (-w) * sin(w * t)
    ydp = r * (p/(1 + p**2) + (t-p)/(1+(t-p)**2)) * w * cos(w * t)
    zdp = (h/2) * tanh(t - 3.5)

    xph = zd
    yph = xd
    zph = yd

    xphp = zdp
    yphp = xdp
    zphp = ydp

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

    Rthd = np.array([
                    [Rthd11, Rthd12, Rthd13],
                    [Rthd21, Rthd22, Rthd23],
                    [Rthd31, Rthd32, Rthd33],
                    ])
    
    # Matriz homogenea de la pose deseada del marco h con respecto al marco t
    Tthd = np.array([
                    [Rthd[0,0], Rthd[0,1], Rthd[0,2], xph],
                    [Rthd[1,0], Rthd[1,1], Rthd[1,2], yph],
                    [Rthd[2,0], Rthd[2,1], Rthd[2,2], zph],
                    [0,         0,         0,         1]
                    ])


    # Matriz de la pose deseada del marco h c.r al marco 0 
    T0hd = np.dot(T0t,Tthd)
    # Matriz de la pose deseada del marco 6 c.r al marco 0:
    T06d = np.dot(T0hd,Th6) 

    SX = T06d[0, 0]
    SY = T06d[1, 0]
    SZ = T06d[2, 0]

    NX = T06d[0, 1]
    NY = T06d[1, 1]
    NZ = T06d[2, 1]

    AX = T06d[0, 2]
    AY = T06d[1, 2]
    AZ = T06d[2, 2]

    PX = T06d[0, 3]
    PY = T06d[1, 3]
    PZ = T06d[2, 3]

    Snap = np.array([[SX, NX, AX, PX],
                     [SY, NY, AY, PY],
                     [SZ, NZ, AZ, PZ],
                     [0,  0,  0,  1]])
    
    # Calculo del estado de velocidad deseado de la herramiento respecto al marco 0
    vph_t = np.array([xph, yph, zph])
    whh_t = np.array([wxh, wyh, wzh])

    vph_0 = np.dot(R0T, vph_t)
    whh_0 = np.dot(R0T, whh_t)

    nh_0 = [AX * tx, AY * ty, AZ * tz]
    wnn_0 = whh_0

    difvnhx_0 = wnn_0[1] * nh_0[2] - wnn_0[2] * nh_0[1]
    difvnhy_0 = wnn_0[2] * nh_0[0] - wnn_0[0] * nh_0[2]
    difvnhz_0 = wnn_0[0] * nh_0[1] - wnn_0[1] * nh_0[0]

    difvnh_0 = [difvnhx_0, difvnhy_0, difvnhz_0]
    von_0 = vph_0 - difvnh_0
    Spun_0 = np.array([von_0[0], von_0[1], von_0[2], wnn_0[0], wnn_0[1], wnn_0[2]])

    xdg[i] = xd
    ydg[i] = yd
    zdg[i] = zd

    xdpg[i] = xdp
    ydpg[i] = ydp
    zdpg[i] = zdp

    # Cálculo de las variables articulares:
    th1 = atan2(e1 * PY, e1 * PX)

    z1 = -d2 + PX * cos(th1) + PY * sin(th1)
    b1 = 2 * (-(d4 * PZ) - r4 * z1)
    b2 = 2 * (PZ * r4 - d4 * z1)
    b3 = d3 ** 2 - d4 ** 2 - PZ ** 2 - r4 ** 2 - z1 ** 2

    SQ = (b1 * b3 + b2 * sqrt(b1 ** 2 + b2 ** 2 - b3 ** 2) * e2) / (b1 ** 2 + b2 ** 2)
    CQ = (b2 * b3 - b1 * sqrt(b1 ** 2 + b2 ** 2 - b3 ** 2) * e2) / (b1 ** 2 + b2 ** 2)

    th2 = atan2(-((-PZ - r4 * CQ + d4 * SQ) / (d3)), (z1 - d4 * CQ - r4 * SQ) / (d3))

    th3 = atan2(SQ, CQ) - th2

    X = -(AY * cos(th1)) + AX * sin(th1)
    Y = -(AX * cos(th1) * cos(th2 + th3)) - AY * cos(th2 + th3) * sin(th1) - AZ * sin(th2 + th3)

    th4 = atan2(-X * e4, Y * e4)

    Y12 = -(cos(th4) * (AX * cos(th1) * cos(th2 + th3) + AY * cos(th2 + th3) * sin(th1) + AZ * sin(th2 + th3))) - (- (AY * cos(th1) + AX * sin(th1)) * sin(th4))
    Y1 = - AZ * cos(th2 + th3) - AX * cos(th1) * sin(th2 + th3) - AY * sin(th1) * sin(th2 + th3)

    th5 = atan2(-Y12, -Y1)

    Y22 = - (cos(th4) * (-SY * cos(th1) + SX * sin(th1))) + (SX * cos(th1) * cos(th2 + th3) + SY * cos(th2 + th3) * sin(th1) + SZ * sin(th2 + th3)) * sin(th4)
    Y21 = - (cos(th4) * (-NY * cos(th1) + NX * sin(th1))) + (NX * cos(th1) * cos(th2 + th3) + NY * cos(th2 + th3) * sin(th1) + NZ * sin(th2 + th3)) * sin(th4)

    th6 = atan2(-Y22, -Y21)

    # Matriz jacobiana
    J11 = (-d2 - d3*cos(th2) - d4*cos(th2 + th3) - r4*sin(th2 + th3))*sin(th1)
    J21 = (d2 + d3*cos(th2) + d4*cos(th2 + th3) + r4*sin(th2 + th3))*cos(th1)
    J31 = 0
    J41 = 0
    J51 = 0
    J61 = 1
    J12 = (-d3*sin(th2) - d4*sin(th2 + th3) + r4*cos(th2 + th3))*cos(th1)
    J22 = (-d3*sin(th2) - d4*sin(th2 + th3) + r4*cos(th2 + th3))*sin(th1)
    J32 = d3*cos(th2) + d4*cos(th2 + th3) + r4*sin(th2 + th3)
    J42 = sin(th1)
    J52 = -cos(th1)
    J62 = 0
    J13 = (-d4*sin(th2 + th3) + r4*cos(th2 + th3))*cos(th1)
    J23 = (-d4*sin(th2 + th3) + r4*cos(th2 + th3))*sin(th1)
    J33 = d4*cos(th2 + th3) + r4*sin(th2 + th3)
    J43 = sin(th1)
    J53 = -cos(th1)
    J63 = 0
    J14 = 0
    J24 = 0
    J34 = 0
    J44 = sin(th2 + th3)*cos(th1)
    J54 = sin(th1)*sin(th2 + th3)
    J64 = -cos(th2 + th3)
    J15 = 0
    J25 = 0
    J35 = 0
    J45 = sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3)
    J55 = -sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4)
    J65 = -sin(th4)*sin(th2 + th3)
    J16 = 0
    J26 = 0
    J36 = 0
    J46 = sin(th1)*sin(th4)*sin(th5) + sin(th5)*cos(th1)*cos(th4)*cos(th2 + th3) + sin(th2 + th3)*cos(th1)*cos(th5)
    J56 = sin(th1)*sin(th5)*cos(th4)*cos(th2 + th3) + sin(th1)*sin(th2 + th3)*cos(th5) - sin(th4)*sin(th5)*cos(th1)
    J66 = sin(th5)*sin(th2 + th3)*cos(th4) - cos(th5)*cos(th2 + th3)

    JTA = np.array([[J11, J12, J13, J14, J15, J16],
                [J21, J22, J23, J24, J25, J26],
                [J31, J32, J33, J34, J35, J36]])

    JRW = np.array([[J44, J45, J46],
                    [J54, J55, J56],
                    [J64, J65, J66]])

    # Cálculo de Mtrs1 y Mrts1
    Mtrs1 = np.sqrt(np.linalg.det(JTA @ JTA.T))
    Mrts1 = np.sqrt(np.linalg.det(JRW + JRW.T))

    # Matriz J
    J = np.array([[J11, J12, J13, J14, J15, J16],
                [J21, J22, J23, J24, J25, J26],
                [J31, J32, J33, J34, J35, J36],
                [J41, J42, J43, J44, J45, J46],
                [J51, J52, J53, J54, J55, J56],
                [J61, J62, J63, J64, J65, J66]])

    # Cálculo de las velocidades articulares
    qp = np.linalg.inv(J) @ Spun_0

    # Asignación de las velocidades articulares a las variables correspondientes
    th1p[i] = qp[0]
    th2p[i] = qp[1]
    th3p[i] = qp[2]
    th4p[i] = qp[3]
    th5p[i] = qp[4]
    th6p[i] = qp[5]

    # PREPARACION DE MATRICES PARA LA ANIMACION
    # MATRICES DE TRANSFORMACION HOMOGENEAS

    # Cálculo de las transformaciones T0T6
    T0T611 = ((sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)) * cos(th5) - sin(th5) * sin(th2 + th3) * cos(th1)) * cos(th6) + (sin(th1) * cos(th4) - sin(th4) * cos(th1) * cos(th2 + th3)) * sin(th6)
    T0T621 = ((sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)) * cos(th5) - sin(th1) * sin(th5) * sin(th2 + th3)) * cos(th6) + (-sin(th1) * sin(th4) * cos(th2 + th3) - cos(th1) * cos(th4)) * sin(th6)
    T0T631 = (sin(th5) * cos(th2 + th3) + sin(th2 + th3) * cos(th4) * cos(th5)) * cos(th6) - sin(th4) * sin(th6) * sin(th2 + th3)
    T0T612 = -((sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)) * cos(th5) - sin(th5) * sin(th2 + th3) * cos(th1)) * sin(th6) + (sin(th1) * cos(th4) - sin(th4) * cos(th1) * cos(th2 + th3)) * cos(th6)
    T0T622 = -((sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)) * cos(th5) - sin(th1) * sin(th5) * sin(th2 + th3)) * sin(th6) + (-sin(th1) * sin(th4) * cos(th2 + th3) - cos(th1) * cos(th4)) * cos(th6)
    T0T632 = -(sin(th5) * cos(th2 + th3) + sin(th2 + th3) * cos(th4) * cos(th5)) * sin(th6) - sin(th4) * sin(th2 + th3) * cos(th6)
    T0T613 = (sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)) * sin(th5) + sin(th2 + th3) * cos(th1) * cos(th5)
    T0T623 = (sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)) * sin(th5) + sin(th1) * sin(th2 + th3) * cos(th5)
    T0T633 = sin(th5) * sin(th2 + th3) * cos(th4) - cos(th5) * cos(th2 + th3)
    T0T614 = d2 * cos(th1) + d3 * cos(th1) * cos(th2) + d4 * cos(th1) * cos(th2 + th3) + r4 * sin(th2 + th3) * cos(th1)
    T0T624 = d2 * sin(th1) + d3 * sin(th1) * cos(th2) + d4 * sin(th1) * cos(th2 + th3) + r4 * sin(th1) * sin(th2 + th3)
    T0T634 = d3 * sin(th2) + d4 * sin(th2 + th3) - r4 * cos(th2 + th3)

    # Tramsformation matrix 0 T 5
    T0T511 = (sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)) * cos(th5) - sin(th5) * sin(th2 + th3) * cos(th1)
    T0T521 = (sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)) * cos(th5) - sin(th1) * sin(th5) * sin(th2 + th3)
    T0T531 = sin(th5) * cos(th2 + th3) + sin(th2 + th3) * cos(th4) * cos(th5)
    T0T512 = -(sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)) * sin(th5) - sin(th2 + th3) * cos(th1) * cos(th5)
    T0T522 = -(sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)) * sin(th5) - sin(th1) * sin(th2 + th3) * cos(th5)
    T0T532 = -sin(th5) * sin(th2 + th3) * cos(th4) + cos(th5) * cos(th2 + th3)
    T0T513 = sin(th1) * cos(th4) - sin(th4) * cos(th1) * cos(th2 + th3)
    T0T523 = -sin(th1) * sin(th4) * cos(th2 + th3) - cos(th1) * cos(th4)
    T0T533 = -sin(th4) * sin(th2 + th3)
    T0T514 = d2 * cos(th1) + d3 * cos(th1) * cos(th2) + d4 * cos(th1) * cos(th2 + th3) + r4 * sin(th2 + th3) * cos(th1)
    T0T524 = d2 * sin(th1) + d3 * sin(th1) * cos(th2) + d4 * sin(th1) * cos(th2 + th3) + r4 * sin(th1) * sin(th2 + th3)
    T0T534 = d3 * sin(th2) + d4 * sin(th2 + th3) - r4 * cos(th2 + th3)

    # Tramsformation matrix 0 T 4
    T0T411 = sin(th1) * sin(th4) + cos(th1) * cos(th4) * cos(th2 + th3)
    T0T421 = sin(th1) * cos(th4) * cos(th2 + th3) - sin(th4) * cos(th1)
    T0T431 = sin(th2 + th3) * cos(th4)
    T0T412 = sin(th1) * cos(th4) - sin(th4) * cos(th1) * cos(th2 + th3)
    T0T422 = -sin(th1) * sin(th4) * cos(th2 + th3) - cos(th1) * cos(th4)
    T0T432 = -sin(th4) * sin(th2 + th3)
    T0T413 = sin(th2 + th3) * cos(th1)
    T0T423 = sin(th1) * sin(th2 + th3)
    T0T433 = -cos(th2 + th3)
    T0T414 = d2 * cos(th1) + d3 * cos(th1) * cos(th2) + d4 * cos(th1) * cos(th2 + th3) + r4 * sin(th2 + th3) * cos(th1)
    T0T424 = d2 * sin(th1) + d3 * sin(th1) * cos(th2) + d4 * sin(th1) * cos(th2 + th3) + r4 * sin(th1) * sin(th2 + th3)
    T0T434 = d3 * sin(th2) + d4 * sin(th2 + th3) - r4 * cos(th2 + th3)

    # Tramsformation matrix 0 T 3
    T0T311 = cos(th1) * cos(th2 + th3)
    T0T321 = sin(th1) * cos(th2 + th3)
    T0T331 = sin(th2 + th3)
    T0T312 = -sin(th2 + th3) * cos(th1)
    T0T322 = -sin(th1) * sin(th2 + th3)
    T0T332 = cos(th2 + th3)
    T0T313 = sin(th1)
    T0T323 = -cos(th1)
    T0T333 = 0
    T0T314 = d2 * cos(th1) + d3 * cos(th1) * cos(th2)
    T0T324 = d2 * sin(th1) + d3 * sin(th1) * cos(th2)
    T0T334 = d3 * sin(th2)

    # Tramsformation matrix 0 T 2
    T0T211 = cos(th1) * cos(th2)
    T0T221 = sin(th1) * cos(th2)
    T0T231 = sin(th2)
    T0T212 = -sin(th2) * cos(th1)
    T0T222 = -sin(th1) * sin(th2)
    T0T232 = cos(th2)
    T0T213 = sin(th1)
    T0T223 = -cos(th1)
    T0T233 = 0
    T0T214 = d2 * cos(th1)
    T0T224 = d2 * sin(th1)
    T0T234 = 0

    # Tramsformation matrix 0 T 1
    T0T111 = cos(th1)
    T0T121 = sin(th1)
    T0T131 = 0
    T0T112 = -sin(th1)
    T0T122 = cos(th1)
    T0T132 = 0
    T0T113 = 0
    T0T123 = 0
    T0T133 = 1
    T0T114 = 0
    T0T124 = 0
    T0T134 = 0

    T01 = np.array([
        [T0T111, T0T112, T0T113, T0T114],
        [T0T121, T0T122, T0T123, T0T124],
        [T0T131, T0T132, T0T133, T0T134],
        [0, 0, 0, 1]
    ])

    T02 = np.array([
        [T0T211, T0T212, T0T213, T0T214],
        [T0T221, T0T222, T0T223, T0T224],
        [T0T231, T0T232, T0T233, T0T234],
        [0, 0, 0, 1]
    ])

    T03 = np.array([
        [T0T311, T0T312, T0T313, T0T314],
        [T0T321, T0T322, T0T323, T0T324],
        [T0T331, T0T332, T0T333, T0T334],
        [0, 0, 0, 1]
    ])

    T04 = np.array([
        [T0T411, T0T412, T0T413, T0T414],
        [T0T421, T0T422, T0T423, T0T424],
        [T0T431, T0T432, T0T433, T0T434],
        [0, 0, 0, 1]
    ])

    T05 = np.array([
        [T0T511, T0T512, T0T513, T0T514],
        [T0T521, T0T522, T0T523, T0T524],
        [T0T531, T0T532, T0T533, T0T534],
        [0, 0, 0, 1]
    ])

    T06 = np.array([
        [T0T611, T0T612, T0T613, T0T614],
        [T0T621, T0T622, T0T623, T0T624],
        [T0T631, T0T632, T0T633, T0T634],
        [0, 0, 0, 1]
    ])

    T0h = np.dot(T06, T6h)

    # Poses de los marcos de la cadena cinematica
    TE1 = np.dot(TE0, T01)
    TE2 = np.dot(TE0, T02)
    TE3 = np.dot(TE0, T03)
    TE4 = np.dot(TE0, T04)
    TE5 = np.dot(TE0, T05)
    TE6 = np.dot(TE0, T06)
    TEh = np.dot(TE0, T0h)

    # COORDENADAS DE LA FIGURA 1 (EMPLAZADO)
    x0, y0, z0 = TE0[:3, 3]
    x1, y1, z1 = TE1[:3, 3]
    x2, y2, z2 = TE2[:3, 3]
    x3, y3, z3 = TE3[:3, 3]
    x4, y4, z4 = TE4[:3, 3]
    x5, y5, z5 = TE5[:3, 3]
    x6, y6, z6 = TE6[:3, 3]
    xh, yh, zh = TEh[:3, 3]

    # Conversiones de ángulos a grados
    th1g[i] = degrees(th1)
    th2g[i] = degrees(th2)
    th3g[i] = degrees(th3)
    th4g[i] = degrees(th4)
    th5g[i] = degrees(th5)
    th6g[i] = degrees(th6)
    
    x0g[i] = x0
    x1g[i] = x1
    x2g[i] = x2
    x3g[i] = x3
    x4g[i] = x4
    x5g[i] = x5
    x6g[i] = x6
    xhg[i] = xh
    
    y0g[i] = y0
    y1g[i] = y1
    y2g[i] = y2
    y3g[i] = y3
    y4g[i] = y4
    y5g[i] = y5
    y6g[i] = y6
    yhg[i] = yh
    
    z0g[i] = z0
    z1g[i] = z1
    z2g[i] = z2
    z3g[i] = z3
    z4g[i] = z4
    z5g[i] = z5
    z6g[i] = z6
    zhg[i] = zh

    tg[i] = t


    # Definición de cadenas cinemáticas
    r1x = [x0, x1]
    r1y = [y0, y1]
    r1z = [z0, z1]

    r2x = [x1, x2]
    r2y = [y1, y2]
    r2z = [z1, z2]

    r3x = [x2, x3]
    r3y = [y2, y3]
    r3z = [z2, z3]

    r4x = [x3, x4]
    r4y = [y3, y4]
    r4z = [z3, z4]

    r5x = [x4, x5]
    r5y = [y4, y5]
    r5z = [z4, z5]

    r6x = [x5, x6]
    r6y = [y5, y6]
    r6z = [z5, z6]

    rhx = [x6, xh]
    rhy = [y6, yh]
    rhz = [z6, zh]

    rz = [0, 0]

    r1xg[i] = r1x
    r2xg[i] = r2x
    r3xg[i] = r3x
    r4xg[i] = r4x
    r5xg[i] = r5x
    r6xg[i] = r6x
    rhxg[i] = rhx

    r1yg[i] = r1y
    r2yg[i] = r2y
    r3yg[i] = r3y
    r4yg[i] = r4y
    r5yg[i] = r5y
    r6yg[i] = r6y
    rhyg[i] = rhy

    r1zg[i] = r1z
    r2zg[i] = r2z
    r3zg[i] = r3z
    r4zg[i] = r4z
    r5zg[i] = r5z
    r6zg[i] = r6z
    rhzg[i] = rhz

    # Definición de vástago
    ss1 = np.array([0, 0, -400, 1])
    xx1 = np.dot(TE0, ss1)
    s1x = [TE0[0, 3], xx1[0]]
    s1y = [TE0[1, 3], xx1[1]]
    s1z = [TE0[2, 3], xx1[2]]

    # Definición de la base
    ss2 = np.array([100, 100, -400, 1])
    xx2 = np.dot(TE0, ss2)

    ss3 = np.array([-100, 100, -400, 1])
    xx3 = np.dot(TE0, ss3)

    ss4 = np.array([100, -100, -400, 1])
    xx4 = np.dot(TE0, ss4)

    ss5 = np.array([-100, -100, -400, 1])
    xx5 = np.dot(TE0, ss5)

    b1x = [xx2[0], xx3[0]]
    b1y = [xx2[1], xx3[1]]
    b1z = [xx2[2], xx3[2]]

    b2x = [xx3[0], xx5[0]]
    b2y = [xx3[1], xx5[1]]
    b2z = [xx3[2], xx5[2]]

    b3x = [xx5[0], xx4[0]]
    b3y = [xx5[1], xx4[1]]
    b3z = [xx5[2], xx4[2]]

    b4x = [xx4[0], xx2[0]]
    b4y = [xx4[1], xx2[1]]
    b4z = [xx4[2], xx2[2]]

    # Definicion del marco 0
    ss6 = np.array([300, 0, 0, 1])
    xx6 = np.dot(TE0, ss6)
    ss7 = np.array([0, 300, 0, 1])
    xx7 = np.dot(TE0, ss7)
    ss8 = np.array([0, 0, 300, 1])
    xx8 = np.dot(TE0, ss8)

    e1x = [TE0[0, 3], xx6[0]]
    e1y = [TE0[1, 3], xx6[1]]
    e1z = [TE0[2, 3], xx6[2]]

    e2x = [TE0[0, 3], xx7[0]]
    e2y = [TE0[1, 3], xx7[1]]
    e2z = [TE0[2, 3], xx7[2]]

    e3x = [TE0[0, 3], xx8[0]]
    e3y = [TE0[1, 3], xx8[1]]
    e3z = [TE0[2, 3], xx8[2]]


# Visualización existente
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
ax.plot(x1g, y1g, z1g)
ax.plot(x2g, y2g, z2g)
ax.plot(x3g, y3g, z3g)
ax.plot(x4g, y4g, z4g)
ax.plot(x5g, y5g, z5g)
ax.plot(x6g, y6g, z6g)
ax.plot(xhg, yhg, zhg)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

fig2 = plt.figure()
# Crear subplots adicionales
ax1 = fig2.add_subplot(323)
ax1.plot(tg, x6g, 'r')
ax1.plot(tg, xhg, 'b')
ax1.set_title('x vs t')
ax1.set_xlabel('t')
ax1.set_ylabel('x')

ax2 = fig2.add_subplot(324)
ax2.plot(tg, y6g, 'r')
ax2.plot(tg, yhg, 'b')
ax2.set_title('y vs t')
ax2.set_xlabel('t')
ax2.set_ylabel('y')

ax3 = fig2.add_subplot(325)
ax3.plot(tg, z6g, 'r')
ax3.plot(tg, zhg, 'b')
ax3.set_title('z vs t')
ax3.set_xlabel('t')
ax3.set_ylabel('z')

# Ajustar el espaciado entre subplots
plt.tight_layout()

# Mostrar la gráfica
plt.show()

# # Inicializar la figura y el eje 3D
# fig3 = plt.figure()
# ax = fig3.add_subplot(111, projection='3d')

# # Crear las líneas iniciales
# line1, = ax.plot(x1g, y1g, z1g)
# line2, = ax.plot(x2g, y2g, z2g)
# line3, = ax.plot(x3g, y3g, z3g)
# line4, = ax.plot(x4g, y4g, z4g)
# line5, = ax.plot(x5g, y5g, z5g)
# line6, = ax.plot(x6g, y6g, z6g)
# line7, = ax.plot(xhg, yhg, zhg)

# # Función de actualización para la animación
# def update(frame):
#     # Actualizar los datos de las líneas originales
#     line1.set_data(x1g[:frame], y1g[:frame])
#     line1.set_3d_properties(z1g[:frame])

#     line2.set_data(x2g[:frame], y2g[:frame])
#     line2.set_3d_properties(z2g[:frame])

#     line3.set_data(x3g[:frame], y3g[:frame])
#     line3.set_3d_properties(z3g[:frame])

#     line4.set_data(x4g[:frame], y4g[:frame])
#     line4.set_3d_properties(z4g[:frame])

#     line5.set_data(x5g[:frame], y5g[:frame])
#     line5.set_3d_properties(z5g[:frame])

#     line6.set_data(x6g[:frame], y6g[:frame])
#     line6.set_3d_properties(z6g[:frame])

#     line7.set_data(xhg[:frame], yhg[:frame])
#     line7.set_3d_properties(zhg[:frame])

#     return line1, line2, line3, line4, line5, line6, line7

# # Función de actualización para la animación 3D
# def update_3d(frame):
#     update(frame)
#     return line1, line2, line3, line4, line5, line6, line7

# # Crear la animación
# ani = FuncAnimation(fig3, update_3d, frames=len(x1g), interval=50)

# # Mostrar la animación
# plt.show()


# # Inicializar la figura y el eje 3D
# fig3 = plt.figure()
# ax = fig3.add_subplot(111, projection='3d')

# # Crear las líneas iniciales para los eslabones
# lines = [ax.plot([], [], [])[0] for _ in range(7)]

# # Función de inicialización para la animación
# def init():
#     for line in lines:
#         line.set_data([], [])
#         line.set_3d_properties([])
#     return lines

# # Función de actualización para la animación
# def update(frame):
#     # Actualizar las coordenadas de los extremos de los eslabones
#     coordinates = [
#         (x1g[frame], y1g[frame], z1g[frame]),
#         (x2g[frame], y2g[frame], z2g[frame]),
#         (x3g[frame], y3g[frame], z3g[frame]),
#         (x4g[frame], y4g[frame], z4g[frame]),
#         (x5g[frame], y5g[frame], z5g[frame]),
#         (x6g[frame], y6g[frame], z6g[frame]),
#         (xhg[frame], yhg[frame], zhg[frame])
#     ]

#     # Actualizar las líneas que representan los eslabones
#     for i, line in enumerate(lines):
#         if i + 1 < len(coordinates):
#             line.set_data([coordinates[i][0], coordinates[i+1][0]], [coordinates[i][1], coordinates[i+1][1]])
#             line.set_3d_properties([coordinates[i][2], coordinates[i+1][2]])

#     return lines

# # Configurar los límites de los ejes XYZ con un margen de 200 unidades
# margin = 200
# xmin = min(x1g.min(), xhg.min()) - margin
# xmax = max(x1g.max(), xhg.max()) + margin
# ymin = min(y1g.min(), yhg.min()) - margin
# ymax = max(y1g.max(), yhg.max()) + margin
# zmin = min(z1g.min(), zhg.min()) - margin
# zmax = max(z1g.max(), zhg.max()) + margin

# ax.set_xlim([xmin, xmax])
# ax.set_ylim([ymin, ymax])
# ax.set_zlim([zmin, zmax])

# # Crear la animación
# ani = FuncAnimation(fig3, update, frames=len(x1g), init_func=init, interval=50)

# # Mostrar la animación
# plt.show()


# Inicializar la figura y el eje 3D
fig3 = plt.figure()
ax = fig3.add_subplot(111, projection='3d')

# Crear las líneas iniciales para los eslabones
lines = [ax.plot([], [], [])[0] for _ in range(7)]

# Función de inicialización para la animación
def init():
    for line in lines:
        line.set_data([], [])
        line.set_3d_properties([])
    return lines

# Función de actualización para la animación
def update(frame):
    # Actualizar las coordenadas de los extremos de los eslabones
    coordinates = [
        (x1g[frame], y1g[frame], z1g[frame]),
        (x2g[frame], y2g[frame], z2g[frame]),
        (x3g[frame], y3g[frame], z3g[frame]),
        (x4g[frame], y4g[frame], z4g[frame]),
        (x5g[frame], y5g[frame], z5g[frame]),
        (x6g[frame], y6g[frame], z6g[frame]),
        (xhg[frame], yhg[frame], zhg[frame])
    ]

    # Actualizar las líneas que representan los eslabones
    for i, line in enumerate(lines):
        if i + 1 < len(coordinates):
            line.set_data([coordinates[i][0], coordinates[i+1][0]], [coordinates[i][1], coordinates[i+1][1]])
            line.set_3d_properties([coordinates[i][2], coordinates[i+1][2]])

    return lines

# Configurar los límites de los ejes XYZ con un margen de 200 unidades
margin = 200
xmin = min(x1g.min(), xhg.min()) - margin
xmax = max(x1g.max(), xhg.max()) + margin
ymin = min(y1g.min(), yhg.min()) - margin
ymax = max(y1g.max(), yhg.max()) + margin
zmin = min(z1g.min(), zhg.min()) - margin
zmax = max(z1g.max(), zhg.max()) + margin

ax.set_xlim([xmin, xmax])
ax.set_ylim([ymin, ymax])
ax.set_zlim([zmin, zmax])

# Crear las líneas iniciales para la trayectoria de las articulaciones
line1, = ax.plot([], [], [])
line2, = ax.plot([], [], [])
line3, = ax.plot([], [], [])
line4, = ax.plot([], [], [])
line5, = ax.plot([], [], [])
line6, = ax.plot([], [], [])
line7, = ax.plot([], [], [])

# Función de actualización para la trayectoria de las articulaciones
def update_joint_trajectory(frame):
    line1.set_data(x1g[:frame], y1g[:frame])
    line1.set_3d_properties(z1g[:frame])

    line2.set_data(x2g[:frame], y2g[:frame])
    line2.set_3d_properties(z2g[:frame])

    line3.set_data(x3g[:frame], y3g[:frame])
    line3.set_3d_properties(z3g[:frame])

    line4.set_data(x4g[:frame], y4g[:frame])
    line4.set_3d_properties(z4g[:frame])

    line5.set_data(x5g[:frame], y5g[:frame])
    line5.set_3d_properties(z5g[:frame])

    line6.set_data(x6g[:frame], y6g[:frame])
    line6.set_3d_properties(z6g[:frame])

    line7.set_data(xhg[:frame], yhg[:frame])
    line7.set_3d_properties(zhg[:frame])

    return line1, line2, line3, line4, line5, line6, line7

# Crear la animación de los eslabones
ani1 = FuncAnimation(fig3, update, frames=len(x1g), init_func=init, interval=50)

# Crear la animación de la trayectoria de las articulaciones
ani2 = FuncAnimation(fig3, update_joint_trajectory, frames=len(x1g), interval=50)

# Mostrar la animación
plt.show()