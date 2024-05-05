import numpy as np
from math import cos, sin, radians, atan2, atan, tanh, sqrt, degrees
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

th1g=np.zeros(num_points)
th2g=np.zeros(num_points)
th3g=np.zeros(num_points)
th4g=np.zeros(num_points)
th5g=np.zeros(num_points)
th6g=np.zeros(num_points)

th1p=np.zeros(num_points)
th2p=np.zeros(num_points)
th3p=np.zeros(num_points)
th4p=np.zeros(num_points)
th5p=np.zeros(num_points)
th6p=np.zeros(num_points)

time=np.zeros(num_points)

x0g = np.zeros(num_points)
y0g = np.zeros(num_points)
z0g = np.zeros(num_points)

x1g = np.zeros(num_points)
y1g = np.zeros(num_points)
z1g = np.zeros(num_points)

x2g = np.zeros(num_points)
y2g = np.zeros(num_points)
z2g = np.zeros(num_points)

x3g = np.zeros(num_points)
y3g = np.zeros(num_points)
z3g = np.zeros(num_points)

x4g = np.zeros(num_points)
y4g = np.zeros(num_points)
z4g = np.zeros(num_points)

x5g = np.zeros(num_points)
y5g = np.zeros(num_points)
z5g = np.zeros(num_points)

x6g = np.zeros(num_points)
y6g = np.zeros(num_points)
z6g = np.zeros(num_points)

xhg = np.zeros(num_points)
yhg = np.zeros(num_points)
zhg = np.zeros(num_points)

xdg = np.zeros(num_points)
ydg = np.zeros(num_points)
zdg = np.zeros(num_points)

xdpg = np.zeros(num_points)
ydpg = np.zeros(num_points)
zdpg = np.zeros(num_points)

tg = np.linspace(0,T,num_points)

# Parametros del movimiento del robot
e1 = 1
e2 = 1 
e4 = 1

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
    vph_t = np.array([xdp, ydp, zdp])
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

    # Calculo de las variables articulares
    th1 = atan2(e1 * PY, e1 * PX)

    z1 = -d2 + PX * cos(th1) + PY * sin(th1)
    b1 = 2 * (-(d4 * PZ) - r4 * z1)
    b2 = 2 * (PZ * r4 - d4 * z1)
    b3 = d3**2 - d4**2 - PZ**2 - r4**2 - z1**2

    SQ = (b1 * b3 + b2 * sqrt(b1**2 + b2**2 - b3**2) * e2) / (b1**2 + b2**2)
    CQ = (b2 * b3 - b1 * sqrt(b1**2 + b2**2 - b3**2) * e2) / (b1**2 + b2**2)

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

    # Definición de matrices de transformación
    T01 = np.array([
        [cos(th1), -sin(th1), 0, 0],
        [sin(th1), cos(th1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T12 = np.array([
        [cos(th2), -sin(th2), 0, d2],
        [0, 0, -1, 0],
        [sin(th2), cos(th2), 0, 0],
        [0, 0, 0, 1]
    ])

    T23 = np.array([
        [cos(th3), -sin(th3), 0, d3],
        [sin(th3), cos(th3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T34 = np.array([
        [cos(th4), -sin(th4), 0, d4],
        [0, 0, -1, -r4],
        [sin(th4), cos(th4), 0, 0],
        [0, 0, 0, 1]
    ])

    T45 = np.array([
        [cos(th5), -sin(th5), 0, 0],
        [0, 0, 1, 0],
        [-sin(th5), -cos(th5), 0, 0],
        [0, 0, 0, 1]
    ])

    T56 = np.array([
        [cos(th6), -sin(th6), 0, 0],
        [0, 0, -1, 0],
        [sin(th6), cos(th6), 0, 0],
        [0, 0, 0, 1]
    ])

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
    th1g[i+1] = degrees(th1)
    th2g[i+1] = degrees(th2)
    th3g[i+1] = degrees(th3)
    th4g[i+1] = degrees(th4)
    th5g[i+1] = degrees(th5)
    th6g[i+1] = degrees(th6)
    
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


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-1000, 1000)
ax.set_ylim(-1000, 1000)
ax.set_zlim(-1000, 1000)

# Función para actualizar la animación
def update(num, x, y, z):
    ax.cla()
    ax.plot(x[:num], y[:num], z[:num], 'r-')
    ax.scatter(x[num], y[num], z[num], color='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-1000, 1000)
    ax.set_ylim(-1000, 1000)
    ax.set_zlim(-1000, 1000)

# Animación
ani = plt.FuncAnimation(fig, update, frames=num_points, fargs=(xdg, ydg, zdg), interval=50)
plt.show()


# # Crear la figura 3D
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # Graficar la trayectoria en 3D
# ax.plot3D(xdg, ydg, zdg, label='Trayectoria deseada', color='blue')

# # Etiquetas de los ejes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Título de la gráfica
# ax.set_title('Trayectorias en 3D')

# # Mostrar la leyenda
# ax.legend()

# # Mostrar la gráfica
# plt.show()

# # Gráficas en función del tiempo
# plt.figure(figsize=(8, 6))
# plt.plot(tg, xdg, label='xd')
# plt.xlabel('t')
# plt.ylabel('xd')
# plt.title('xd vs t')
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.plot(tg, ydg, label='yd')
# plt.xlabel('t')
# plt.ylabel('yd')
# plt.title('yd vs t')
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.plot(tg, zdg, label='zd')
# plt.xlabel('t')
# plt.ylabel('zd')
# plt.title('zd vs t')
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.plot(tg, xdpg, label='xdp')
# plt.xlabel('t')
# plt.ylabel('xdp')
# plt.title('xdp vs t')
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.plot(tg, ydpg, label='ydp')
# plt.xlabel('t')
# plt.ylabel('ydp')
# plt.title('ydp vs t')
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.plot(tg, zdpg, label='zdp')
# plt.xlabel('t')
# plt.ylabel('zdp')
# plt.title('zdp vs t')
# plt.legend()
# plt.grid(True)
# plt.show()