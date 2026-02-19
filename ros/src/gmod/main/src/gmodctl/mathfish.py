#!/usr/bin/env python3
# coding: utf-8
'''
  Математика для поведенческих процедур
  V 1.02
  26.03.2022
  LP 15.02.2024
'''
import math, random
import copy
import numpy as np

#################################################################################
#
# Вспомогательные математические функции
#
#################################################################################

def sign(x):
    if x>0: return 1
    if x<0: return -1
    return 0

# Параметры поворота от угла a1 до a2
# Определяются знак поворота и угол
def GetTurnAng(a1, a2):
    if a1<0: a1 += 360
    a1 %= 360
    if a2<0: a2 += 360
    a2 %= 360

    if (a1==a2): return 0, 0
    a = a2 - a1
    if a<0: a += 360
    a %= 360
    dang = abs(a1-a2)
    if dang>180: dang = 360-dang
    rdir = -1 if a>180 else 1

    return rdir, dang

# Перевод сферических координат в декартовы
# phi = a[0], theta = a[1] - градусы
def Sph2Dec(r, a):
    yaw = math.radians(a[0])
    pitch = math.radians(a[1])

    x = r*np.cos(yaw)*np.cos(pitch)
    y = r*np.sin(yaw)*np.cos(pitch)
    z = r*np.sin(pitch)

    return x, y, z

# Перевод декартовых координат в сферические
# Возвращает длину и ориентацию вектора, углы в градусах
def Dec2Sph(x, y ,z):
    r = np.linalg.norm([x,y,z])
    theta = math.degrees(math.atan2(z, math.hypot(x, y)))
    phi = math.degrees(math.atan2(y, x))

    if phi<0: phi += 360
    phi %= 360

    if theta<0: theta +=360
    theta %= 360

    return r, (phi, theta, 0)

# Перевод декартовых координат в сферические
# Coord = [x, y ,z]
# Возвращает только ориентацию вектора (углы в градусах)
def Dec2SphCoord(coord):
    x, y, z = coord[0], coord[1], coord[2]
    theta = math.degrees(math.atan2(z, math.hypot(x, y)))
    phi = math.degrees(math.atan2(y, x))

    if phi<0: phi += 360
    phi %= 360

    if theta<0: theta +=360
    theta %= 360

    return (phi, theta, 0)

# Сумма векторов, заданных в сферических координатах
# Возвращает длину вектора и углы в градусах
def SumVect(r1, a1, r2, a2):
    x1, y1, z1 = Sph2Dec(r1, a1)
    x2, y2, z2 = Sph2Dec(r2, a2)

    r, a = Dec2Sph(x1+x2, y1+y2, z1+z2)
    return r, a

''' 
Определение пересечения прямой и сферы
Прямая задается точками A и B
Сфера - центром C и радиус R
Возвращает координаты точек пересечения

1. Параметрическое уравнение прямой:
    X = A + t*(B-A)                 (выр.1)
2. Уравнение поверхности сферы:
   (X-C)^2 - R^2 = 0
3. Решаем уравнение at^2+bt+c=0
   где a=(B-A)^2, b=2*((B-A),(A-C)), c=(A-C)^2-R^2

    D=b^2-4ac
    Если D<0, то возврат с неудачей
    t1 = (-b + sqrt(D))/2a
    t2 = (-b - sqrt(D))/2a
4. Подставляем t1, t2 в (выр.1) и получаем решения X1, X2
'''
def LineSphIntersection(A, B, C, R):
    a = np.dot(B-A, B-A)
    b = 2*np.dot(B-A, A-C)
    c = np.dot(A-C, A-C)-R**2
    D = b*b - 4*(a*c)
    if D<0: return None, None

    t1 = (-b + math.sqrt(D))/(2*a)
    t2 = (-b - math.sqrt(D))/(2*a)

    X1 = A + t1*(B-A)
    X2 = A + t2*(B-A)
    return X1, X2

# Определение принадлежности точки X отрезку [A,B]
def PIsIn(A,B,X):
    for i in range(len(X)):
        low, hi = A[i], B[i]
        if low>hi: low, hi = hi, low
        if X[i]<low or X[i]>hi: return False
    return True 

'''
Определение пересечения отрезка и сферы
Отрезок задается точками A и B
Сфера - центром C и радиус R
Возвращает признак пересечения и координаты точек пересечения
'''
def LSegmentSphIntersection(A, B, C, R):
    X1, X2 = LineSphIntersection(A, B, C, R)
    if X1 is None: return False, None, None
    # Проверяем принадлежность точек X1,X2 отрезку A,B
    f1 = PIsIn(A,B,X1)
    f2 = PIsIn(A,B,X2)
    if not f1: X1 = None
    if not f2: X2 = None
    return f1 or f2, X1, X2

'''
Пересечение прямой с плоскостью
  A,B - точки, через которые проходит прямая
  Уравнение прямой: X=A+t(B-A)
  P - точка, через которую проходит плоскость, N - вектор нормали к плоскости
  Уравнение плоскости: <N;X-P>=0
  <;> - скалярное произведение

     <N;(X-X0)> = 0
     <N;X> = <N;X0>

     <N;A+t(B-A)> = <N;X0>
     <N;A> + t<N;(B-A)> = <N;X0>
     t = (<N;X0> - <N;A>)/<N;(B-A)>
     Xres = X(t)
'''
def LinePlaneIntersection(A, B, X0, N):
    D = np.dot(N, (B-A))
    if D==0: return None
    t = (np.dot(N, X0) - np.dot(N, A))/D
    X = A + t*(B-A)
    return X

'''
Определение пересечения отрезка и плоскости
Отрезок задается точками A и B
Плоскость - точкой P и вектором нормали N
Возвращает признак пересечения и координату точки пересечения
'''
def LSegmentPlaneIntersection(A, B, P, N):
    X = LinePlaneIntersection(A, B, P, N)
    if X is None: return False, None
    # Проверяем принадлежность точки X отрезку A,B
    f = PIsIn(A,B,X)
    if not f: X = None    
    return f, X


#################################################################################
#
# Задача преследования. Параллельное сближение
#
#################################################################################
#
# x1, y1 - координаты жертвы
# V1 - скорость движения жертвы
# angle1 - ориентация жертвы (в градусах)
#
# x2, y2 - координаты преследователя
# V2 - скорость движения преследователя
# Возвращает угол - направление движения преследователя (в градусах)
#
def GetParallelRendezvousAngle(x1, y1, V1, angle1, x2, y2, V2):
    dx = x1 - x2
    dy = y1 - y2
    gamma = math.degrees(math.atan2(dx, dy))
    angle2 = GetParallelRendezvousAngle2(gamma, V1, angle1, V2)
    return angle2

#
# gamma - угол направления от преследователя к жертве
# V1 - скорость движения жертвы
# angle1 - ориентация жертвы  (в градусах)
#
# V2 - скорость движения преследователя
# Возвращает угол - направление движения преследователя
#
def GetParallelRendezvousAngle2(gamma, V1, angle1, V2):
    angle1 = math.radians(angle1)
    gamma = math.radians(gamma)

    angle11 = angle1 + gamma

    v1x = V1 * math.cos(angle11)
    v1y = V1 * math.sin(angle11)

    v2x = v1x
    try:
        v2y = math.sqrt(V2**2 - v2x**2)
    except:
        print("V1 = ", V1, "V2 = ", V2, "v1x = ", v1x, "v2x = ", v2x)
        sys.exit(1)

    angle2_tmp = math.atan2(v2y, v2x)

    angle2 = math.degrees(angle2_tmp - gamma)
    if angle2<0: angle2 += 360
    angle2 %= 360

    return angle2
