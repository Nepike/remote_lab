#!/usr/bin/env python3
# coding: utf-8

"""
  Agent module
  Author: Valery Karpov

  06.02.2015, 26.02.2017, 31.08.2020, 31.12.2022, 23.03.2024, 01.10.2024
  Version 2.3
  LP 12.01.2025

"""

import sys, random, math
from accessify import protected, private
from kvorum2 import gdic, geometry

################################################################################
#
# Агент
#
################################################################################
NUM_LOCATOR = 50

class TAgent(object):
    def __init__(self, cid, cpos, cshape, cenv, show_id = False, csize = 0):
        # csize - размер объекта: 0 - по умолчанию, -1 - по размеру сетки, иначе - по заданному значению
        # cshape - имя формы для отображения
        if cid<=0: gdic.error(f"TAgent: illegal agent`s id {cid}, id must be >0")

        self.env = cenv
        self.State = gdic.STAT_NORM  # Состояние агента
        self.id = cid                # Имя (идентификатор агента)
        self.alive = True            # Флаг того, что агент жив
        self.pos = cpos              # Местопроложение (координаты и направление) (x, y, dir)
        self.shape = cshape
        self.size = csize
        self.show_id = show_id

        self._curr_movespeed = 0     # Текущие скорости движения и разворота
        self._curr_rotatespeed = 0

        self._norm_movespeed = 1     # Крейсерские скорости движения и разворота
        self._norm_rotatespeed = 5

        self.traceOn = False         # Флаг режима рисования трассы

        self.DrawSensors = False

        # Сенсоры - значения датчиков
        self.__sensors_list = []            # Сенсоры - массив датчиков (объектов)
        self.SENSORS = {}

        # "След" агента. То, что он оставляет на поле
        self.Trail = dict({ gdic.LEVEL_GROUND: 0, gdic.LEVEL_COLOR: 0, gdic.LEVEL_LIGHT: 0, gdic.LEVEL_IR: 0})

        # Здесь сохраняются атрибуты точки поля
        self.__saved_trail = dict({ gdic.LEVEL_GROUND: 0, gdic.LEVEL_COLOR: 0, gdic.LEVEL_LIGHT: 0, gdic.LEVEL_IR: 0})

        # Агент оставляет след как минимум на слое gdic.LEVEL_GROUND
        self.SetSrc(gdic.LEVEL_GROUND, 1)

        # cname, encoder_side, wheel_radius, resolution, wheel_distance
        self.AddSens(TEncoder("ENCODER_LEFT",  -1, 10, 10, 10))
        self.AddSens(TEncoder("ENCODER_RIGHT",  1, 10, 10, 10))
        s = TRegister("POSITION")
        self.AddSens(s)
        s.SetVal(cpos)

        self.SENSORS["ENCODER_LEFT"] = [0]
        self.SENSORS["ENCODER_RIGHT"] = [0]
        self.SENSORS["POSITION"] = [0,0,0]

    #
    # Агент становится источником сигнала value на уровне level
    #
    def SetSrc(self, level, value): self.Trail[level] = value

    def GetSrc(self, level): return self.Trail[level]

    def SetState(self, st): self.State = st  # Состояние агента

    @protected
    def __clear_trail(self, x, y):
        x, y = int(x), int(y)
        for v in self.__saved_trail.items():
            level, val = v
            # Восстанавливаем значения атрибутов поля
            self.env.Field[y][x][level] = val

    @protected
    def __set_trail(self, x, y):
        x, y = int(x), int(y)
        for v in self.Trail.items():
            level, val = v
            # Сохраняем значения атрибутов поля
            self.__saved_trail[level] = self.env.Field[y][x][level]
            self.env.Field[y][x][level] = val

    #
    # Перемещение в точку newpos
    #
    def MoveTo(self, newpos):
        x, y, a = newpos[0], newpos[1], newpos[2] % 360
        if not self.alive: return
        (oldx, oldy) = int(self.GetX()), int(self.GetY())
        (x, y) = self.env.normalizate_xy(x, y)

        SameCell = (oldx == int(x)) and (oldy == int(y))

        if(self.env.Field[int(y)][int(x)][gdic.LEVEL_GROUND] == 0 or SameCell):
            self.__clear_trail(oldx, oldy)
            self.__set_trail(x, y)
            self.pos = [x, y, a]
            return True
        else:   # только разворот
            self.pos[2] = a
            return False

    def SetAng(self, ang):
        if(not self.alive): return
        self.pos[2] = ang % 360

    def Turn2(self, ang):
        if(not self.alive): return
        self.pos[2] = (self.pos[2] + ang) % 360

    #
    # Позиция робота в модельной среде
    #
    def GetX(self): return self.pos[0]
    def GetY(self): return self.pos[1]
    def GetAng(self): return self.pos[2] % 360

    # Установка текущих скоростей
    def SetCurr2WSpeed(self, left_speed, right_speed):
        """ Sets speed for both left and right wheel.

        NOTE: makes sense only for some types of robots, especially suitable for
        two-wheeled ones. This is a limitation imposed by the rcX protocol. See
        CMD_SET_SPEED in arduino code, etc.
        """
        # TODO: constants for YARP4, should be moved elsewhere in code
        WHEEL_DISTANCE = 48. # cm

        self._curr_movespeed = (right_speed + left_speed) / 2
        self._curr_rotatespeed = (right_speed - left_speed) / WHEEL_DISTANCE
    #
    # Управление движением
    #
    # Текущие скорости движения и разворота
    def SetCurrVSpeed(self, mspeed, rspeed):
        if(not self.alive): return
        self._curr_movespeed = mspeed
        self._curr_rotatespeed = rspeed
    # Крейсерские скорости движения и разворота
    def SetNormVSpeed(self, mspeed, rspeed):
        if(not self.alive): return
        self._norm_movespeed = mspeed
        self._norm_rotatespeed = rspeed

    def GetNormVSpeed(self): return self._norm_movespeed, self._norm_rotatespeed
    def GetCurrVSpeed(self): return self._curr_movespeed, self._curr_rotatespeed 

    #
    # Удаление агента
    #
    def Delete(self):
        self.traceOn = False
        self.__clear_trail(self.GetX(), self.GetY())
        # Позиция на экране (кладбище)
        self.pos = (-1+self.id, -10, 90)
        self.alive = False

    #
    # Шаг
    #
    def Step(self):
        if(not self.alive): return
        x, y, a = self.GetX(), self.GetY(), self.GetAng()
        nx = x + self._curr_movespeed*geometry.FastCos(a)*gdic.Delta_T
        ny = y + self._curr_movespeed*geometry.FastSin(a)*gdic.Delta_T
        na = a + self._curr_rotatespeed*gdic.Delta_T
        res = self.MoveTo([nx, ny, na])

        s = self.FindSensor('POSITION')
        s.SetVal([int(self.GetX()), int(self.GetY()), int(self.GetAng())])

        # process encoders, if any
        '''
        s = self.FindSensor('ENCODER_LEFT')
        s.Update(self._curr_movespeed, self._curr_rotatespeed, gdic.Delta_T)

        s = self.FindSensor('ENCODER_RIGHT')
        s.Update(self._curr_movespeed, self._curr_rotatespeed, gdic.Delta_T)
        '''
        # Остальные сенсоры будем читать по запросу

        return res

    #
    # Работа с сенсорами
    #
    def AddSens(self, sens):
        self.SENSORS[sens.name.upper()] = sens.GetSData(self)
        self.__sensors_list.append(sens)
        return sens

    def GetSensor(self, sensname):
        return self.SENSORS[sensname.upper()]

    def SetSensorVal(self, sensname, data):
        sensname = sensname.upper()
        if self.FindSensor(sensname) is None:
            gdic.error(f"TAgent.SetSensorVal error: sensor {sensname} not found")
        if type(data)!=list: data = [data]
        self.SENSORS[sensname] = data

    def FindSensor(self, sensname):
        sensname = sensname.upper()
        for s in self.__sensors_list:
            if s.name.upper()==sensname: return s
        return None

    def DeleteSensor(self, sensname):
        sensname=sensname.upper()
        s = self.FindSensor(sensname)
        self.__sensors_list.remove(s)
        del self.SENSORS[sensname]

    #@property
    def GetSensorsList(self): return self.__sensors_list
    #
    # Опрос всех сенсоров и заполнение словаря
    #
    def ReadSensors(self, senslist):
        for sname in senslist:
            sname = sname.upper()
            s = self.FindSensor(sname)
            val = s.GetSData(self)
            if type(val)!=list: val = [val]
            self.SENSORS[s.name] = val
        return

################################################################################
#
# Сенсор
#
################################################################################
CContext = { "agent": None, "level": 0}

class TSensor(object):
    def __init__(self, cname, cdir, cang, cR, cLevel, ctip, cvaltype = gdic.RST_SCALAR, cfproc = None, cval = 0):
        self.name = cname.upper() # Имя
        self.dir = cdir      # Направление относительно робота (x, y, dir)
        self.ang = cang      # Угол раствора
        self.R = cR          # Радиус действия. Если R=0, то это - точечный датчик
        self.Level = cLevel  # Рабочий слой сенсора
        self.tip = ctip      # Тип датчика: gdic.ST_SHARP, gdic.ST_USONIC, gdic.ST_DETECTOR, gdic.ST_CONST
        self.valtype = cvaltype # Тип результата
        self.fproc = cfproc  # Функция постобработки сигнала (или None)
        self.cvalue = cval   # Значение датчика для типа ST_CONST

    def GetSData(self, agent):
        # Константная величина
        if(self.tip==gdic.ST_CONST):
            return self.cvalue
        val = 0
        if(self.valtype==gdic.RST_SCALAR):
            val = self.GetScalar(agent)
        elif(self.valtype==gdic.RST_VECTOR):
            val = self.GetVector(agent, True)
        elif(self.valtype==gdic.RST_SUPER_VECTOR):
            val = self.GetVector(agent, False)
        else: gdic.error("TSensor.GetSData: unknown valtype")
        if type(val)!=list: val = [val]
        return val

    def nang(self, a1, a2):
        a1 = int(a1) % 360
        a2 = int(a2) % 360
        if(a1<0): a1 = 360+a1;
        if(a2<0): a2 = 360+a2;

        if(a1>a2): a1, a2 = a2, a1

        # Особое расположение
        if(a1<=90 and a2>=270): # Опять меняем
            a1, a2 = a2, a1
        return (a1, a2)

    def EvalResult(self, e, res, isScalar):
        rv, val = 0, 0
        if(e): # что-то найдено
            if(res == None): gdic.error("*** EvalResult")
            else: (dist, val) = res
            if self.tip==gdic.ST_SHARP: rv = self.R - dist
            elif self.tip==gdic.ST_USONIC: rv = dist
            elif self.tip==gdic.ST_DETECTOR: rv = val
        if not (self.fproc is None): rv = self.fproc(svalue)
        if isScalar: svalue = rv
        else: svalue = (rv, val)
        return svalue

    def GetScalar(self, agent):
        # Координаты сенсора и углы обзора
        sx, sy, sa = agent.GetX(), agent.GetY(), self.dir + agent.GetAng()
        a1, a2 = self.nang(sa-self.ang/2, sa+self.ang/2)

        global CContext
        CContext["level"] = self.Level
        CContext["agent"] = agent
        if(self.R == 0): # Точечный датчик
            # Датчик расположен со смещением
            sx = int(sx)
            sy = int(sy)
            for x in range(sx-1, sx+1+1):
                for y in range(sy-1, sy+1+1):
                    if(x!=sx and y!=sy):
                        obj = agent.env.GetObject(x, y)
                        value = obj[self.Level]
                        if(value!=0): return value
            return 0
        # Датчик секторного типа
        e, res = geometry.BresFillArc(sx, sy, a1, a2, self.R, SPlot)
        value = self.EvalResult(e, res, True)
        return value

    def GetVector(self, agent, isScalar):
        # Координаты сенсора и углы обзора
        sx, sy, sa = agent.GetX(), agent.GetY(), self.dir + agent.GetAng()
        a1, a2 = sa-self.ang/2, sa+self.ang/2

        global CContext
        CContext["level"] = self.Level
        CContext["agent"] = agent

        # Вычисляем шаг, зависящий от типа локатора
        if(isScalar):
            step = float(self.ang)/NUM_LOCATOR
            if(step<1): # Угол меньше, чем максимальное количество элементов
                avnum = int(self.ang)
                step = 1
            else:
                avnum = NUM_LOCATOR
        else:
            step = 1
            avnum = self.ang # 360

        v = []
        a = a1
        if(a1<a2): # норма
            while (a<a2 and len(v)<avnum):
                aa = int(a)
                e, res = geometry.BresLine(int(sx), int(sy), int(a), self.R, SPlot)
                value = self.EvalResult(e, res, isScalar)
                v.append(value)
                a+=step
        else: # Переход через 0
            while (a<360 and len(v)<avnum):
                e, res = geometry.BresLine(int(sx), int(sy), a, self.R, SPlot)
                a+=step
                value = self.EvalResult(e, res, isScalar)
                v.append(value)
            a = 0
            while (a<a2 and len(v)<avnum):
                e, res = geometry.BresLine(int(sx), int(sy), a, self.R, SPlot)
                a+=step
                value = self.EvalResult(e, res, isScalar)
                v.append(value)
        return v

#
# Класс энкодера, придерживающийся интерфейса общего датчика TSensor, однако
# добавляющего нужные параметры и автозаполняющий ненужные.
# Предполагается двухколесный робот.
# TODO: proper min / max for encoder values and 2-byte representation
# encoder_side : для энкодеров < 0 - левый, > 0 - правый
#
class TEncoder(TSensor) :
    def __init__(self, cname, encoder_side,
            wheel_radius, resolution, wheel_distance):
        super(TEncoder, self).__init__(cname, 0, 0, 0, 0, ctip = gdic.ST_DETECTOR, cvaltype = gdic.RST_SCALAR)

        self.wheel_radius = wheel_radius
        self.resolution = resolution # ticks per turn
        self.wheel_distance = wheel_distance # distance between wheels

        self.encoder_side = encoder_side # для энкодеров < 0 лево, > 0 право

        self.ticks = 0 # ticks registered by the encoder

        self.TICKS_MAX = 255
        self.TICKS_MIN = 0

    def Update(self, fwd_speed, rot_speed, dt) :
        """
          fwd_speed : direct movement, in cm / s
          rot_speed : rotation speed in place, in degrees / s
          dt : time passed since last step, in s
        """
        if (self.encoder_side < 0):
            # left encoder
            self.ticks += ( fwd_speed * dt / (2*math.pi*self.wheel_radius) ) * self.resolution
            self.ticks += ( ( (rot_speed * dt / 360.) * math.pi
                    * self.wheel_distance ) / (2*math.pi*self.wheel_radius) ) * self.resolution
        else:
            # right encoder
            self.ticks += ( fwd_speed * dt / (2*math.pi*self.wheel_radius) ) * self.resolution
            self.ticks -= ( ( (rot_speed * dt / 360.) * math.pi
                    * self.wheel_distance ) / (2*math.pi*self.wheel_radius) ) * self.resolution

        if (self.ticks > self.TICKS_MAX): self.ticks = self.TICKS_MIN

    def SetVal(self, ct): self.ticks = ct
    def GetSData(self, agent): return int(self.ticks)

#
# Класс внутреннего датчика, придерживающийся интерфейса общего датчика TSensor
#
class TRegister(TSensor):
    def __init__(self, cname):
        super(TRegister, self).__init__(cname, 0, 0, 0, 0, ctip = gdic.ST_DETECTOR, cvaltype = gdic.RST_VECTOR)
        self.__data = []
    def SetVal(self, ndata): self.__data = ndata
    def GetSData(self, agent): return self.__data

################################################################################
#
# Чтение/рисование
#
################################################################################

def SPlot(x, y):
    global CContext

    agent = CContext["agent"]
    env = agent.env
    level = CContext["level"]
    myx = int(agent.pos[0])
    myy = int(agent.pos[1])
    x = int(x)
    y = int(y)
    if(myx==x and myy==y): return (False, None)

    # Анализ точки x, y
    obj = env.GetObject(x, y)

    p = obj[level]
    if(p!=0): return (True, p)

    return (False, None)
