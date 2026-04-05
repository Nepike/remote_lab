#!/usr/bin/env python3
# coding: utf-8
"""
  tmu-base.py
  Модель робота TMU
  Author: Valery Karpov

  06.02.2015
  Version 3.5
  15.07.2021, 21.01.2022, 23.03.2024, 01.10.2024
  LP 14.06.2025

"""
import sys, time
import roslib, rospy

from kvorum2.env import TEnv
from kvorum2.agent import TAgent, TSensor

# Топики ROS
from msg_kvorum2.msg import action
from msg_kvorum2.msg import sdata, sensors

from kvorum2 import gdic

Robots = []

G_pub_cmd = None

# Процедуры: примитивы движения
(
  PROC_STOP,
  PROC_GOFWD,
  PROC_GOBACK,
  PROC_GOLEFT,   # У этой команды может быть аргумент. Он воспринимается, как поворот на заданный угол.
  PROC_GORIGHT,  # Аналогично
  PROC_STEPFWD,
  PROC_STEPBACK,
  PROC_STEPLEFT,
  PROC_STEPRIGHT,
  PROC_BEEP,
  PROC_BEEP_START,
  PROC_BEEP_STOP,
  PROC_KILL,
  PROC_SET_STATE,
  PROC_LAST,
  PROC_EAT,      # Поедание пищи. Указывается уровень. Удаляются все объекты в месте нахождения агента (x-2, x+2), (y-2, y+2)
  PROC_BIRTH,    # Порождение объекта в месте нахождения агента. Аргумент - (level, value)
  PROC_NONE
) = range(18)

# Процедуры
( PROC_SEARCH_FOOD,       # Поиск пищи
  PROC_ESCAPE,            # Убегание от препятствия
  PROC_SEARCH_SHADOW,     # Поиск тени
  PROC_SLEEP,             # Сон
  PROC_WALK,              # Свободное блуждание
  PROC_MOVE_TO_OBSTACLE,  # Движение к препятствию
  PROC_ONE_STEP,
  PROC_REFLEX,
) = range(PROC_NONE+1, PROC_NONE+1+8)

ProcNames = { PROC_SEARCH_FOOD:      "S.FOOD",
              PROC_EAT:              "EAT",
              PROC_BIRTH:            "BIRTH",
              PROC_ESCAPE:           "ESCAPE",
              PROC_SEARCH_SHADOW:    "S.SHADOW",
              PROC_SLEEP:            "SLEEP",
              PROC_WALK:             "WALK",
              PROC_MOVE_TO_OBSTACLE: "MOVE TO OBSTACLE",
              PROC_ONE_STEP:         "ONE STEP",
              PROC_REFLEX:           "REFLEX",
              PROC_NONE:             "None"}

#
# Основной класс - робот TMU
#
class TRobot():
    def __init__(self, ca):
        self.agent = ca
        self.id = self.agent.id

        self.CurrentAction = None            # Текущее движение
        self.CurrentActionArgument = None    # Аргумент текущего движения
        self.reflex_level = 2 # Уровень рефлекса (0, 1). 2 - это уже не рефлекс.
        #
        # Управляющий автомат, связанный с роботом
        #
        self.curr_fsm = None # Текущий автомат
        self.pred_fsm = None # Предыдущий автомат
        self.FSMList = []    # Список автоматов

        # Признак ожидания ответа на запрос сенсоров
        self.waits_for_sensors = False

        # Сообщаем всем о своей крейсерской скорости
        self.SetNormVSpeed(ca._norm_movespeed, ca._norm_rotatespeed)

    def GetSensor(self, sensname):
        data = self.agent.GetSensor(sensname)
        if len(data)==1: return data[0]
        return data

    def SetSensorVal(self, sensname, data):
        self.agent.SetSensorVal(sensname, data)

    #
    # Прием значения сенсора
    #
    def accept_sens_data(self, msg):
        if(not self.agent.alive): return
        sensname = msg.args
        data = msg.data
        s = self.agent.FindSensor(sensname)
        if not s: gdic.error(f"tmubase: Sensor \'{sensname}\' not found")
        if s.valtype==gdic.RST_SUPER_VECTOR:
            data = gdic.Vector2SuperVector(msg.data)
        self.agent.SENSORS[sensname] = list(data)
        return

    def WaitForSensorsData(self):
        if self.waits_for_sensors: return True
        self.waits_for_sensors = True
        return False

    #
    # Запрос сенсоров из заданного списка
    # Если sensorname == None, то запрос всех сенсоров
    #
    def RequestSensors(self, sensors_list=None):
        if(not self.agent.alive): return
        # Формируем список запрашиваемых сенсоров
        if sensors_list is None:
            sens_names = list(self.agent.SENSORS.keys())
        else:
            sens_names = sensors_list
        sens_num = len(sens_names)
        self.__publish_cmd(gdic.CMD_GET_SENS, slist=sens_names)
        #self.waits_for_sensors = True

    #
    # Действия
    #

    # Движение с крейсерской скоростью
    def Stop(self): self.__publish_cmd(gdic.CMD_STOP)
    def GoFwd(self): self.__publish_cmd(gdic.CMD_FWD)
    def GoBack(self): self.__publish_cmd(gdic.CMD_BACK)
    def GoFastLeft(self, ang=0): self.__publish_cmd(gdic.CMD_FAST_LEFT, slist=[], arg1=ang)
    def GoFastRight(self, ang=0): self.__publish_cmd(gdic.CMD_FAST_RIGHT, slist=[], arg1=ang)
    def GoLeft(self, ang=0): self.GoFastLeft(ang)
    def GoRight(self, ang=0): self.GoFastRight(ang)

    def StepFwd(self, n): return
    def StepBack(self, n): return
    def StepLeft(self, n): return
    def StepRight(self, n): return

    # Задание текушей скорости движения:
    #   vlin - линейная скорость
    #   vang - угловая скорость (градусы)
    def SetCurrVSpeed(self, vlin, vang):
        self.agent.SetCurrVSpeed(vlin, vang)
        self.__publish_cmd(gdic.CMD_SET_CURR_V_SPEED, arg1=vlin, arg2=vang)

    # Задание крейсерской скорости движения:
    #   vlin - линейная скорость
    #   vang - угловая скорость (градусы)
    def SetNormVSpeed(self, vlin, vang):
        self.agent.SetNormVSpeed(vlin, vang)
        self.__publish_cmd(gdic.CMD_SET_NORM_V_SPEED, arg1=vlin, arg2=vang)

    def GetNormVSpeed(self): return self.agent.GetNormVSpeed()
    def GetCurrVSpeed(self): return self.agent.GetCurrVSpeed()

    # Удалить объекты на слое level в окрестностях координат агента
    def ProcEat(self, level): self.__publish_cmd(gdic.CMD_DELETE_OBJ, slist=[], arg1=level)

    # Создать объекты со значением val на слое level в окрестностях координат агента
    # level_val = (level, val)
    # CMD_F_SET_FIELD: Установить значение поля. Аргументы: id=value, arg1=x, arg2=y, arg3=level    
    def ProcBirth(self, level_val):
        level, val = level_val[0], level_val[1]
        pos = self.GetSensor("POSITION")
        x, y = pos[0], pos[1]
        PublishCmd(gdic.CMD_F_SET_FIELD, val, slist=[], arg1 = x, arg2 = y, arg3 = level)

    # Агент становится источником сигнала value на уровне level
    def SetSrc(self, level, val):
        self.agent.SetSrc(level, val)
        self.__publish_cmd(gdic.CMD_F_SET_A_SRC, arg1=level, arg2=val)

    # Получить значение сигнала агента (своего кода) на уровне level
    def GetSrc(self, level): return self.agent.GetSrc(level)

    # Установить статус агента
    def SetState(self, stat):
        self.__publish_cmd(gdic.CMD_F_SET_STATE, arg1=stat)
        self.agent.SetState(stat)

    #
    # Сообщение выходного топика
    #
    def __publish_cmd(self, cmd: int, slist=[], arg1 = 0, arg2 = 0, arg3 = 0):
        PublishCmd(cmd, self.agent.id, slist, arg1, arg2, arg3)
    #
    # Выполнение действия act с аргументом arg
    #
    def Make(self, act: int, arg1 = 0):
        #if(not self.agent.alive): return
        self.CurrentAction = act
        self.CurrentActionArgument = arg1

        if(act==PROC_STOP): self.Stop()
        elif(act==PROC_GOFWD): self.GoFwd()
        elif(act==PROC_GOBACK): self.GoBack()
        elif(act==PROC_GOLEFT): self.GoFastLeft(arg1)
        elif(act==PROC_GORIGHT): self.GoFastRight(arg1)
        elif(act==PROC_STEPFWD): self.StepFwd(1)
        elif(act==PROC_STEPBACK): self.StepBack(1)
        elif(act==PROC_STEPLEFT): self.StepLeft(1)
        elif(act==PROC_STEPRIGHT): self.StepRight(1)
        elif(act==PROC_KILL): KillAgent(arg1)
        elif(act==PROC_EAT): self.ProcEat(arg1)
        elif(act==PROC_BIRTH): self.ProcBirth(arg1)
        elif(act==PROC_SET_STATE): self.SetState(arg1)
        elif(act==PROC_BEEP_START): self.__publish_cmd(gdic.CMD_BEEP_ON)
        elif(act==PROC_BEEP_STOP): self.__publish_cmd(gdic.CMD_BEEP_OFF)
        return

    def EvaluateSensorsInfo(self):
        return

    def ShowStatus(self, sensors_list=None):
        if sensors_list is None:
            keys=self.agent.SENSORS.keys()
        else:
            keys = sensors_list
        if len(keys)==0: return
        print(f"Agent {self.agent.id} Sensors: ", end='')
        for k in keys:
            print(f"{k.upper()}: {self.agent.SENSORS[k.upper()]} ", end='')
        print()

    #
    # Работа с автоматами
    #
    def FindFSM(self, name):
        for f in self.FSMList:
            if f.Name==name:
                return f
        gdic.error("FindFSM: "+name+" not found")

    def LoadFSM(self, name, arg = None):
        self.curr_fsm = self.FindFSM(name)
        self.curr_fsm.reset()
        self.curr_fsm.arg = arg
        return self.curr_fsm

################################################################################
#
# Вспомогательные функции
#
################################################################################
#
# Системные функции
#

def RFind(id):
    for r in Robots:
        if(r.agent.id == id):
            return r
    gdic.error("RFind: agent "+str(id) + " not found")

#
# Прием сообщения sensors.msg
#
def sensors_callback(msg):
    if(msg.dest==0): return
    r = RFind(msg.dest)
    for m in msg.data:
        r.accept_sens_data(m)
    r.waits_for_sensors = False
    return

#
#
#
def PublishCmd(act: int, aid: int, slist=[], arg1 = 0, arg2 = 0, arg3 = 0):
    msg = action()
    msg.team_id = 1
    msg.agent_id = aid
    msg.action = act # Command
    msg.slist = slist  # String list argument
    msg.arg1 = arg1  # Arguments
    msg.arg2 = arg2
    msg.arg3 = arg3

    msg.data = []
    G_pub_cmd.publish(msg)

def KillAgent(id):
    RFind(id).agent.Delete()
    PublishCmd(gdic.CMD_F_DELETE_AGENT, id)

# Установить координаты агента. Аргументы: id агента, x, y, angle
def SetAgentPos(id, x, y, angle):
    PublishCmd(gdic.CMD_F_SET_APOS, id, '', x, y, angle)

def SetState(id, stat):
    PublishCmd(gdic.CMD_F_SET_STATE, id, '', stat)

################################################################################

#
# Подписываемся на нужные топики
# num_agents - количество агентов
#
def InitUsersTopics(num_agents):
    global G_pub_cmd
    # Входной топик
    # Подписываемся на топик sensors, второй параметр - объем кэша отправки
    inpqlen = max(num_agents*7, 10)
    rospy.Subscriber(gdic.TOPIC_NAME_SENSORS, sensors, sensors_callback, queue_size=inpqlen)

    # Выходной топик
    outqlen = max(num_agents*7, 10)
    G_pub_cmd = rospy.Publisher(gdic.TOPIC_NAME_COMMAND, action, queue_size=outqlen)

    time.sleep(2)
#
#
#
__current_robot = 0
def GetNextRobot():
    global __current_robot
    if __current_robot>=len(Robots):
        __current_robot = 0
    r = Robots[__current_robot]
    __current_robot += 1
    return r
