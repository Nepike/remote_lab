#!/usr/bin/env python3
# coding: utf-8

"""
  gdic.py
  Глобальный словарь системы
  Author: Valery Karpov

  06.02.15, 09.03.2016, 03.12.2022, 28.09.2024
  Version 3.5
  LP 14.06.2025

"""

import sys, os

def error(msg):
    print("error:", msg)
    raise Exception(msg)
    sys.exit(1)

def warning(msg):
    print("*** Warning:", msg)

def terminate_program(msg = "\n\nProgram terminated"):
    print(msg)
    sys.exit(0)

def pause(s):
    print(s)
    input("")

#
# Поиск пользовательских библиотек и прописывание путей к ним.
# Возвращает имя каталога, где живут библиотеки.
#
def get_kvorum_lib_level():
    for d in [".", "..", "../..", "../../.."]:
        lib_path = d+"/lib"
        fsm_lib_path = d+"/fsm.lib"
        if os.path.exists(lib_path) and os.path.isdir(lib_path) and os.path.exists(fsm_lib_path) and os.path.isdir(fsm_lib_path):
            sys.path.insert(0, fsm_lib_path)
            sys.path.insert(0, lib_path)
            sys.path.insert(0, ".")
            return d
    error("Can't find libraries lib, fsm.lib")

################################################################################
#
# Имена топиков
#
################################################################################
TOPIC_NAME_COMMAND   = "/kaction"   # Команды
TOPIC_NAME_SENSORS   = "/ksensors"  # Множество значений сенсоров
TOPIC_NAME_VIZ_PROC  = "/viz_proc"  # Команда визуализации

################################################################################
#
#
#
################################################################################

Delta_T = 0.1

# Слои
(
LEVEL_LIGHT,   # Освещенность
LEVEL_COLOR,   # Цвет поверхности
LEVEL_IR,      # Эфир (ИК-область)
LEVEL_TEXT,    # Фиктивный слой. Текст
LEVEL_GROUND,  # Поверхность
LEVEL_ONBOARD  # Бортовой настроечный датчик (фиктивный уровень)
) = range(6)

# Словарь цветов
LevelColor = { LEVEL_GROUND: "black", LEVEL_COLOR: "green", LEVEL_LIGHT: "yellow", LEVEL_IR: "red", LEVEL_TEXT: "black"}

################################################################################
#
################################################################################
# Состояния агента
(STAT_NORM, STAT_LEADER, STAT_S0, STAT_S1, STAT_S2) = range(5)

# Словарь цветов
VocColor = { STAT_NORM: "black", STAT_LEADER: "red", STAT_S0: "white", STAT_S1: "yellow", STAT_S2: "green"}

################################################################################
# Тип датчика:
################################################################################
ST_SHARP     = 0
ST_USONIC    = 1
ST_DETECTOR  = 2
ST_CONST     = 3

# Тип возвращаемого значения
RST_SCALAR       = 0
RST_VECTOR       = 1
RST_SUPER_VECTOR = 2

################################################################################
#
#  Список команд для Kvorum2
#
################################################################################
# Движение с крейсерской скоростью (_norm_movespeed, _norm_rotatespeed)
CMD_STOP         = 1  # Останов
CMD_FWD          = 2  # Вперед
CMD_BACK         = 3  # Назад
CMD_LEFT         = 4  # Налево (одним колесом)
CMD_RIGHT        = 5  # Направо (одним колесом)
CMD_FAST_LEFT    = 6  # Налево (танковый разворот с реверсом)
CMD_FAST_RIGHT   = 7  # Направо (танковый разворот с реверсом)

# Движение с дополнительным аргументом
CMD_STOP2          = 21
CMD_FWD2           = 22
CMD_BACK2          = 23
CMD_LEFT2          = 24
CMD_RIGHT2         = 25
CMD_FAST_LEFT2     = 26
CMD_FAST_RIGHT2    = 27

CMD_SET_CURR_2W_SPEED = 28  # Установка текущих скоростей вращения колес (левого и правого)
CMD_SET_CURR_V_SPEED  = 29  # Установка текущих линейной и угловой скоростей (_curr_movespeed, _curr_rotatespeed)
CMD_SET_NORM_V_SPEED  = 30  # Установка крейсерских скоростей - линейной и разворота (_norm_movespeed, _norm_rotatespeed)

# Прочие действия
CMD_BEEP           = 8   # Звуковой сигнал
CMD_BEEP_ON        = 9   # Включить пищалку
CMD_BEEP_OFF       = 10  # Выключить пищалку
CMD_SET_ANG        = 13  # Установить угол сервопривода (град)

# Работа с сенсорами
CMD_GET_SENS       = 11  # Запрос: Получить значения сенсоров. Аргумент - список имен сенсоров

#
# Непосредственное управление на уровне модели
#
CMD_DELETE_OBJ     = 50  # Удалить объект на слое level в координатах агента. Аргумент: слой level

CMD_F_DELETE_AGENT = 51 # Удалить агента. Аргумент: id агента
CMD_F_SET_APOS     = 52 # Установить координаты агента. Аргументы: id агента, arg1=x, arg2=y, arg3=angle

CMD_F_SET_FIELD    = 53 # Установить значение поля. Аргументы: id=value, arg1=x, arg2=y, arg3=level
CMD_F_SET_STATE    = 54 # Установить статус агента. Аргумент: arg1=mode
CMD_F_SET_A_SRC    = 55 # Установить у агента источник сигнала value на уровне level. Аргументы: arg1=level, arg2=value

################################################################################

MY_ADDR = 1

################################################################################

#
# Команды протокола pub_viz (/viz_proc_topic)
#
VIZ_CMD_AINIT     =   0 # Команда инициализации агентов. Аргументы: [(id, x, y, a, cs, shape)]
VIZ_CMD_ADRAW     =   1 # Команда рисования агентов. Аргументы: [(id, x, y, a)]
VIZ_CMD_SET_STATE =   2 # Установить состояние агента
VIZ_CMD_SET_FIELD = 100 # Установить значение поля. Аргументы: x, y, a (level), id (value)

################################################################################
#
# Полезные процедуры
#
################################################################################
# Преобразуем супервектор в вектор
# rmult - масштабный множитель
def SuperVector2Vector(sv, rmult):
    v = []
    n = len(sv)
    for i in range(n):
        v.append(int(sv[i][0]*rmult))
        v.append(sv[i][1])
    return v

# Преобразуем вектор в супервектор
def Vector2SuperVector(v):
    sv = []
    n = len(v)//2
    for i in range(n):
        d1 = v[i*2]
        d2 = v[i*2+1]
        sv.append([d1, d2])
    return sv

def pmap(x, x_min, x_max, out_min, out_max):
    if (x is None) or (x_max is None) or (out_max is None): return 0
    if (x>x_max): x = x_max
    if (x<x_min): x = x_min
    r = (x - x_min) * (out_max - out_min) / (x_max - x_min) + out_min
    return int(r)
