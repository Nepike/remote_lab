#!/usr/bin/env python3
# coding: utf-8
'''
  Определение режима работы (выбор номера программы)
  Используется либо LCD, либо OLED SSD1306
  Все кнопки должны быть подтянуты к 1 (кнопки OLED уже подтянуты)
  -- 2 кнопки выбора номера программы (для OLED: '^', 'v')
  -- 1 кнопка "Старт" (для OLED: '#')

  Если прошло 10 с. и ничего не нажато, то определяется выбор по умолчанию - №1
  Если клавиатура с подтянутыми кнопками не подключена (программа проверяет это), то будет выбор по умолчанию

  Пищалка и индикатор работаю синхронно

  Звуковая (световая) индикация:
  -- запуск программы -- 1 сигнал
  -- ошибка инициализации LCD/OLDE -- 3 сигнала
  -- нормальное завершение программы -- 1 сигнал
  V 1.02
  04.08.2024
  LP 07.11.2024
'''

import RPi.GPIO as gpio
import os, sys, time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__))+"/lib")

import lcddriver
import oledssd1306

# Выбор устройства
USE_LCD = False
USE_OLED = True

gpio.setwarnings(False)
gpio.cleanup()

'''
  GPIO.BOARD — нумерация портов на плате по порядку
  GPIO.BCM — это более низкий уровень и обращается напрямую к номерам каналов на процессоре
    gpio.setmode(gpio.BCM)
    gpio.setmode(gpio.BOARD)
'''

# Было так: pin_mode = gpio.BOARD
pin_mode = gpio.BCM

gpio.setmode(pin_mode)

pin_mode = gpio.getmode() # -> GPIO.BOARD, GPIO.BCM

'''
  PIN_INP[0] -- Up   ('^' номер программы++)
  PIN_INP[1] -- Down ('v' номер программы--)
  PIN_INP[2] -- Mode ('*' не используется)
  PIN_INP[3] -- Start ('#')
'''

if pin_mode==gpio.BOARD:
    PIN_LED = 11  # Индикатор
    PIN_BEEP = 13 # Пищалка
    PIN_INP = [15, 16, 18, 19]
else:
    PIN_LED = 17  # Индикатор
    PIN_BEEP = 27 # Пищалка
    PIN_INP = [22, 23, 24, 10]

def Beep(T):
    gpio.output(PIN_LED, gpio.HIGH)
    gpio.output(PIN_BEEP, gpio.HIGH)
    time.sleep(T)
    gpio.output(PIN_LED, gpio.LOW)
    gpio.output(PIN_BEEP, gpio.LOW)
    time.sleep(T)

def ReadButton():
    for i in range(len(PIN_INP)):
        if gpio.input(PIN_INP[i]) == gpio.LOW:
            while gpio.input(PIN_INP[i]) == gpio.LOW:
                time.sleep(0.1)
            return i
    return -1

if __name__ == '__main__':
    gpio.setup([PIN_LED, PIN_BEEP], gpio.OUT, initial=gpio.LOW)
    gpio.setup(PIN_INP, gpio.IN, pull_up_down=gpio.PUD_UP)

    T = 0.5
    Beep(T)

    PROGR_NUM = 4 # Максимальное количество программ
    curr_prog = 1

    #print(gpio.VERSION)
    #print(gpio.RPI_INFO)
    try:
        if USE_LCD:
            screen = lcddriver.lcd(0x27)
        if USE_OLED:
            screen = oledssd1306.lcd(0x3c)

    except:
        Beep(T)
        Beep(T)
        Beep(T)
        gpio.cleanup()
        print(curr_prog)
        sys.exit(1)

    screen.display_string = screen.lcd_display_string
    screen.clear = screen.lcd_clear

    screen.lcd_clear()

    screen.lcd_display_string(f"# = {curr_prog}", 2)

    exit_stat_str = "???"

    pred_t = -1
    WAIT_TIME = 10 # Время ожижания ввода команды
    TCNT = WAIT_TIME

    #
    # Проверка клавиатуры
    #
    n = sum([gpio.input(PIN_INP[i]) for i in range(len(PIN_INP))])
    kbd_not_found = (n!=len(PIN_INP))

    while True:
        if kbd_not_found:
            exit_stat_str = "Keyboard not found"
            break
        #
        # Разбираемся со временем
        #
        ct = int(time.time())
        if pred_t!=ct:
            pred_t=ct
            screen.lcd_display_string(f"P.SELECTOR | cnt: {TCNT} ", 1)
            TCNT-=1
            if TCNT<0:
                exit_stat_str = "Waiting time expired"
                break

        #
        # Считываем кнопки
        #
        n = ReadButton()
        if n<0: continue
        if n==0: # Up
            curr_prog += 1
            if(curr_prog>PROGR_NUM): curr_prog = 1
        if n==1: # Down
            curr_prog -= 1
            if(curr_prog<1): curr_prog = PROGR_NUM
        if n==2: # Mode
            pass
        if n==3: # Start Button
            exit_stat_str = f"Ok"
            break
        screen.lcd_display_string(f"# = {curr_prog}", 2)
        TCNT = WAIT_TIME


    screen.lcd_display_string(exit_stat_str, 3)
    #
    # Выполняем программу curr_prog
    #
    screen.lcd_display_string(f"Exec prog {curr_prog}...", 4)
    print(curr_prog)
    '''
    if curr_prog==1:
        ....
    if curr_prog==2:
        ....
    if curr_prog==3:
        ....
    if curr_prog==2:
        ....
    '''
    screen.lcd_display_string(f"Exec prog {curr_prog}... Done", 4)

    Beep(T)

    gpio.cleanup()
