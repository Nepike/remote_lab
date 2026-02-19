#!/usr/bin/env python3
# coding: utf-8
'''
Version 1.1

Сервер передачи данных по ZigBee
Управляет сервером ZigBee через топики

16.03.2025

LP 18.03.2025
'''


REG_SENDER = 0
REG_READER = 1
REG_FILE_READER = 2

command_topic_name = "xbeeds_cmd"
result_topic_name = "xbeeds_res"

# Маркеры начала и конца потока сообщений
MARKER_START = "<start>"
MARKER_STOP = "<stop>"

# Команды передатчику
CMD_SEND_DATA = 1 # arg1 - адрес получателя. В поле data - передаваемые данные
CMD_SEND_FILE = 2 # arg1 - адрес получателя. В поле data - имя файла

# Команды приемнику
CMD_READ_DATA = 3      # В поле data - полученные данные
CMD_READ_DATA_FILE = 4 # В поле data - имя файла c данными

#
# Ответ системы
#
# Приемник
STAT_WAIT_FOR_DATA     = 1 # Ожидание данных
STAT_DATA_READY        = 2 # Данные готовы. В поле data - полученные данные
STAT_DATA_FILE_READY   = 3 # Данные готовы. В поле data - имя файла-результата

# Передатчик
STAT_START_TRANSFERING = 4 # Идет передача данных
STAT_END_TRANSFERING   = 5 # Идет передача данных

def warning(msg):
    print(f"*** Warning: {msg}")

def error(msg):
    print(f"*** Error: {msg}")
    sys.exit(1)

