ROS PROJECTS 04.04.2024
КАНОНИЧЕСКАЯ ВЕРСИЯ KINETIC/MELODIC/NOETIC

---------------------------------------------------------

04.04.2024
GMOD, YYCTL оформлены как пакеты
---------------------------------------------------------

24.03.2024
KVORUM оформлен как пакет, со всеми вытекающими

---------------------------------------------------------

12.06.2021

# Curses
sudo apt-get --yes install libncurses5-dev libncursesw5-dev

# Joystick
sudo apt-get install joystick
sudo apt-get install ros-noetic-joy

# Addons
sudo apt install wmctrl

---------------------------------------------------------

07.03.2020
1. Исправлен файл \src\lib\servlib.cpp
2. Определены как игнорируемые (из=за ошибок компиляции):
  src/rs/rsmanager3: CATKIN_IGNORE
  src/pll/pllclient: CATKIN_IGNORE
  src/intelligent_wheelchair/wheelchair_moves: CATKIN_IGNORE
  src/scene_localization: CATKIN_IGNORE

---------------------------------------------------------

18.09.2019 

Добавлен \src\kvorum\msg_kvorum\msg\usrcmd.msg 
Изменен  \src\kvorum\pylib\behavior.py
---------------------------------------------------------

10.04.2019 (?)
Изменен код для сервера датчика цветов на Arduino (ros\arduino\servers\colorserver).
Изменен код отладочной программы верхнего уровня управления под Mega для YARP-2 (YARP\tmu\prog\arduino\usr\YARP2_Mega_HL_Test)
Добавлена программа для езды по линии (YARP\tmu\prog\arduino\usr\YARP2_Mega_HL_Line)

---------------------------------------------------------

22.04.2019
Каталог src/lib/rcrpto ликвидирован
rcproto теперь живет в одном экземпляре, в каталоге arduino/libraries
В каталоге src/lib/ rcrpto - это символическая ссылка на arduino/libraries/rcproto

---------------------------------------------------------

19.03.2018
Добавлен код для интерфейсов-драйверов разных моделей роботов, используемый в системе виртуальной
реальности робота, а также прочий относящийся к ней код и некоторые другие общие пакеты (одометрия).

---------------------------------------------------------

22.08.2017
Некоторые пакеты пернесены в каталог src/sys:
  control_msgs, control_toolbox, realtime_tools, ros_control, ros_controllers

---------------------------------------------------------

15.08.2017

1. Изменения в spotserver
2. Добавление каталога polygone

Судя по всему, при установке пакетов надо разобраться с ros_control
(см. http://wiki.ros.org/ros_control?distro=indigo)

Помогает:

sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers

или

cd CATKIN_WORKSPACE/src
wstool init
wstool merge https://raw.github.com/ros-controls/ros_control/indigo-devel/ros_control.rosinstall
wstool update
cd ..
rosdep install --from-paths . --ignore-src --rosdistro indigo -y
catkin_make

(ссылка: http://wiki.ros.org/ros_control?distro=indigo)

---------------------------------------------------------

04.08.2017

Основные изменения:
- Добавлен тест энкодеров (прошивка) для одноканальных энкодеров
- mvctl3 добавлены режимы компиляции для МПП большого и маленького
- Обновлён скрипт установки пакетов для работы системы 
  (у Малышева должна быть более полная версия)
- Добавлен скрипт для полуавтоматического сравнения с канонической версей
- drrobot_odom модифицирован для работы с параметрами робота
- src/drrobot/msg_user_control удалён по причине дупликата (более нового) в msg/msg_user_control
  ??? Файлы CMakeLists.txt, package.xml
- В ermanctl добавлены новые команды движения с заданными скоростями колёс
- Исправлены ошибки в распознавании речевых команд, связанные с юникодом
- Обновлена версия пакетов П.Сорокоумова, связанных с GPS
- Обновлена версия пакетов П.Сорокоумова, связанных с обнаружением линии полигона
- Изменён пакет библиотек, эмулирующий работу клиента и сервера целевого оборудования
- Добавлены дополнительные параметры (в т.ч. для симуляции) в launch скрипты МПП
- Улучшен метод движения МПП вдоль траектории (trajectory_follower_2)
- Реализован Д-метод на основе сравнения множеств для распознавания команд

Более подробный список изменений ниже.
---
Merge branch 'pre_merge_04_08_2017'
Merge 'canonical_mpp' and 'canonical_no_mpp'
Add test fixes to hardware code and ermanctl
Fix pid in hardware code
Fix big MPP geometry and small fixes in hardware code
Add ermanctl commands with pid speed uhjk
Add +x permissions to kvorum scripts
Merge branch 'dropbox_sync_v0.6'
Add changes from dropbox presync v0.6
Merge branch 'master' of https://gitlab.com/robofob/canonical
Fix command recognition using d-method and unicode fixes
Change launchfile to new command recognition module
Merge branch 'master' of https://gitlab.com/robofob/canonical
Add mvctl3 for invalid MPP
Add command recognition as sets
Fix trajectory following bad behaviour near finish, fix spam of movement finishes
Add encoder test for 1-channel encoders
Add odometry from params initialization
Add arguments to simulation launch
Add packages to installation scrip
Add mpp aws protocol emulation client and server
Fix route follower 2 and its launch
Fix encoder driver error in msgs

??? Change ardsrv command buffer to 1
  ***** new
    ros::Subscriber pos_sub = nsub.subscribe(sub_topic, 1, GetMsg);
  ***** ARDSRV.CPP
    ros::Subscriber pos_sub = nsub.subscribe(sub_topic, 10, GetMsg);
  *****


ADD KOSTYL for arduino addr check, modify for reversed wheels
WIP line follower code
Add trajectory following with builtin interval approximation
Fix permissions
Fix buildall to set chmod +x to setmode
WIP utility
Add script for canonical update preparation
Merge branch 'master' of https://gitlab.com/robofob/canonical
Add user key for Yandex SpeechKit Cloud

---------------------------------------------------------

20.06.2017
Основные изменения:
- Добавлены вспомогательные скрипты, связанные с обработкой git
- Изменена обработка приоритетов команд в smp.py (основной обработчик автоматов) и сокращена
  длина фраз голосовой обратной связи
- Переписан словарный обработчик текстовых команд, выделяющий из фразы номера команд и модификаторы
  приоритетов, на Python, изменена некоторая логика работы
- Улучшен интерфейс запуска голосовых обработчиков с помощью launch файлов (добавлены аргументы
  для тестового запуска, регулировки уровня шума, номера робота)
- Добавлена строчка соединения с камерой нового типа в connect_camera.bash
- Исправлены имена состояний у автомата замирания
- Добавлен новый ключ Yandex Cloud SpeeckKit API (на моё имя) в обработчик речи, добавлен 
  регулируемый параметр уровня шума
- Добавлен словарь команд в виде модуля Python

Более подробный список изменений ниже.
---
Add script for canonical update preparation
Merge branch 'master' of https://gitlab.com/robofob/canonical
Add user key for Yandex SpeechKit Cloud
Fix commentary
Fix fsm paralysis state names
Add an authorization for robot3 camera
Add git utility script
Add new camera
Add voice feedback for unknown commands
Add voice command modifications
Fix command priority sorting
Change commands priority processing to inverse
Add sound threshold parameter to speech-to-text
Add script to show assume-unchanged files
Add some machine-dependent scripts to ignore
Fix text commands recognition id check
Change text recognition command priorities to float
Add python version of text commands recognition
Add new modifier word and fix debug message
Add params to voice launch
Extend dictionary of voice commands

---------------------------------------------------------
01.06.2017

Основные изменения:
- Добавлен пакет msg_speech - модификация сообщений и изменение имени в целях улучшить интерфейс
обработки голосовых команд;
- Добавлены файлы запуска для отображения карты (launch/map.launch, map.rviz);
- Добавлен пакет обработки голосовых команд с приоритетами (voice_command_recognition, бывший gclient), улучшен пакет распознавания речи (speech_to_text, бывший googlesp) и добавлено использование облачного сервиса Яндекса, добавлены соответствующие файлы конфигурации
- Модифицированы и добавлены конечные автоматы действий робота
- В основной управляющий модуль конечных автоматов добавлены новые автоматы, голосовое оповещение о принятии команд, в том числе с приоритетами, рефлексы (на основе автомата рефлексов)

Удалены:
src/gspeech/gclient/src/gclient.voc - лишний, не используется
src/multibot_config/share/ObjectBase.xml - старая версия


---------------------------------------------------------
28.04.2017

1. Библиотека igraph
Либо скачать отсюда: http://igraph.org/c/#downloads
либо взять файл igraph-0.7.1.tar.gz отсюда
./usr/libs

Установить, запустив из папки библиотеки

./configure
make
make check
make install
(или sudo make install)

2. Автоматы
Добавлены и исправлены различные автоматы, добавлен учёт приоритетов команд,
рефлексы на автоматной системе, новая версия распознавания.

3. Прочее
Улучшены скрипты установки (buildall и setmode).

---------------------------------------------------------
10/03/2017

1. Изменения в rcproto
rcproto.cpp и rcproto.h переименованы в rcproto2.cpp и rcproto2.h
Из rcproto2.h коды команд и наименования регистров помещены в файл tmucmd.h

Изменения касались "упрятывания" в структуру TPckg адресов функций (rcError, rcReadByte и проч.), а также 
бывшей голбальной переменной MY_ADDR.
Теперь структура TPckg является самодостаточной и готовой к превращению в объект

2. Изменены все модули, в которых фигурировал rcproto

---------------------------------------------------------
25/02/2017
Изменения в топике action.msg
Вместо
uint8 team_id    # Team id
uint8 agent_id   # Agent id
uint8 action     # Command

Стало:
int32 team_id   # Team id
int32 agent_id  # Agent id
int32 action    # Command

---------------------------------------------------------
18/02/2017
Изменения в системе KVORUM

---------------------------------------------------------
30/09/2016

Описание изменений в коде канонической версии в связи с переходом на
платформу-коляску от велосипеда (и некоторые небольшие изменения, не 
связанные с этим).

Список изменений:

Создание папки src/mpp для МПП-специфичных пакетов
src/mpp/mpp_odom (Перевод показаний энкодеров в координаты одометрии - 
  для велосипеда, для коляски энкодеров пока нет)
src/mpp/mpp_odom_driver (Приём сообщений от компаса)
src/mpp/mpp_test (Старые тесты с обновлениями для новой системы и черновик 
  библиотеки взаимодействия с целевым оборудованием, также необходимый для
  тест)
src/mpp/msg_odom (Сообщения для одометрии)

arduino/mpp/invalid/mpctl (Изменения прошивки)

arduino/mpp/invalid/joystick_emulator (Попытка эмулировать напряжения джойстика для активации
  системы управления колёсами: несмотря на правильно выставленные напряжения, не
  работает. Возможно, что из-за отсутствия DAC на всех выходах)
arduino/mpp/invalid/joystick_emulator_analog (Эмуляция управляющих сигналов джойстика - двух
  осей, задающих скорость вращения и поступательного движения)
arduino/mpp/invalid/joystick_sniffer (Считывание значений джойстика по всем каналам для 
  изучения его особенностей)

---------------------------------------------------------
07/09/2016
1. Изменены erlib, ermanctl:
   Добавлена команда ERCMD_SET_SPEED
2. arduino: mctl_mw
3. arduino: tpdu

---------------------------------------------------------
28/07/2016
1. Изменен ardsrv
2. Сгруппированы каталоги семейства drrobot
3. Сгруппированы каталоги семейства scene_localization
4. Выделен каталог multiribit_congig

---------------------------------------------------------
28/03/2016
1. Внесена рабочая версия системы objdetector
---------------------------------------------------------
17/03/2016
1. rcproto
   Изменены коды и наименования некоторых команд

   CMD_DATA -> CMD_ANS_GET_SENS
   CMD_REG_VAL -> CMD_ANS_GET_REG
   CMD_ALL_REG_VAL -> CMD_ANS_GET_ALL_REG
   CMD_USR_DATA -> CMD_ANS_GET_USR_DATA
   CMD_I2C_DATA -> CMD_ANS_GET_I2C_DATA
---------------------------------------------------------
04/03/2016
1. Изменена структура каталогов
   Проекты частично сгруппированы по своим директориям.
2. Изменения в пакетах drrobot

---------------------------------------------------------
05/02/2016

Изменены:
1. Программы ходового контроллера Arduino

2. Протокол обмена информацией с ходовым контроллером в плане работы с i2c-устройствами (серверами)
   В т.ч. добавлена команда #define CMD_I2C_DATA           0x35  /// Ответ на CMD_GET_I2C_DATA

3. Модули:
    ardsrv
    ermanctl
    ersenfield (учтены данные от i2c контроллеров)
    ermain (вывод данных от i2c контроллеров)
    lib/rcproto
    lib/erlib

4. Топики
4.1. msg_senfield
    Добавлено поле
    # Arduino i2c-servers
    msg_ans/ans ardi2c
4.2. msg_rsaction
    Добавлено поле uint8[] data # Data array
---------------------------------------------------------
04/11/2015

Подключение контроллеров:
  mpctl   - USB0
  mpmanip - USB1
Соответствующие изменения в
etc/mpp/ardsrv.ini

pllsrvl.ini - порт ttyACM0

Изменения:
1. mpmanip.ino
2. ermain.cpp
3. ardsrv.cpp
4. pllsrvl.cpp

---------------------------------------------------------

8/12/2015
...
sudo apt-get install ros-indigo-usb-cam

Установить библиотеку lxml для python :
sudo apt-get install python-lxml

Установить python-pip :
sudo apt-get install python-pip

Установить через pip библиотеку autobahn[twisted] :
sudo pip install autobahn[twisted]

Не забыть проверить ip робота в start скрипте

Если websocket сервер\клиент не заработал, то стоит попробовать установить
самую-самую последную версию кода из исходников autobahn:
sudo pip install git+https://github.com/crossbario/autobahn-python

NOTE: возможно, нужно также установить gstreamer версии 1.0 со всеми плагинами 
(по умолчанию стоит версия 0.10) и v4l2loopback (может, что даже из исходников)

sudo apt-get install v4l2loopback-dkms v4l2loopback-utils
sudo apt-get install gstreamer1.0-*
