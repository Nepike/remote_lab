Обустройство ROS Noetic (Server and Raspberry)
~~~~~~~~~~~~~~~~~~~~~~~
14.06.2021, 17.12.2024, 19.03.2025

################################################################
#
# Установка компонентов
#
################################################################

# pip
sudo apt install python3-pip

# Камера
-- Проверить наличие:
rospack find usb_cam
-- Если нет ([rospack] Error: package 'usb_cam' not found), то установить:
sudo apt-get install ros-noetic-usb-cam

# Addons
# libuvc
sudo apt install libuvc-dev

# Для Python
pip install accessify

# Curses
sudo apt-get --yes install libncurses5-dev libncursesw5-dev

# ros_serial
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino

# Joystick
sudo apt-get install joystick
sudo apt-get install ros-noetic-joy

# Утилита управления окнами wmctrl
sudo apt-get install wmctrl

# OpenCV
pip install opencv-contrib-python

# ZigBee (XBee)
pip install digi-xbee

# tmux
sudo apt install tmux
Рекомендуемый конфигурационный файл:
https://github.com/gpakosz/.tmux

$ cd
$ git clone https://github.com/gpakosz/.tmux.git
$ ln -s -f .tmux/.tmux.conf
$ cp .tmux/.tmux.conf.local .

# ifconfig, nmap
sudo apt-get install nmap

------------------------------------------
Возможные проблемы
------------------------------------------
Для Rasbperry Pi (если AttributeError: module 'cv2.cv2' has no attribute 'aruco'):
$ pip uninstall opencv-python
$ pip uninstall opencv-contrib-python
$ pip install opencv-contrib-python
------------------------------------------

################################################################
#
# Системные настройки
#
################################################################

1. Изменение ~/.bashrc
~~~~~~~~~~~~~~~~~~~~~~
A) В конец файла записать:
source /opt/ros/noetic/setup.bash
source ~/ros/devel/setup.bash

Б) Работа с роботом
-- Мастер
Добавить в конец указание того, что этот ПК будет мастером
IP-адрес можно посмотреть командой ifconfig

# Remote master (example)
IP_MASTER=192.168.1.37
или
IP_MASTER=`hostname -I`
source `rospack find canonical_utilities`/scripts/connect_as_ros_master.bash $IP_MASTER

-- Бортовой ПК
Надо знать адрес мастера
IP_MASTER=192.168.0.101
IP_SLAVE=`hostname -I`
source `rospack find canonical_utilities`/scripts/connect_as_ros_slave.bash $IP_MASTER $IP_SLAVE


2. Права доступа к портам
~~~~~~~~~~~~~~~~~~~~~~~~~
Посмотреть существующие группы, найти там себя:
$ groups

Добавить к группе:
$ sudo usermod -aG dialout [user]

Например:
$ sudo usermod -aG dialout karpov

3. Настойка среды mc для работы с ino-файлами
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
В файл ~/.config/mc/mc.ext добавить строки:

# ino
shell/i/.ino
     Open=/usr/lib/mc/ext.d/doc.sh open msdoc

Либо напрямую, либо отсюда:
   mc: (Menu): Commands->Edit extention file

4. Системное время в Ubuntu и в Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Если системные часы работают в локальном времени (как в windows):
$ sudo hwclock --localtime --adjust

Чтобы вернуть обратно:
$ sudo hwclock --utc --adjust

Еще один вариант настройки.
Проверить состояние:

$ timedatectl | grep local

Если в ответ "RTC in local TZ: no", то в Биосе время Гринвича (+0)
Если в ответ "RTC in local TZ: yes", то локальное (то есть что в Linux, то и в Биосе, и так же будет в Windows).

Чтобы Линукс при выключении не переводил часы на «+0», делаем
$ timedatectl set-local-rtc 1

чтобы вернуть все назад:
$ timedatectl set-local-rtc 0

################################################################
#
# Прочее
#
################################################################

Установка даты и времени (актуально для Raspberry)
sudo date -s "2024-08-17 16:30"
