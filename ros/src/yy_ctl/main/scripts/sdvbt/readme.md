
Сорокоумов П.С.
11.01.2023

Порядок установки ПО для получения сенсорных данных мобильного телефона через Bluetooth
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Установка на телефон приложения
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Установить на телефон приложение, способное отправлять данные через Bluetooth - программу "Sensor data via Bluetooth".
Включить на телефоне Bluetooth.

2. Установка ПО
~~~~~~~~~~~~~~~
Установить на Linux-компьютер (Kubuntu 20.04) зависимости:

$ sudo apt install bluetooth bluez bluez-tools rfkill python3-bluez

3. Установка режима подключения
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Поменять режим подключения Bluetooth на режим совместимости.
Для этого создать вручную директорию /etc/systemd/system/bluetooth.service.d:

$ sudo mkdir /etc/systemd/system/bluetooth.service.d

и в ней файл /etc/systemd/system/bluetooth.service.d/override.conf с содержимым:
'''
[Service]
ExecStart=
ExecStart=/usr/lib/bluetooth/bluetoothd -C
'''

Затем перезапустить (или запустить) Bluetooth-сервис командами:
$ sudo systemctl daemon-reload
$ sudo service bluetooth restart

4. Проверка доступности Bluetooth
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Проверить доступность Bluetooth через стандартные GUI среды KDE. Проще всего найти их из меню ApplicationLauncher, введя в строку поиска "Bluetoooth".
В приложении Adapters включить адаптер, если он не включен; если его нет, нужно сначала поставить драйверы к нему или загрузить поддерживающий модуль ядра, вид которого зависит от производителя.
В приложении Devices желательно запустить поиск видимых устройств и убедиться, что телефон видим.

5. Создание файла устройства
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Создать файл для работы с устройством. Для этого запустить

$ sudo bluetoothctl

и в выведенной консоли выполнить команды:

power on  # включить питание модуля; чаще всего после предыдущего шага оно уже включенл
agent on  # обычно он уже создан
scan on   # будут выводиться MAC-адреса видимых устройств; надо найти из них нужный
scan off  # набрать, если нужное устройство найдено
pair <найденный MAC>  # для создания "пары" (канала с устройством)
                      # (например, для телефона S58Pro: pair 5E:C3:46:78:E1:D6) 
                      # После этого начнутся вские подтверждения и нажатия на кнопки
# выход - Ctrl-D

Если ранее пара уже была создана, можно проверить её наличие командой paired-devices в bluetoothctl.

Когда канал создан, создать файл для связи с устройством /etc/rfcomm0:
$ sudo rfcomm bind 0 <найденный MAC>

Например:
$ sudo rfcomm bind 0 5E:C3:46:78:E1:D6

6. Запуск тестового скрипта
~~~~~~~~~~~~~~~~~~~~~~~~~~~

a) Запустить тестовый скрипт получения данных sdvbt.py как root:

$ sudo python3 sdvbt.py

Должны быть выведены строки:

start advertise 1
Waiting for connection on RFCOMM channel 1

b) Запустить на телефоне приложение "Sensor data via Bluetooth"; после старта сбора данных нажать кнопку "Pick Device" и выбрать нужное устройство.
В консоли должны после этого строки пересылаемых сырых данных вида

received [b'(-0.61292,4.634534,8.242342,0,0,0,0,0,0,0,0,0,0,0)\n']

После обработки входной строки функция parse_message возвращает dictionary со следующими полями:
- ускорения по осям (в системе отсчёта мобильного устройства): AccX, AccY, AccZ;
- угловые скорости поворотов вокруг осей (в системе отсчёта мобильного устройства): AngVelX, AngVelY, AngVelZ;
- ориентацию в глобальной системе отсчёта: Azimuth, Pitch, Roll
- показания сенсора близости (обычно используется для распознавания близости телефона к лицу): Distance
- положение в системе географических координат: Latitude, Longitude, Altitude
- модуль скорости в глобальной системе координат: Speed.


7. Настройка возможности запуска системы без sudo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

a) Изменить права доступа к файлу /var/run/sdp, через который осуществляется обмен данными в выбранном нами режиме.
В качестве одноразового решения (до перезапуска) можно использовать команду

$ sudo chmod o+rw /var/run/sdp

Так как этот файл пересоздаётся при каждом запуске системы, желательно автоматизировать этот процесс.
Для этого следует добавить текущего пользователя в группу bluetooth, если его там ещё нет:

$ cat /etc/group | grep bluetooth 

Если в выводе нет пользователя, добавляем его:

$ sudo usermod -G bluetooth -a <имя_пользователя>  # собственно добавление
$ newgrp bluetooth   # подгрузка группы уже в текущем сеансе, чтобы исключить повторный вход

b) Создать задачи systemd

В каталоге /etc/systemd/system/

1) Создать файл /etc/systemd/system/var-run-sdp.path с содержимым:
'''
[Unit]
Descrption=Monitor /var/run/sdp

[Install]
WantedBy=bluetooth.service

[Path]
PathExists=/var/run/sdp
Unit=var-run-sdp.service
'''

2) Создать файл  /etc/systemd/system/var-run-sdp.service с содержимым:
'''
[Unit]
Description=Set permission of /var/run/sdp

[Install]
RequiredBy=var-run-sdp.path

[Service]
Type=simple
ExecStart=/bin/chgrp bluetooth /var/run/sdp
'''

3) Поставить их в автозагрузку:

$ sudo systemctl daemon-reload
$ sudo systemctl enable var-run-sdp.path
$ sudo systemctl enable var-run-sdp.service
$ sudo systemctl start var-run-sdp.path

8. Источники
~~~~~~~~~~~~

https://play.google.com/store/apps/details?id=com.amalanjula.sensorApp&hl=en&gl=US
https://github.com/pybluez/pybluez/blob/master/examples/simple/rfcomm-server.py
https://stackoverflow.com/questions/34599703/rfcomm-bluetooth-permission-denied-error-raspberry-pi
