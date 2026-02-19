#!/bin/bash

#
# Автозапуск
# Задача - что-то сказать и определить номер программы.
#

HOMEDIR=/home/ubuntu/ros/src/yy_ctl/startup
LOGFILE=/home/ubuntu/log

# Добавляем запись в log-файл
echo "Statup at `date`" >> $LOGFILE

# Озвучиваем запуск скрипта
runuser - ubuntu -c "$HOMEDIR/say.sh Система загружена"
runuser - ubuntu -c "$HOMEDIR/say.sh Выбор текущей программы"

# Запускаем скрипт выбора программы.
# Реальная работа может быть произведена в нем, а может использоваться результат ее работы.
# Программа помещает в стандартный выходной поток номер выбранной программы (номер программы - 1, 2, 3, 4).
# Здесь результат помещается в NPROG.
NPROG=`python3 $HOMEDIR/psel.py`

# Протоколируем результат в log-файле
echo "prog=$NPROG">> $LOGFILE

# Озвучиваем номер программы
runuser - ubuntu -c "$HOMEDIR/say.sh Выбрана программа $NPROG"


#
# Далее делаем все, что необходимо, исходя из значения переменной NPROG
# ...
#
#

runuser - ubuntu -c "$HOMEDIR/say.sh Система готова к работе"
