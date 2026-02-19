"""Описание класса, реализующего приемо-передачу с помощью
устройств Xbee pro series 2.
"""

from serial import SerialException
from typing import Dict, List, Union

from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException, TransmitException
from digi.xbee.models.message import XBeeMessage


# Величина входной очереди сообщений. При величине больше удаляется
# первое сообщение очереди
RECEIVED_DATA_QUEUE_SIZE = 10
# Паттер адресной части. Осуществляется поиск этого паттерна, чтобы
# найти id устройства, к которому направлено сообщение
ADDR_PATTERN = 'addr='


class ZigBeeConnection:
    """Класс-описание протокола приема-передачи."""

    # Входная очередь сообщений
    rec_data_queue: List[str] = []

    def __init__(self, addr: int, port: str, speed: int = 9600):
        """
            addr - адрес устройства. Любое натуральное число. Не зависит
        от параметра NI в прошивке модуля, но лучше, чтобы они совпадали
        (важно при дальнейшей модификации кода под адресную передачу)

            port - имя порта, к которому подключено устройство. В Linux
        системах - это /dev/ttyUSB*, в Windows - COM

            speed - скорость передачи данных по порту. По умолчанию - 9600.
        """
        self.my_address: int = addr
        # Создается и открывается устройство Xbee. Задается функция-callback
        # для принятия входящих сообщений. Прием - асинхронный.
        try:
            self.device = XBeeDevice(port, speed)
            self.device.open()
            self.device.add_data_received_callback(
                self.__received_callback)
        except SerialException as error:
            print(f'{error}')
            exit()

    def __del__(self):
        """Деструктор класса. Закрывает открытое Xbee устройство."""
        self.device.close()

    def __received_callback(self, xbee_message: XBeeMessage) -> None:
        """Фоновая функция-callback обработки входящих сообщений."""
        # Переводим сообщение в строку и разбиваем по разделителю \t,
        # который отделяет адресную часть от информационной
        message: List[str] = xbee_message.data.decode("utf8").split('\t')
        addr: Union[int, None] = None
        addr_data: str = ''
        msg_data: str = ''
        start_element: int = 0
        # Отделяем адресный паттерн, если он есть. Если он был, то
        # addr_data и message[0] станут неравны (адрес в сообщении был)
        addr_data = message[0].lstrip(ADDR_PATTERN)
        if addr_data != message[0]:
            # Получаем адрес устройство и позицию в message, с которой
            # надо заново собрать информационную строку. Это нужно тогда,
            # когда в сообщении тоже были табуляции
            start_element = 1
            addr = int(addr_data)
        # Формируем переданную информационную строку. Необходимо если в строке
        # также были табуляции
        msg_data = message[start_element].join(
            (st + '\t') for st in message[start_element:-1])
        msg_data += message[-1]
        # Если сообщение отправлено всем или этому конкретному устройству,
        # то оно кладется в очередь сообщений.
        if addr is None or addr == self.my_address:
            if len(self.rec_data_queue) == RECEIVED_DATA_QUEUE_SIZE:
                del self.rec_data_queue[0]
            self.rec_data_queue.append(msg_data)

    def transmit_msg(self, msg: str, addr: Union[int, None] = None) -> Dict:
        """Функция передачи сообщений.
            msg - информационное сообщение.

            addr - адрес устройства, которому надо его передать. Если None -
            сообщение отправляется всем.

            Возвращает словарь, где ключ -1 - неуспешная передача,
            а 1 - успешная. В значении по ключу - адрес и сообщение.
        """
        try:
            if addr is None:
                self.device.send_data_broadcast(msg)
            else:
                # Если адрес есть, то в начало строки добавляется
                # ADDR_PATTERN(в данном случае addr=)<адрес устройства>\t.
                self.device.send_data_broadcast(
                    ADDR_PATTERN + str(addr) + '\t' + msg)
        except (TimeoutException, TransmitException):
            return {
                -1: [addr, msg]
            }
        return {
            1: [addr, msg]
        }

    def get_message(self) -> Union[str, None]:
        """Выдает первое сообщение из очереди входных сообщений."""
        if len(self.rec_data_queue) > 0:
            return self.rec_data_queue.pop(0)
        return None

    def clear_received_buffer(self) -> None:
        """Очистка буфера входных сообщений."""
        self.rec_data_queue.clear()
