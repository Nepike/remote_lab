"""Описание класса, реализующего приемо-передачу с помощью
устройств Xbee pro series 2.
v 3.0 от 18.03.25.
Изменения по сравнению с v 1.0:
- В функции transmit_msg тип аргумента addr [List[int], None]
Дает возможность указать несколько устройств, которым нужно
передать сообщение. Если None - broadcast передача.
- Исправлен баг со статической переменной класса ZigBeeConnection
rec_data_queue. Перенесена в конструктор класса.
- Изменена функция __receive_callback, т.к. теперь нет необходимости
проверки адреса принимающей стороны.
- Добавлена переменная self.id, хранящая прошитый Node Id (NI) устройства.
"""

from serial import SerialException
from typing import Dict, List, Union

from digi.xbee.devices import RemoteZigBeeDevice, ZigBeeDevice
from digi.xbee.exception import TimeoutException, TransmitException
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.models.message import XBeeMessage


# Величина входной очереди сообщений. При величине больше удаляется
# первое сообщение очереди
RECEIVED_DATA_QUEUE_SIZE = 10


class ZigBeeConnection:
    """Класс-описание протокола приема-передачи."""

    def __init__(self, addrs: Union[Dict, str], port: str, speed: int = 9600):
        """
            addrs - словарь, где ключ - id устройства, а значение - его
            64-битный адрес. Либо строка с путем к файлу-описанию соответствия
            id устройства и его адресом.

            port - имя порта, к которому подключено устройство. В Linux
        системах - это /dev/ttyUSB*, в Windows - COM

            speed - скорость передачи данных по порту. По умолчанию - 9600.
        """

        # Создается и открывается устройство Xbee. Задается функция-callback
        # для принятия входящих сообщений. Прием - асинхронный.
        try:
            self.device = ZigBeeDevice(port, speed)
            self.device.open()
            self.id = self.device._node_id
            self.device.add_data_received_callback(
                self.__received_callback)
        except SerialException as error:
            print(f'{error}')
            exit()
        # Входная очередь сообщений
        self.rec_data_queue: List[str] = []
        # Словарь с id устройства и его 64-битным адресом
        self.addresses: Dict[int, str] = {}
        # Проверка, передан ли словарь или путь к файлу-описнаию
        if isinstance(addrs, dict):
            self.addresses = addrs
        else:
            with open(addrs, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    id_and_addrs = line.split()
                    self.addresses[int(id_and_addrs[0])] = id_and_addrs[1]

    def __del__(self):
        """Деструктор класса. Закрывает открытое Xbee устройство."""
        self.device.close()

    def __received_callback(self, xbee_message: XBeeMessage) -> None:
        """Фоновая функция-callback обработки входящих сообщений."""
        if len(self.rec_data_queue) == RECEIVED_DATA_QUEUE_SIZE:
            del self.rec_data_queue[0]
        self.rec_data_queue.append(xbee_message.data.decode("utf8"))

    def transmit_msg(self,
                     msg: str,
                     addrs: Union[List[int], None] = None) -> Dict:
        """Функция передачи сообщений.
            msg - информационное сообщение, тип str.

            addrs- список адресов устройств, которым надо передать msg.
            Если None - сообщение отправляется всем.

            Возвращает словарь, где ключ номер переданного сообщения,
            а значение - словарь со статусом передачи (
            success - успешная передача,
            exception - было выброшено исключение,
            no_addr - в переданном словаре нет устройства с таким id
            ), адрес устройства, куда передается сообщение и сообщение.
        """
        out_data = dict()
        if addrs is None:
            try:
                self.device.send_data_broadcast(msg)
            except (TimeoutException, TransmitException) as error:
                out_data[0] = [f'exception: {error}', addrs, msg]
            else:
                out_data[0] = ['success', addrs, msg]
        else:
            for addr, msg_counter in zip(addrs, range(len(addrs))):
                address = self.addresses.get(addr)
                if address is not None:
                    remote = RemoteZigBeeDevice(
                        self.device,
                        XBee64BitAddress.from_hex_string(
                            address))
                    try:
                        self.device.send_data(remote, msg)
                    except (TimeoutException, TransmitException) as error:
                        out_data[msg_counter] = [f'exception: {error}',
                                                 str(addr),
                                                 msg]
                    else:
                        out_data[msg_counter] = ['success', str(addr), msg]
                else:
                    out_data[msg_counter] = ['no_addr', str(addr), msg]
        return out_data

    def get_message(self) -> Union[str, None]:
        """Выдает первое сообщение из очереди входных сообщений."""
        if len(self.rec_data_queue) > 0:
            return self.rec_data_queue.pop(0)
        return None

    def clear_received_buffer(self) -> None:
        """Очистка буфера входных сообщений."""
        self.rec_data_queue.clear()
