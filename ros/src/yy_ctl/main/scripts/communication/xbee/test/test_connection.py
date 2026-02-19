#!/usr/bin/env python3

from zbconnection import ZigBeeConnection


if __name__ == '__main__':
    # Создаем два устройства.
    dev1 = ZigBeeConnection(2, "/dev/ttyUSB0")
    dev2 = ZigBeeConnection(3, "/dev/ttyUSB1")

    # Проверка, что при пустой очереди выдается None
    print(dev2.get_message())
    # Первое устройство отправляет сообщение второму
    print(dev1.transmit_msg("Hello 3!", 3))
    # Проверка, что если очередь не пустая выдается сообщение
    print(dev2.get_message())
    # Первое устройство отправляет сообщение устройству, которого нет
    print(dev1.transmit_msg("Hello 1!", 1))
    # Первое устройство отправляет сообщение всем устройствам
    print(dev1.transmit_msg("Hello -1!"))
    # Вывод того, что приняло второе устройство. ['Hello 3!', 'Hello -1!']
    print(dev2.rec_data_queue)
    for i in range(10):
        # По какой то причине часто 6 и 7 передача неуспешны.
        # Ошибка по таймауту ожидания подтверждения. При этом сообщение
        # может быть и доставлено.
        # Если поставить sleep на полсекунды, то все работает.
        # Пока отсавил так, так как функция transmit позволяет отследить
        # какие сообщения не были отправлены. Пользователь в своем коде
        # может повторно осуществить их отправку.
        print(f" {i} {dev1.transmit_msg(f'Bye {i}')}")
    print(dev2.rec_data_queue)
    # И наоборот. Чистить буфер необходимо, т.к. при отправке всем,
    # отправитель тоже получает свои же сообщения. Просто для наглядности.
    dev1.clear_received_buffer()

    print(dev2.transmit_msg("Hello 2!", 2))
    print(dev2.transmit_msg("Hello 1!", 1))
    print(dev2.transmit_msg("Hello -1!"))
    print(dev1.rec_data_queue)
    for i in range(10):
        print(f" {i} {dev2.transmit_msg(f'Bye {i}')}")
    print(dev1.rec_data_queue)
