#!/usr/bin/env python3

from time import sleep

from zbconnection import ZigBeeConnection


if __name__ == '__main__':
    # Создаем три устройства.
    dev1 = ZigBeeConnection("addresses.cfg", "/dev/ttyUSB0")
    dev2 = ZigBeeConnection({
        10: "0013A20040A62B24",
        11: "0013A200405E0D50",
        12: "0013A200405E0D49",
        13: "0013A200405E0D64"},
        "/dev/ttyUSB1")
    dev3 = ZigBeeConnection("addresses.cfg", "/dev/ttyUSB2")

    addrs = []
    for addr in range(10, 14):
        addrs.append(addr)
        print(dev1.transmit_msg(f'Msg {addr}', addrs))
        sleep(0.1)
        print(dev2.id, dev2.rec_data_queue)
        print(dev2.get_message())
        print(dev3.id, dev3.rec_data_queue)
        print(dev3.get_message())

    dev1.clear_received_buffer()
    dev2.clear_received_buffer()
    dev3.clear_received_buffer()
    addrs.clear()

    for addr in range(10, 14):
        addrs.append(addr)
        print(dev2.transmit_msg(f'Msg {addr}', addrs))
        sleep(0.1)
        print(dev1.id, dev1.rec_data_queue)
        print(dev1.get_message())
        print(dev3.id, dev3.rec_data_queue)
        print(dev3.get_message())

    dev1.clear_received_buffer()
    dev2.clear_received_buffer()
    dev3.clear_received_buffer()
    addrs.clear()

    for addr in range(10, 14):
        addrs.append(addr)
        print(dev3.transmit_msg(f'Msg {addr}', addrs))
        sleep(0.1)
        print(dev1.id, dev1.rec_data_queue)
        print(dev1.get_message())
        print(dev2.id, dev2.rec_data_queue)
        print(dev2.get_message())

    dev1.clear_received_buffer()
    dev2.clear_received_buffer()
    dev3.clear_received_buffer()
    addrs.clear()
