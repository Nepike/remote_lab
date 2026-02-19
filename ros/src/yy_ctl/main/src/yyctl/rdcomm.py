#!/usr/bin/env python
# coding: utf-8
'''
  Функции упакрвки/распаковки данных при передече маршрута по протоколу rc5

  13.03.2025

  LP 08.04.2025
'''
import sys
from datetime import datetime

# Упаковываем число в базу
#   base - формируемая база
#   num - добавляемое число
#   offs - смещение
#   size - количество разрядов
def pack(base, num, offs, size):
    mask = (2**size-1)
    num = num & mask
    return base + (num<<offs)

# Распаковываем число из базы
#   base - база
#   offs - смещение
#   size - количество разрядов
def unpack(base, offs, size):
    mask = (2**size-1)
    n = (base >> offs) & mask
    return n

HDR_SIZE = 2
CS_SIZE = 3

TIP_OBJ = 1
TIP_SCENE = 2
TIP_CMD = 3

CMD_START = 1
CMD_STOP = 2

PkgVoc = [
    {'name': 'obj',   'header': TIP_OBJ  , 'elsize': [HDR_SIZE, 3, 3, 3, 3, 3, 2, 3, 4, CS_SIZE]},
    {'name': 'scene', 'header': TIP_SCENE, 'elsize': [HDR_SIZE, 3, 3, 3, 15, CS_SIZE]},
    {'name': 'cmd',   'header': TIP_CMD,   'elsize': [HDR_SIZE, 3, CS_SIZE]}]

def get_pckg_descr(hdr):
    for e in PkgVoc:
        if hdr==e['header']: return e
    return None

# Расчет CS
def count_CS(data):
    cs = 0
    for i in range(len(data)-1):
        cs = cs ^ data[i]
    mask = (2**CS_SIZE-1)
    cs = cs & mask
    return cs

def error(msg):
    print(f"*** Error: {msg}")
    sys.exit(1)

def pack_data(data):
    voc = get_pckg_descr(data[0])
    if voc is None: error(f"unknown header {data[0]} in {data}")
    # Пересчитываем CS
    CS = count_CS(data)
    data[-1] = CS
    val = 0
    offs = 0
    for i in range(len(data)):
        sz = voc['elsize'][i]
        val = pack(val, data[i], offs, sz)
        offs += sz
    return val, offs, CS

def unpack_data(base):
    offs = 0
    hdr = unpack(base, offs, HDR_SIZE)
    voc = get_pckg_descr(hdr)
    if voc is None: return None, hdr, None
    data = []
    offs = 0
    n = len(voc['elsize'])
    for i in range(n):
        sz = voc['elsize'][i]
        val = unpack(base, offs, sz)
        offs += sz
        data.append(val)

    CS = data[-1]
    # Пересчитываем CS
    realcs = count_CS(data)
    if realcs!=CS:
        return None, hdr, realcs # error("CS error at {base}")
    return data, hdr, realcs

def PackSentences(sentences):
    outdata = []
    for sent in sentences:
        val, size, CS = pack_data(sent)
        outdata.append((val, size, CS))
    return outdata

def PackSentencesFile(fname):
    sentences = ReadSentenсes(fname)
    outdata = PackSentences(sentences)
    return outdata

def UnpackFile(fname):
    outdata = []
    f = open(fname, 'r', encoding='utf-8')
    text = f.read()
    f.close()
    text = text.split('\n')
    for s in text:
        v = s.strip()
        if v=='' or v[0]=='#': continue
        sl = v.split()
        d = [int(e) for e in sl]
        data, hdr, cs = unpack_data(d[0])
        if data is None:
             error(f"illegal data: (header={hdr}, CS={cs})")
        outdata.append(data)
    return outdata

# Чтение предложений из файла и формирование списка списков
def ReadSentenсes(fname):
    outdata = []
    f = open(fname, 'r', encoding='utf-8')
    text = f.read()
    f.close()
    text = text.split('\n')
    for s in text:
        v = s.strip()
        if v=='' or v[0]=='#': continue
        sl = v.split()
        data = [int(e) for e in sl]
        outdata.append(data)
    return outdata

def SaveSentences(fname, sentences):
    print(f"\nSave package: {fname}")
    f = sys.stdout if fname is None else open(fname, 'w', encoding='utf-8')
    current_datetime = datetime.now()
    print(f"# {current_datetime}", file=f)
    for sent in sentences:
        for e in sent:
            print(f"{e} ", end='', file=f)
        print(file=f)
    f.close()

if __name__ == '__main__':

    inpfile = 'data1.txt'
    outfile = 'result'

    # Читаем из файла
    sentences = ReadSentenсes(inpfile)
    print("Source sentences:")
    print(sentences)

    # Формируем двиочное представление для отправки по RC5

    data = PackSentencesFile(inpfile)
    print("RC5 Data:")
    print(data)

    fout = open(outfile, 'w', encoding='utf-8')
    for d in data:
        print(f"{d[0]} {d[1]}", file=fout)
    fout.close()

    print("Read:")
    data = UnpackFile(outfile)
    for e in data:
        print(e)

