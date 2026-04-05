#!/usr/bin/env python3
# coding: utf-8
"""
  rcproto_helper.py

  Library for rcX protocol ros message forming.
  Author: Valery Karpov

  @author Rovbo Maxim
  @version 1.0
  @date 29.05.2018
"""

from msg_rsaction.msg import action
from msg_ans.msg import ans

from kvorum import rcproto

def set_cmd_pub(command_publisher) :
    global cmd_pub
    cmd_pub = command_publisher

# Преобразуем в вектор данных, убирая лишние заголовочные байты
def msg2data(m):
    data = []
    datalen = m[rcproto.POS_LEN]
    pkglen = datalen+rcproto.POS_LEN+1

    if(pkglen>=rcproto.RC_MAX_BUFF):
        gdic.error("msg2data: packet len error: " + str(datalen))

    for i in range(0, datalen):
        data.append(m[i+rcproto.POS_DATA])
    return data

def Super_msg2data(m):
    data = []
    datalen = m[rcproto.POS_LEN]
    pkglen = datalen+rcproto.POS_LEN+1

    if(pkglen>=rcproto.RC_MAX_BUFF):
        gdic.error("msg2data: packet len error: " + str(datalen))

    for i in range(0, datalen/2):
        d1 = m[i*2+rcproto.POS_DATA]
        d2 = m[i*2+1+rcproto.POS_DATA]
        data.append([d1, d2])
    return data

# Обработка принятого сообщения
def AcceptMessage(self, msg):
    if(not self.agent.alive): return

    if(msg.result==rcproto.CMD_F_ANS_GET_SUPER_LOCATOR): # Суперлокатор
        rdata = Super_msg2data(msg.data)
        # Переворачиваем (так надо: углы по-другому отсчитываются)
        n = len(rdata)
        for i in range(0, n):
            self.agent.SuperLocator[i] = rdata[n-i-1]
    else:
        rdata = msg2data(msg.data)
    # sic Очень странное место (вместо 0D приходит 0A). Не знаю, как с этим бороться
    if(msg.result==rcproto.CMD_ANS_GET_SENS or msg.result==0x0A or msg.result==0x0D):
        for i in range(0, len(rdata)):
            self.agent.MainSensors[i] = rdata[i]
    if(msg.result==rcproto.CMD_ANS_GET_USR_DATA): # Локатор
        # Переворачиваем (так надо: углы по-другому отсчитываются)
        n = len(rdata)
        for i in range(0, n):
            self.agent.Locator[i] = rdata[n-i-1]
    if(msg.result==rcproto.CMD_ANS_GET_ALL_REG):
        for i in range(0, len(rdata)):
            self.agent.Registers[i] = rdata[i]
    if(msg.result==rcproto.CMD_ANS_GET_I2C_DATA): # Сначала идет адрес i2c-устройства
        i2caddr = rdata[0]
        # Копируем, пропуская первый элемент (там живет адрес)
        if(i2caddr == gdic.ADDR_I2CDataServer):
            csize = min(gdic.DATALEN_I2CDataServer, len(rdata))
            for i in range(1, csize):
                self.Dataserver[i-1] = rdata[i]
        if(i2caddr == gdic.ADDR_RC5Server):
            csize = min(gdic.DATALEN_RC5Server, len(rdata))
            for i in range(1, csize):
                self.TSOPRC5[i-1] = rdata[i]
    return

    #
    # Запрос всех сенсоров
    #   immediate: True - отправить все запросы сразу. Опасный режим, т.к. может быть переполнение очереди сообщений
    #              False - отправлять запросы по тактам
    #
    def RequestAllSensors(self, immediate=False, req_main_sensors=True, req_locator=True, req_super_locator=True, req_i2cdata=True, req_tsoprc5=True, req_registers=True):
        if(not self.agent.alive): return
        if immediate:
            if(req_main_sensors): self.RequestSensors()
            if(req_registers): self.RequestRegisters()
            if(req_locator): self.RequestUsrData()
            if(req_i2cdata): self.RequestI2CData()
            if(req_tsoprc5): self.RequestRC5Data()
            if(req_super_locator): self.RequestSuperLocator()
        else:
            if(self.cnt_act==0 and req_main_sensors): self.RequestSensors()
            if(self.cnt_act==1 and req_registers): self.RequestRegisters()
            if(self.cnt_act==2 and req_locator): self.RequestUsrData()
            if(self.cnt_act==3 and req_i2cdata): self.RequestI2CData()
            if(self.cnt_act==4 and req_tsoprc5): self.RequestRC5Data()
            if(self.cnt_act==5 and req_super_locator): self.RequestSuperLocator()
            self.cnt_act+=1
            if(self.cnt_act>self.NCNT): self.cnt_act = 0
        return

    def RequestSensors(self):
        self.Publish(rcproto.CMD_GET_SENS)

    def RequestRegisters(self):
        self.Publish(rcproto.CMD_GET_ALL_REG)

    def RequestUsrData(self):
        self.Publish(rcproto.CMD_GET_USR_DATA)

    def RequestI2CData(self):
        self.Publish(rcproto.CMD_GET_I2C_DATA, gdic.ADDR_I2CDataServer, gdic.DATALEN_I2CDataServer)

    def RequestSuperLocator(self):
        self.Publish(rcproto.CMD_F_GET_SUPER_LOCATOR)

    def RequestRC5Data(self):
        self.Publish(rcproto.CMD_GET_I2C_DATA, gdic.ADDR_RC5Server, gdic.DATALEN_RC5Server)

    #
    # Действия
    #
    def Stop(self): self.Publish(rcproto.CMD_STOP)
    def GoFwd(self): self.Publish(rcproto.CMD_FWD)
    def GoBack(self): self.Publish(rcproto.CMD_BACK)
    def GoFastLeft(self, ang=0): self.Publish(rcproto.CMD_FAST_LEFT, ang)
    def GoFastRight(self, ang=0): self.Publish(rcproto.CMD_FAST_RIGHT, ang)
    def GoLeft(self, ang=0): self.GoFastLeft(ang)
    def GoRight(self, ang=0): self.GoFastRight(ang)

    def StepFwd(self, n): return
    def StepBack(self, n): return
    def StepLeft(self, n): return
    def StepRight(self, n): return
    def EatProc(self): self.Publish(rcproto.CMD_DEBUG, gdic.LEVEL_COLOR)

    #
    # Сообщение выходного топика
    #
    def Publish(self, act, arg1 = 0, arg2 = 0, arg3 = 0):
        msg = action()
        msg.team_id = 1
        msg.agent_id =  self.agent.id
        msg.action = act # Command
        msg.arg1 = arg1  # Argument 1
        msg.arg2 = arg2  # Argument 2
        msg.arg3 = arg3  # Argument 3
        msg.data = []
        self.pub.publish(msg)
    #
    # Выполнение действия act с аргументом arg
    #
    def Make(self, act, arg1 = 0):
        if(not self.agent.alive): return
        if(act==PROC_STOP): self.Stop()
        if(act==PROC_GOFWD): self.GoFwd()
        if(act==PROC_GOBACK): self.GoBack()
        if(act==PROC_GOLEFT): self.GoFastLeft(arg1)
        if(act==PROC_GORIGHT): self.GoFastRight(arg1)
        if(act==PROC_STEPFWD): self.StepFwd(1)
        if(act==PROC_STEPBACK): self.StepBack(1)
        if(act==PROC_STEPLEFT): self.StepLeft(1)
        if(act==PROC_STEPRIGHT): self.StepRight(1)
        if(act==PROC_KILL): KillAgent(arg1)
        if(act==PROC_EAT): self.EatProc()
        if(act==PROC_SET_STATE): self.Publish(rcproto.CMD_F_SET_STATE, arg1)
        return

    def EvaluateSensorsInfo(self):
        return

    def ShowStatus(self, show_main_sensors=True, show_locator=True, show_super_locator=True, show_dataserver=True, show_tsoprc5=True, show_registers=True):
        if (show_main_sensors):
            print("MainSensors", len(self.agent.MainSensors), self.agent.MainSensors)
        if (show_locator):
            print("Locator", len(self.agent.Locator), self.agent.Locator)
        if (show_super_locator):
            print("SuperLocator", len(self.agent.SuperLocator), self.agent.SuperLocator)
        if (show_dataserver):
            print("Dataserver", len(self.Dataserver), self.Dataserver)
        if (show_tsoprc5):
            print("TSOPRC5", len(self.TSOPRC5), self.TSOPRC5)
        if (show_registers):
            print("Registers", len(self.agent.Registers), self.agent.Registers)
        return

    #
    # Работа с автоматами
    #
    def FindFSM(self, name):
        for f in self.FSMList:
            if f.Name==name:
                return f
        gdic.error("FindFSM: "+name+" not found")

    def LoadFSM(self, name):
        self.curr_fsm = self.FindFSM(name)
        self.curr_fsm.reset()
        return self.curr_fsm

################################################################################
#
# Вспомогательные функции
#
################################################################################
#
# Системные функции
#

def RFind(id):
    global Robots
    for r in Robots:
        if(r.agent.id == id):
            return r
    gdic.error("RFind: agent "+str(id) + " not found")

def AcceptMsg():
    global MSG_ANS
    if(MSG_ANS==None): return
    # Адресата извлекаем из data
    addr = MSG_ANS.data[3]
    if(addr!=0):
        r = RFind(addr)
        r.AcceptMessage(MSG_ANS)
    MSG_ANS = None
    return

def send_cmd(aid, cmd, arg1 = 0, arg2 = 0, arg3 = 0):
    global cmd_pub

    msg = action()

    msg.team_id = 1
    msg.agent_id =  aid
    msg.action = cmd # Command
    msg.arg1 = arg1  # Arguments
    msg.arg2 = arg2
    msg.arg3 = arg3
    msg.data = []

    cmd_pub.publish(msg)

