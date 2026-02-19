#
# Описание робота
#

########################################################################
# Создание робота
########################################################################

def CreateRobot(id, pos, tip):
    print ('create {}'.format(id))
    # Аргументы: cid, cpos, cshape, cenv, csize = 0
    A = TAgent(id, pos, tip, Env, csize = 0.75)

    # A.traceOn = True

    # Сенсоры
    # Аргументы: caddr, cid,  cdir, cang, cR, cLevel, ctip, cvaltype = gdic.ST_SCALAR, cfproc = None, cvalue = 0

    # HC-SR04 (ultrasonic ranger)
    angle = 15 # угол раствора
    dist = 50 # in cm, only max distance is considered

    # USONIC левый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 8,  45, angle,
        dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # USONIC правый передний
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 9, -45, angle,
        dist, gdic.LEVEL_GROUND, gdic.ST_USONIC))

    # TODO: more usonic and IR rangers

    # TODO: landmark (IR comms) sensors 
    # TODO: change to I2C if possible

    # Food sensor
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 10, 0, 45,
		100, gdic.LEVEL_COLOR, gdic.ST_DETECTOR))

    # Landmark sensors
    # TODO: check 45 or 90 degrees cone needed 
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 11, 0, 45,
		100, gdic.LEVEL_IR, gdic.ST_DETECTOR))
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 12, 90, 45,
		100, gdic.LEVEL_IR, gdic.ST_DETECTOR))
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 13, 180, 45,
		100, gdic.LEVEL_IR, gdic.ST_DETECTOR))
    A.Sensors.append(TSensor(gdic.ADDR_MvCtl, 14, 270, 45,
		100, gdic.LEVEL_IR, gdic.ST_DETECTOR))

    # Encoders, используем однобайтовое представление
    # TODO: double-byte representation and max/min encoder ticks
    wheel_radius = 11 # wheel radius, in cm 
    enc_resolution = 55 # encoder ticks per turn
    wheel_dist = 48 # distance between the wheels, in cm

    A.Sensors.append(TEncoder(19, -1,
        wheel_radius, enc_resolution, wheel_dist)) # left
    A.Sensors.append(TEncoder(21, 1,
        wheel_radius, enc_resolution, wheel_dist)) # right
 
    return A

########################################################################
# Роботы
########################################################################

random.seed(0)

Agents.append(CreateRobot(1, [20,5,90], "turtle"))
for i in range(2,2) :
	print("creating agent #{}".format(i))
	Agents.append(CreateRobot(i, [random.randint(10,90), 
	random.randint(10,90),
	random.randint(0,360)], "elefant"))

