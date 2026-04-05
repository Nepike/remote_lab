#!/usr/bin/env python3
# coding: utf-8
"""

30.01.2024
LP 13.02.2024

"""
import sys, math
 
def quaternion_2_euler1(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z


def quaternion_2_euler2(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# in radians
def quaternion_2_euler3(x, y, z, w):
    roll_x = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch_y = math.asin(2*(w*y-z*z))
    yaw_z = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    return roll_x, pitch_y, yaw_z

################################################################################

def r2a(r):
    a = int(math.degrees(r))
    a = a%360
    if a<0: a = a+360
    return a

################################################################################
if __name__ == "__main__" :

    """
    angleR = r*180/math.pi
    angleP = p*180/math.pi
    angleY = y*180/math.pi

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    w = float(sys.argv[4])

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*z))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    angleR = r*180/math.pi
    angleP = p*180/math.pi
    angleY = y*180/math.pi

    print (angleR)
    print (angleP)
    print (angleY)
        yaw_z = math.degrees(math.atan2(t3, t4))
    """

    x = 1
    y = 1
    z = 1
    w = 1

    R1, P1, Y1 = quaternion_2_euler1(x, y, z, w)
    R2, P2, Y2 = quaternion_2_euler1(x, y, z, w)
    R3, P3, Y3 = quaternion_2_euler1(x, y, z, w)


    R1 = math.degrees(R1);
    P1 = math.degrees(P1);
    Y1 = math.degrees(Y1);


    R2 = math.degrees(R2);
    P2 = math.degrees(P2);
    Y2 = math.degrees(Y2);

    R3 = math.degrees(R3);
    P3 = math.degrees(P3);
    Y3 = math.degrees(Y3);

    print(f"{R1} {P1} {Y1}")
    print(f"{R2} {P2} {Y2}")
    print(f"{R3} {P3} {Y3}")
