#!/usr/bin/env python
'''
**********************************************************************
* Filename    : ultra_sonic_avoidance.py
* Description : An example for sensor car kit to followe light
* Author      : Dream
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Dream    2016-09-27    New release
**********************************************************************
'''

from fastdrink.capteurs import Ultrason
from fastdrink.config import FW_WHEEL_OFFSET, ULTRASONIC_OFFSET
from fastdrink.keyboard import keyboard
from picar import front_wheels
from picar import back_wheels
from time import sleep
import time as t
import argparse
import picar

picar.setup()

fw = front_wheels.Front_Wheels(db='config')
bw = back_wheels.Back_Wheels(db='config')

ua = Ultrason()

fw.ready()
bw.ready()

fw.turning_max = 45

def main():

    arg = argparse.ArgumentParser()
    arg.add_argument("-d", type=int, default=60, help="Stopping distance")
    arg.add_argument("-s", type=int, default=30, help="Speed")
    args = arg.parse_args()

    calibration()

    stop_dist = args.d
    dist = get_ua_distance()

    s = t.time()
    bw.forward()
    bw.speed = args.s
    while dist > stop_dist:
        fw.turn_straight()
        dist = get_ua_distance()
    stop()
    print("Time taken: ", t.time()-s, " s")

def calibration():
    fw.turn_left()
    sleep(0.25)
    fw.turn_right()
    sleep(0.25)
    fw.wheel.offset = FW_WHEEL_OFFSET
    fw.calibration()
    sleep(0.25)

def get_ua_distance():
    return ua.get_distance() * 10 - ULTRASONIC_OFFSET


def stop():
    bw.stop()
    fw.turn_straight()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop()
