#!/usr/bin/env python
from fastdrink.capteurs import Ultrason
from picar import back_wheels
import picar

picar.setup()

bw = back_wheels.Back_Wheels(db='config')

bw.ready()


def main():
    while True:
        bw.speed = 25
        bw.backward()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        bw.stop()
