# fast heading check in 2D
#
# - acquire new data for n seconds 
# - acquire additionnal data for n seconds
# - calibrate
# - test

import imu9_driver_v2 as imu9drv
import numpy as np
import time
import sys

imu = imu9drv.Imu9IO()

mag_raw = []
n_mag_raw = 0
dt_acq = 0.05 # 20 Hz

while True:
    cmd = input("command : a,u,c,t,e ? ") 
    if cmd == "a":
        tacq = float(input("duration (s) ? "))
        t0acq = time.time()
        n_acq = 0
        t_acq = []
        while (time.time() - t0acq) < tacq:
            mag_raw_x, mag_raw_y = imu.read_mag_raw()[0:2]
            print (round((time.time() - t0acq)*10.0)/10.0,mag_raw_x, mag_raw_y)
            t_acq.append([mag_raw_x, mag_raw_y])
            n_acq += 1
            time.sleep(dt_acq)
    elif cmd == "e":
        break
