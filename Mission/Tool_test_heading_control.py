# test the heading controller from DDBOAT controler

import sys
from lib.DDBOAT_mission_v2 import *


if __name__ == "__main__":
    try:
        heading_d = int(sys.argv[3])
    except:
        heading_d = 90 # deg = north
    print("testing heading control")
    print("desired heading is ", heading_d)
    heading_d_rad = heading_d/180*3.14

    mb = MissionBlock()
    cmdL,cmdR = 0.0,0.0
    print("mission started")
    while True:

        mb.measure(cmdL,cmdR)
        print(" Desired | Current heading is ", heading_d, " , ", 180/3.14*mb.y_th)

        vd = 1.
        wd = control_heading(heading_d_rad,mb.y_th)
        print("wd is ",wd)

        cmdL,cmdR = convert_motor_control_signal(vd,wd,mb.wmLeft,mb.wmRight,cmdL,cmdR,mb.dt)
        mb.ard.send_arduino_cmd_motor(cmdL, cmdR)
        print("cmdL = "+str(cmdL)+" | cmdR = "+str(cmdR))
        print("---")
        if not mb.wait_for_next_iteration():
            break
    print("mission ended")
    mb.ard.send_arduino_cmd_motor(0, 0)
