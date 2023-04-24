# test the control of the motor in speed

import sys
from lib.DDBOAT_filter_v2 import *
from lib.DDBOAT_log_v2 import init_drivers, time

if __name__ == "__main__":
    try:
        wmLeft_d = int(sys.argv[1])
    except:
        wmLeft_d = 50
    try:
        wmRight_d = int(sys.argv[2])
    except:
        wmRight_d = 75
    print("testing motor control")

    ard, _, _, encoddrv, _ = init_drivers()
    kpwm = 1.5
    filt = DdboatFilter(0, 0, np.zeros((3,3)), np.zeros((3,1)), encoddrv)

    dt = 0.1  # 10hz
    cmdL_old,cmdR_old = 0, 0
    print("mission started")
    while True:

        # measurements
        t = time.time()
        try:
            sync, data_encoders = encoddrv.read_packet()
        except:
            sync = False
            data_encoders = []
            print("can't log the encoders")

        wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
        print(" --- ")
        print("wmLeftd = "+str(wmLeft_d)+" | wmRightd = "+str(wmRight_d))
        print("wmLeft = "+str(wmLeft)+" | wmRight = "+str(wmRight))
        print("data_encodeur ",data_encoders)

        cmdL = min(max(cmdL_old + dt * kpwm * (wmLeft_d - wmLeft), 0), 200)
        cmdR = min(max(cmdR_old + dt * kpwm * (wmRight_d - wmRight), 0), 200)
        print("cmdL = "+str(cmdL)+" | cmdR = "+str(cmdR))
        ard.send_arduino_cmd_motor(cmdL, cmdR)
        cmdL_old,cmdR_old = cmdL,cmdR

        # loop update
        if not sync:
            print("arduino communication lost, break !")
            break

        t_execution = time.time() - t
        delta_t = dt - t_execution
        if delta_t > 0:
            time.sleep(delta_t)
        else:
            print("LAG loop frequency reduced, t_execution ", t_execution)
    print("mission ended")
    ard.send_arduino_cmd_motor(0, 0)
