# 19 avril 2023
# Same as M5 but this DDBoats is a Byzantine robot
# It will be manually contolled by the remote control
# The center point sent to the other robot will be the position of the robot
# this robot should disturb the consensus

from lib.DDBOAT_mission_v2 import *
from lib.DDBOAT_client import Client

# init robot
mp = MissionBlock()
cmdL, cmdR = 0, 0

p0 = mp.kal.p()  # initial position
th0 = mp.y_th  # initial heading

my_data0 = [robot_id, mp.kal.p()[0,0], mp.kal.p()[1,0], mp.kal.p()[0,0], mp.kal.p()[1,0]]
cl = Client(my_data0)
cl.start()
while time.time() < mp.time_mission_max:
    mp.measure(cmdL, cmdR)

    # communication
    my_data = [robot_id, mp.kal.p()[0,0], mp.kal.p()[1,0], mp.kal.p()[0,0], mp.kal.p()[1,0]]
    cl.my_data = my_data

    print("my_data", my_data)
    print("other_data", cl.other_data)

    vd = 0
    wd = 0
    cmdL, cmdR = 0 , 0

    # log update
    c = mp.kal.p()
    print("c is ", c.T)
    print(" --- ")
    mp.log_rec.log_control_update(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, c, mp.y_th,
                                  mp.kal)
    mp.kal.Kalman_update(mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break

mp.auto_home(cmdL,cmdR)
mp.ard.send_arduino_cmd_motor(0, 0)
cl.join()
