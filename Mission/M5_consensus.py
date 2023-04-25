# 19 avril 2023
# mission script for consensus of DDBoats
# by default, the DDBOATs turn around the point "c" with a defined radius "rho"
# the DDBOATs can communicate their point "c" and their position "p"
# the point c follow a one order consensus to converge toward the other c points
# the DDOBOATs follow two vector fields:
# - vfc to follow their cirle
# - vfr to be repulsed from the other DDBOATs

from lib.DDBOAT_mission_v2 import *
from lib.DDBOAT_client import Client

# init robot
mp = MissionBlock()
cmdL, cmdR = 0, 0

p0 = mp.kal.p()  # initial position
th0 = mp.y_th  # initial heading
rho = 10  # m circle radius
CONS = ConsensusController(p0, th0, rho)

my_data0 = [robot_id, mp.kal.p()[0,0], mp.kal.p()[1,0], CONS.c[0,0], CONS.c[1,0]]
cl = Client(my_data0)
cl.start()
while time.time() < mp.time_mission_max:
    mp.measure(cmdL, cmdR)

    # communication
    my_data = [robot_id, mp.kal.p()[0,0], mp.kal.p()[1,0], CONS.c[0,0], CONS.c[1,0]]
    cl.my_data = my_data
    other_data = cl.other_data

    print("my_data", my_data)
    print("other_data", other_data)

    # controller update
    Lp = [mp.kal.p(), mp.kal.p()]  # TODO update to have the measurement of the other robots
    vd = 1.
    wd = CONS.compute_angular_speed(mp.kal.p(), mp.kal.th, Lp)
    Lc = [CONS.c, CONS.c]  # TODO update to have the center of the other robots
    CONS.update_point_center(Lc, mp.dt)  # update the center of the circle
    cmdL, cmdR = convert_motor_control_signal(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, mp.dt)
    # mp.ard.send_arduino_cmd_motor(cmdL, cmdR)

    # log update
    mp.log_rec.log_control_update(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, CONS.c, mp.y_th,
                                  mp.kal)
    mp.kal.Kalman_update(mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break

mp.auto_home(cmdL,cmdR)
mp.ard.send_arduino_cmd_motor(0, 0)
cl.join()
