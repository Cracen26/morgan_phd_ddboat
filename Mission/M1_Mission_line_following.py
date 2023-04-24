# M1 - the robot follow a list of lines defined in the waypoint_list variable

from lib.DDBOAT_controler_v2 import *
from lib.DDBOAT_mission_v2 import *

mp = MissionBlock(rh=True)  # trajectory is useless
# init variables
k, cmdL, cmdR = 0, 0, 0

waypoint_list = np.array(
    [[48.199024, 48.199038, 48.199817], [-3.014790, -3.015807, -3.015603]])  # home, west, north-west
print("waypoint list:", waypoint_list)

# convert waypoint to local coordinate
for i in range(len(waypoint_list[0])):
    pos = mp.filt.latlon_to_coord(waypoint_list[0, i], waypoint_list[1, i])
    waypoint_list[:, [i]] = pos
print("waypoint list local:", waypoint_list)

while time.time() < mp.time_mission_max:

    mp.measure(cmdL, cmdR)
    print("current position:", mp.kal.p().T)

    # test change of line
    a = waypoint_list[:, [k]]
    b = waypoint_list[:, [k + 1]]
    if np.dot((b - a).flatten(), (mp.kal.p() - b).flatten()) > 0:  # if the boat has passed the end of the line
        print("next line")
        k += 1
        if k > (len(waypoint_list[0]) - 2):
            print("mission done")
            break
    print(" ")

    # update reference
    pd = mp.Lpd[k]
    thd = control_follow_line(a, b, mp.kal.p())
    v_d = 5  # m/s

    print("th", mp.y_th)
    print("th_d:", thd)
    print("b-a", (b - a).T)
    print("m-a", (mp.kal.p() - a).T)

    vd = 1.
    wd = control_heading(thd, mp.kal.th)
    cmdL, cmdR = convert_motor_control_signal(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, b, mp.y_th, mp.kal)
    mp.kal.Kalman_update(np.zeros((2, 1)), mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break

mp.ard.send_arduino_cmd_motor(0, 0)
