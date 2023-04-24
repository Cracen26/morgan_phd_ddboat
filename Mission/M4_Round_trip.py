# M4 - Round trim between the original point and a target

from lib.DDBOAT_mission_v2 import *

mp = MissionBlock(rh=True)

# init variables
k, pd0, pos_old, t_pos_old, cmdL, cmdR = 0, mp.Lpd[:,[0]], mp.kal.p(), time.time(), 0, 0
step = 1

# get desired position (pd0 : initial position, pd1: desired position)
lat1 = int(sys.argv[1])
lon1 = int(sys.argv[2])
pd1 = mp.filt.latlon_to_coord(lat1, lon1)
pd = pd1

while time.time() < mp.time_mission_max:

    mp.measure(cmdL, cmdR)  # measurement

    # reference update
    if step == 1 and np.linalg.norm(mp.kal.p() - pd) < 2:  # return home when reaching target
        pd = pd0
        step += 1
    elif step == 2 and np.linalg.norm(mp.kal.p() - pd) < 2:
        print("end of the mission")
        break

    # control update
    vd = 1.
    wd = control_follow_point(mp.kal.p(), pd, mp.kal.th)
    cmdL,cmdR = convert_motor_control_signal(vd,wd,mp.wmLeft,mp.wmRight,cmdL,cmdR,mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, pd, mp.y_th, mp.kal)
    mp.kal.Kalman_update(0, mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break
mp.ard.send_arduino_cmd_motor(0, 0)
