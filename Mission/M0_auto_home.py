# M0 - the robot return the home gps position defined in the mission script

from lib.DDBOAT_controler_v2 import *
from lib.DDBOAT_mission_v2 import MissionBlock
import time

#######################
# robot setup
#######################

print("robot setup ...")
mb = MissionBlock(rh=True)

# # load mission script
# file_script = open("../mission_script/mission_script.json", "r")
# file_script2 = open("compass_calibration/compass_calibration_ddboat"+robot_number+".json", "r")
# data_script = json.load(file_script)
# data_script2 = json.load(file_script2)
# param = data_script["mission_param"]
# trajs = data_script["trajectories"]
# mission_robot_id = param["mission_robot_id"]
# traj = trajs[mission_robot_id]
# print("robot id", mission_robot_id)
#
# try:
#     hour = str(sys.argv[1])
# except:
#     hour = "0"
# try:
#     minute = str(sys.argv[2])
# except:
#     minute = "0"
# t = time.localtime(time.time())
# dtStr = str(t.tm_year) + "-" + str(t.tm_mon) + "-" + str(t.tm_mday) + "-" + hour + "-" + minute + "-" + "0"
# local_time_mission_begin = time.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
# time_mission_begin = time.mktime(local_time_mission_begin)
#
# ard, temperature, gps, encoddrv, imu = init_drivers()
# log_rec = LogRecorder(local_time_mission_begin)
# lxm = param["lxm"]
# lym = param["lym"]
# b = np.reshape(np.array([data_script2["b"]]), (3, 1))
# A = np.reshape(np.array([data_script2["A"]]), (3, 3))
#
# filt = DdboatFilter(lxm, lym, A, b, encoddrv)
#
# Gamma0 = np.diag(param["Gamma0"])
# Gamma_alpha = np.diag(param["Gamma_alpha"])
# Gamma_beta = np.diag(param["Gamma_beta"])
# while True:  # find initial pose
#     _, _, _, gll_ok, val, _, _, mag, _, _ = log_rec.log_observe_update(temperature, ard, gps, encoddrv, imu)
#     log_rec.log_update_write()
#     if gll_ok:
#         y_th = filt.cap(mag[0], mag[1], mag[2])
#         lat, lon = filt.cvt_gll_ddmm_2_dd(val)
#         pos = filt.latlon_to_coord(lat, lon)
#         X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
#         break
#     time.sleep(0.1)
#
# dt = param["dt"]  # 10hz
# # ~ print("initial state is", X0.T)
# kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, dt)
# print("robot setup done")
# print("---")
#
# time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission
# print("mission will begin at", time.asctime(local_time_mission_begin))
#
# return_home = True # if true, at the en of the mission, the robot return home
# home_lat, home_lon = param["home_lat"], param["home_lon"]
# home_pos = filt.latlon_to_coord(home_lat, home_lon)

#####################
# mission loop
#####################
mission = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mb.home_pos, mb.kal.p(), time.time(), 0, 0
print("Going Home")
# wait_for_signal = True  # after time mission begin, the robot start following the trajectory

while mission:

    # measurements
    mb.measure(cmdL,cmdR)

    # tl, tr, data, gll_ok, val, sync, data_encoders, mag, accel, gyro = log_rec.log_observe_update(temperature, ard, gps,
    #                                                                                               encoddrv, imu)
    # wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
    # y_th = filt.cap(mag[0], mag[1], mag[2])
    # if gll_ok:  # correct kalman filter with new gps data
    #     lat, lon = filt.cvt_gll_ddmm_2_dd(val)
    #     pos = filt.latlon_to_coord(lat, lon)
    #     kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)

    # update reference
    d = np.linalg.norm(pd-mb.kal.p())
    print("home distance",d)
    if d<10:
        print("I am home !!!")
        break

    # control update
    vd = 1
    wd = control_follow_point(mb.kal.p(),pd,mb.kal.th)
    cmdL, cmdR = convert_motor_control_signal(vd,wd,mb.wmLeft,mb.wmRight,cmdL,cmdR,mb.dt)
    mb.ard.send_arduino_cmd_motor(cmdL, cmdR)

    mb.log_rec.log_control_update(vd, wd, mb.wmLeft, mb.wmRight, cmdL, cmdR, pd, mb.y_th, mb.kal)
    mb.kal.Kalman_update(0, mb.y_th) # kalman prediction
    mb.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mb.wait_for_next_iteration():
        break

mb.ard.send_arduino_cmd_motor(0, 0)
