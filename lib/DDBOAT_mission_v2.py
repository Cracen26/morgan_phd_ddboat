#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec
# Date : 25/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library manage the mission script file of the DDBOAT
# There is also a general function to take all the measurements
#################################################

from lib.DDBOAT_controler_v2 import *
from lib.DDBOAT_filter_v2 import DdboatFilter, StateObserver
from lib.DDBOAT_log_v2 import LogRecorder, init_drivers, time, robot_id
import json
import sys


class MissionBlock:
    def __init__(self, rh):  # mission initialisation
        # rh = True if the robot need to return home at the end of the mission
        print("robot setup ...")

        # load mission script
        file_script = open("../mission_script/mission_script.json", "r")
        file_script2 = open("compass_calibration/compass_calibration_ddboat" + robot_id + ".json", "r")
        data_script = json.load(file_script)
        data_script2 = json.load(file_script2)
        param = data_script["mission_param"]
        trajs = data_script["trajectories"]
        mission_robot_id = param["mission_robot_id"]
        try:
            self.traj = trajs[mission_robot_id]  # list of desired pose
            self.Lpd = np.reshape(np.array([self.traj["pd"]]), (2, 1))
        except:
            print("no trajectory loaded")
            self.traj = [[0]]
            self.Lpd = np.zeros((2, 1))
        print("robot id", mission_robot_id)

        # time parameters
        try:
            hour = str(sys.argv[1])
        except:
            hour = "0"
        try:
            minute = str(sys.argv[2])
        except:
            minute = "0"
        t = time.localtime(time.time())
        dtStr = str(t.tm_year) + "-" + str(t.tm_mon) + "-" + str(t.tm_mday) + "-" + hour + "-" + minute + "-" + "0"
        local_time_mission_begin = time.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
        self.time_mission_begin = time.mktime(local_time_mission_begin)
        self.dt = param["dt"]  # loop time period
        self.time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission

        # drivers
        self.ard, self.temperature, self.gps, self.encoddrv, self.imu = init_drivers()

        # filter
        lxm = param["lxm"]
        lym = param["lym"]
        b = np.reshape(np.array([data_script2["b"]]), (3, 1))
        A = np.reshape(np.array([data_script2["A"]]), (3, 3))
        self.filt = DdboatFilter(lxm, lym, A, b, self.encoddrv)

        # Log
        self.log_rec = LogRecorder(local_time_mission_begin)

        # find initial pose
        while True:
            _, _, _, gll_ok, val, _, _, mag, _, _ = self.log_rec.log_observe_update(self.temperature, self.ard,
                                                                                    self.gps, self.encoddrv, self.imu)
            self.log_rec.log_update_write()
            if gll_ok:
                y_th = self.filt.cap(mag[0], mag[1], mag[2])
                lat, lon = self.filt.cvt_gll_ddmm_2_dd(val)
                pos = self.filt.latlon_to_coord(lat, lon)
                X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
                break
            time.sleep(0.1)

        # kalman filter
        Gamma0 = np.diag(param["Gamma0"])
        Gamma_alpha = np.diag(param["Gamma_alpha"])
        Gamma_beta = np.diag(param["Gamma_beta"])
        self.kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, self.dt)

        # set home
        self.return_home = rh  # if true, at the en of the mission, the robot return home
        home_lat, home_lon = param["home_lat"], param["home_lon"]
        self.home_pos = self.filt.latlon_to_coord(home_lat, home_lon)

        # init variables
        self.t, self.sync, self.wmLeft, self.wmRight, self.y_th = 0, False, 0, 0, 0

        print("robot setup done")
        print("---")
        print("mission will begin at", time.asctime(local_time_mission_begin))

    def measure(self, cmdL_old, cmdR_old):
        # measurements
        self.t = time.time()
        _, _, _, gll_ok, val, self.sync, data_encoders, mag, _, _ = self.log_rec.log_observe_update(self.temperature,
                                                                                                    self.ard,
                                                                                                    self.gps,
                                                                                                    self.encoddrv,
                                                                                                    self.imu)
        self.wmLeft, self.wmRight = self.filt.measure_wm(data_encoders, self.dt)
        self.y_th = self.filt.cap(mag[0], mag[1], mag[2])
        if gll_ok:  # correct kalman filter with new gps data
            lat, lon = self.filt.cvt_gll_ddmm_2_dd(val)
            pos = self.filt.latlon_to_coord(lat, lon)
            self.kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)

    def wait_for_next_iteration(self):
        # loop update
        if not self.sync:
            print("arduino communication lost, break !")
            return False # break loop
        if time.time() > self.time_mission_max:
            print("maximum allowed time passed, breaking !")
            return False # break loop

        t_execution = time.time() - self.t
        delta_t = self.dt - t_execution
        if delta_t > 0:
            time.sleep(delta_t)
        else:
            print("LAG loop frequency reduced, t_execution ", t_execution)