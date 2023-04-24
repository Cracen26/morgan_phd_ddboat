#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 25/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library to filter the measurements of the DDBOATs
#################################################
import numpy as np
from math import cos, sin, pi
from lib.DDBOAT_log_v2 import encoder_read
import time



def sawtooth(x):
    return (x + pi) % (2 * pi) - pi  # or equivalently   2*arctan(tan(x/2))


def taking_a_measurement(imu):
    test = input()
    x1 = np.zeros((3, 1))
    if test == "y":
        x1 = np.array([imu.read_mag_raw()]).T  # [0]
    print("measuring ", x1.T)
    return x1
    
def delta_odo (odo1,odo0): # TODO remise à 0 par le sinus car bug quand 2 remises à 0 ?
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    # dodo = (2*pi/(2**16)) * (odo1-odo0) # doit être dans [-pi,pi]
    # dodo = ((2**16)/(2*pi)) * sin(dodo) # revient au valeurs de l'encodeur
    return dodo



class DdboatFilter:
    def __init__(self, lxm, lym, A, b, encoddrv):
        print("filter initialization")
        data_encoders = encoder_read(encoddrv.get_last_value_v2())
        print("initial data encoder is ", data_encoders)
        self.odoLeft, self.odoRight = data_encoders[3], data_encoders[4]  # memorised position of the motors' encoder
        self.lastOdoTime = data_encoders[0] # time of the last odometer measurement
        self.wmLeft, self.wmRight = 0.0, 0.0 # memorised speed of the motors
        print("Initial odometers are ", self.odoLeft,"/", self.odoRight)
        self.lxm, self.lym = lxm, lym,  # longitude and latitude of the origin
        self.rho = 6371009  # radius of the earth (m)
        self.A, self.b = A, b  # calibration parameters of the compass
        self.beta = 46000  # nT, norm of the magnetic field
        print("filter initialized")

    def measure_wm(self, data_encoders, t_boucle):  # measure the rational speed of the motors (turn/sec)
        # data_encoders : data from encod.read_packet()
        # t_boucle : period of the update (s)
        odoTime = data_encoders[0]
        dTime = odoTime - self.lastOdoTime
        if abs(dTime) > 1: # update speed every 2 odo measurement
            odoLeft, odoRight = data_encoders[3], data_encoders[4]  # 0 to 65536=2**16 tics
            dodoLeft = abs(delta_odo(odoLeft,self.odoLeft)) # always positive
            dodoRight = abs(delta_odo(odoRight,self.odoRight))
            self.odoLeft, self.odoRight = odoLeft, odoRight
            self.wmLeft = dodoLeft / 8 / (0.1*dTime)  # 8 tics = 1 turn
            self.wmRight = dodoRight / 8 / (0.1*dTime)
            self.lastOdoTime = odoTime
        return self.wmLeft, self.wmRight

    @staticmethod
    def cvt_gll_ddmm_2_dd(val):  # get lat lon from gps raw data val
        ilat, ilon = val[0], val[2]
        olat = float(int(ilat / 100))
        olon = float(int(ilon / 100))
        olat_mm = (ilat % 100) / 60
        olon_mm = (ilon % 100) / 60
        olat += olat_mm
        olon += olon_mm
        if val[3] == "W":
            olon = -olon
        return olat, olon

    def latlon_to_coord(self, lat, lon):
        pos = np.array(
            [[self.rho * np.cos(lat * np.pi / 180) * (lon - self.lxm) * np.pi / 180],
             [self.rho * (lat - self.lym) * np.pi / 180]])
        return pos  # x in lon, y in lat

    def compass_calibration(self, imu):  # from student
        # imu : Imu9IO driver
        print("Calibration de la Boussole")

        fichier = open("../compass_calibration.txt", "w")
        n_satisfait = 1  # boolean flag to stop procedure

        # Boucle de demande de mesure à l'utilisateur
        while n_satisfait:
            print("Allignez le champ magnetic sur axe x.  Prêt mesure ? (y/n)")
            x1 = taking_a_measurement(imu)

            print("Allignez le champ magnetic sur axe -x. Prêt mesure ? (y/n)")
            x_1 = taking_a_measurement(imu)

            print("Allignez le champ magnetic sur axe y.  Prêt mesure ? (y/n)")
            x2 = taking_a_measurement(imu)

            print("Allignez le champ magnetic sur axe z.  Prêt mesure ? (y/n)")
            x3 = taking_a_measurement(imu)

            print("Satisfait ? (y/n)")
            test_final = input()
            if test_final == "y":
                n_satisfait = 0

        fichier.close()

        # Calcul de calibration de la boussole, A et b
        try:
            # beta = 46000  # nT, norm of the magnetic field
            self.b = -(x1 + x_1) / 2
            X = np.hstack((x1 + self.b, x2 + self.b, x3 + self.b))
            self.A = (1 / self.beta) * X
        except:
            print("ERROR : missing some measurements !!!")

    # def compass_auto_calibration(self, imu):  # when the magnetic field direction is unknown
    #
    #     m_list = np.array(
    #         [[-893, 972, 2637], [2254, 679, 619], [-2310, 1401, -847], [-2135, -1158, 2278], [1076, -584, 2423],
    #          [-418, -2157, -2582]]).T
    #
    #     print("Compas Calibration : move the DDBOAT in all directions")
    #     dt_, N, mag_list = 0.5, 30, []
    #     for k in range(N):
    #         mag_list.append(np.array([imu.read_mag_raw()]).T)
    #         time.sleep(dt_)
    #     print("Measurement done, Calibrating")
    #
    #     b = (m_list[:, [0]] - m_list[:, [1]] - m_list[:, [2]]
    #          - m_list[:, [3]] - m_list[:, [4]] - m_list[:, [5]]) / 4  # (m2+m3+m4+m5+m6-m1)/6
    #
    #     M = np.zeros((N, 6))  # M@q = ones
    #     ones = np.ones((N, 1))
    #     for i in range(N):
    #         z = (mag_list[i] + b) / self.beta
    #         zx, zy, zz = z[0, 0], z[1, 0], z[2, 0]
    #         M[i] = [zx * zx, 2 * zx * zy, 2 * zx * zz, zy * zy, zy * zz, zz * zz]  # line i
    #     q = np.linalg.inv(M.T @ M) @ M.T @ ones  # pseudo inv
    #     Q = np.array([[q[0, 0], q[1, 0], q[2, 0]], [q[1, 0], q[3, 0], q[4, 0]], [q[2, 0], q[4, 0], q[5, 0]]])
    #     d, P = np.linalg.eig(Q)
    #     Gamma_Q = np.linalg.inv(P @ np.sqrt(np.diag(d)) @ np.linalg.inv(P))
    #     A = Gamma_Q
    #     print("Q is ", Q)
    #     print("eig Q is", d)
    #     print("A is ", A.flatten())
    #     print("b is ", b.T)
    #     print("Compass calibration done")
    #     return

    def cap(self, magx, magy, magz):  # is 0 when facing east, trigonometric sign
        # magx, magy, magz : magnetic vector from imu.read_mag_raw()
        x = np.array([[magx, magy, magz]]).T
        magx, magy, magz = np.linalg.inv(self.A) @ (x + self.b).flatten()  # correction of the magnetic field
        return sawtooth(-np.arctan2(magy, magx) + pi / 2)

class StateObserver:
    # Extended Kalman filter to estimate position and speed
    def __init__(self, X0: np.array, th0: float, Gamma0: np.array, Gamma_alpha: np.array, Gamma_beta: np.array,
                 dt: float):
        self.X = X0  # initial state (x,y,v,qx,qy)
        self.Gamma = Gamma0
        self.Gamma_alpha = Gamma_alpha
        self.Gamma_beta = Gamma_beta
        self.dt = dt  # loop period
        self.u = np.array([[0, 0]]).T
        self.y = np.array([[0, 0]]).T
        self.th = th0  # measurement of the heading
        self.Ck = np.array([[1, 0, 0, 0,0], [0, 1, 0, 0,0]])

    def p(self):  # robot position
        return self.X[0:2, :]

    def p_dot(self):  # robot speed vector
        return np.array([[self.X[2, 0] * cos(self.th) + self.X[3, 0], self.X[2, 0] * sin(self.th) + self.X[4, 0]]]).T

    def Kalman_update(self, u,th):  # time update based on the movement of the robot
        # u : [acceleration, angular velocity] control signal
        self.u,self.th = u,th
        Ak = np.eye(5) + self.dt * np.array(
            [[0, 0, cos(self.th), 1, 0],
             [0, 0, sin(self.th), 0, 1],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0,0],
             [0, 0, 0, 0, 0]])
        self.X = self.X + self.dt * np.array([[self.X[2,0] * cos(self.th)+self.X[3,0]],
                                              [self.X[2,0] * sin(self.th)+self.X[4,0]],
                                              [u[0, 0]],
                                              [0],
                                              [0]])
        self.Gamma = Ak @ self.Gamma @ Ak.T + self.Gamma_alpha
        return

    def Kalman_correct(self, y):  # measurement correction of the kalman filter
        # y : measure [x,y]
        self.y = y
        Zk = y - self.Ck @ self.X # g(x) is linear
        Sk = self.Ck @ self.Gamma @ self.Ck.T + self.Gamma_beta
        Kk = self.Gamma @ self.Ck.T @ np.linalg.inv(Sk)
        self.X = self.X + Kk @ Zk
        self.Gamma = (np.eye(5) - Kk @ self.Ck) @ self.Gamma
        return
