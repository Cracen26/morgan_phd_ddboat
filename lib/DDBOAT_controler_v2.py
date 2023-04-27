#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 23/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library for the controllers of the DDBoat
# heading controllers (wd) :
# proportional / control_heading(th_d, th)
# point following / control_follow_point(m, b, th)
# line following / control_follow_line(a, b, m)
# circle consensus / ConsensusController
#
# speed + heading controller (vd,wd):
# station keeping / control_station_keeping(a, d, pos, th, v=1)
#
# vector field:
# point consensus / point_consensus(pc, Lpc, d)
# repulsion / vf_repulsion(p, Lp)
# circle attraction / vf_circle(p, e, rho, c)

# tool functions:
# Lissajou curve / control_lisssajou(...)
# convert (vd,wd) to (cmdL,cmdR) / convert_motor_control_signal(vd, wd, wmLeft, wmRight, cmdL_old, cmdR_old,dt)
#################################################

import numpy as np
from math import sin, cos, sqrt

kT, kpwm, kD, kw = 0.01, 1.5, 15, 0.06  # kpwm verified
# m = 2.5  # weight of the DDBOAT (verified)
umax = 200
# wlrmax = umax / kpwm
# vmax = sqrt(2 * kT / kD) * wlrmax
# wmax = sqrt(kw * kT) * wlrmax
# rmax = vmax / wmax
# amax = 2 * kT * umax ** 2 / (m * kpwm ** 2)
# kp, kd, kth = 1, 2, 1
# K_inv = 1 / (2 * kT) * np.array([[m, -1 / kw], [m, 1 / kw]])  # [wl*wl wr*wr].T = K_inv * ([v_dot, w*|w|].T + D/m)
B = kT * np.array([[1 / kD, 1 / kD], [-kw, kw]])  # [v*v,w*|w|] = B * [wl*wl wr*wr].T
B_inv = np.linalg.inv(B)


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def control_heading(th_d, th):
    # proportional heading regulation
    # th_d: desired heading (rad)
    # th: current heading (rad)
    # return the desired angular speed (rad/s)
    K = 1.  # controller gain
    wd = K * sawtooth(th_d - th)
    return wd


def heading_follow_point(b, m):
    # compute the desired heading to follow the point b
    # b: target position
    # m: current position
    return np.arctan2(b[1, 0] - m[1, 0], b[0, 0] - m[0, 0])


def control_follow_line(a, b, m):
    # compute the desired heading to follow the line [ab)
    # a: beginning of the line
    # b: end of the line
    # m: current position
    r = 4  # gain
    u = (b - a) / np.linalg.norm(b - a)  # unit vector of the line
    e = np.linalg.det(np.hstack([u, m - a]))  # distance error with the line
    phi = np.arctan2(b[1, 0] - a[1, 0], b[0, 0] - a[0, 0])  # orientation of the line
    print("phi", phi)
    print("correct", np.arctan(e / r))
    ang = sawtooth(phi - np.arctan(e / r))  # desired heading
    # print("desired ANG (deg)", ang * 180 / np.pi)
    return ang


def lissajou_trajectory(c, R, T, t, i, N):
    # compute the Lissajou trajectory
    # c : center of the curve
    # R : amplitude of the curve
    # T : period of the trajectory
    # t : current time since the beginning of the mission
    # i : id of the robot
    # N : total number of robots
    phi1, phi2 = t * np.pi * 2 / T + 2 * i * np.pi / N, 2 * (t * np.pi * 2 / T + 2 * i * np.pi / N)
    pd = c + R * np.array([[np.cos(phi1)],
                           [np.sin(phi2)]])
    pd_dot = R * np.array([[-np.pi * 2 / T * np.sin(phi1)],
                           [np.pi * 4 / T * np.cos(2 * phi2)]])
    pd_ddot = R * np.array([[-(np.pi * 2 / T) ** 2 * np.cos(phi1)],
                            [-(np.pi * 4 / T) ** 2 * np.sin(phi2)]])
    return pd, pd_dot, pd_ddot


def point_consensus(pc, Lpc, d):
    # pc: position of the circle center
    # Lpc: list of the position of the other circle center
    # d: desired distance
    # return the vector field of center of the circle
    n = len(Lpc)
    f = np.array([[0., 0.]]).T
    for i in range(0, n):
        nor = np.linalg.norm(pc - Lpc[i])
        if nor > 0.001:
            f = f + (Lpc[i] - pc) * (nor - d) / nor
    return f


def vf_repulsion(p, Lp):
    # p: position of the boat
    # Lp: list of the position of the other boat
    # return the repulsion vector field of the boat
    alpha = 100
    n = len(Lp)
    f = np.array([[0., 0.]]).T
    for i in range(0, n):
        nor = np.linalg.norm(p - Lp[i])
        if nor >0.001:
            f = f + alpha * (-Lp[i] + p) / nor ** 2
    return f


def vf_circle0(p1, p2):
    # vector field of the unit circle
    return np.array([[-p1 ** 3 - p1 * p2 ** 2 + p1 - p2], [-p2 ** 3 - p1 ** 2 * p2 + p1 + p2]])


def vf_circle(p, e, rho, c):
    # vector field of the circle
    # p: position of the boat
    # e: 1 clockwise, -1 counterclockwise
    # rho: radius of the circle
    # c: center of the circle
    D = np.array([[rho, 0], [0, rho * e]])
    z = np.linalg.inv(D) @ (p - c)
    return D @ vf_circle0(z[0, 0], z[1, 0])


def control_follow_point(m, b, th):
    # controller to follow a point
    # m: current position
    # b: target position
    # th: current heading
    thd = heading_follow_point(b, m)
    wd = control_heading(thd, th)
    return wd


def control_station_keeping(a, d, pos, th, v=1):
    # controller to keep the robot next ot a position
    # a: reference position (local position, m)
    # d: max distance desired (m)
    # pos : current position of the robot (local position, m)
    # th:  current heading (rad)
    # return the desired speed (m/s) and the desired angular velocity (rad)

    d_mes = np.linalg.norm(pos - a)
    if d_mes > d:
        th_d = heading_follow_point(a, pos)
        return v, control_heading(th_d, th)
    else:
        return 0, 0

class ControlStationKeeping:

    def __init__(self,state_init,pd0,r0):
        self.state = 0  # 0 : move towards waypoint , 1 : move towards desired heading, 2 : stop
        self.pd0 = pd0  # target waypoint
        self.r = r0 # radius of the station keeping
    def control_station_keeping_2(self,p,th):  # basic station keeping with in and out circle
        # p : current position of the robot
        # change state
        if self.state == 0 and np.linalg.norm(self.pd0 - p) < self.r / 2:
            self.state = 1
            # self.t_burst = t
        if self.state == 1 and np.linalg.norm(self.pd0 - p) > self.r:
            self.state = 0

        # control
        if self.state == 0:
            vd = 1
            wd = control_follow_point(p,self.pd0,th)
        else:
            vd = 0
            wd = 0
        return vd, wd

def convert_motor_control_signal(vd, wd, wmLeft, wmRight, cmdL_old, cmdR_old,
                                 dt):
    # compute pwd control signal for the motors
    # vd desired speed (m/s)
    # wd desired angular speed (rad/s)
    # wmLeft, wmRight : measured rotation speed of the motors (turn/sec)
    # cmdL_old, cmdR_old : previous command (PWM)
    # dt : time step (s)

    wm_sqr = B_inv @ (np.array([[vd * vd], [wd * abs(wd)]]))  # [wl**2 , wr**2]
    wm_sqr[0, 0] = max(0, wm_sqr[0, 0])
    wm_sqr[1, 0] = max(0, wm_sqr[1, 0])
    wmLeft_d, wmRight_d = sqrt(wm_sqr[0, 0]), sqrt(wm_sqr[1, 0])

    # discrete proportional integral corrector for Pwm
    cmdL = min(max(cmdL_old + dt * kpwm * (wmLeft_d - wmLeft), 0), 200)
    cmdR = min(max(cmdR_old + dt * kpwm * (wmRight_d - wmRight), 0), 200)

    # security saturation of the output
    cmdL = max(0, min(umax, cmdL))
    cmdR = max(0, min(umax, cmdR))
    return cmdL, cmdR  # controlled PWM


class ConsensusController:  # controller for the consensus of the DDBOAT circles
    # the DDBOAT move in a cirlce trajectory
    # the center of their trajectory must converge
    def __init__(self, p0, th0, rho):
        self.rho = rho  # radius of the circle

        # center of the circle initialized 10m north from the robot
        self.c = self.pc = np.array([[p0[0, 0], p0[1,0]+self.rho]]).T

    def compute_angular_speed(self, p, th, Lp):
        # compute the desired angular speed of the robot
        # wd: desired angular speed (rad/s)

        vfc = vf_circle(p, 1, self.rho, self.c)  # vector field of the circle
        vfr = vf_repulsion(p, Lp)  # vector field of the repulsion
        scal = (vfr.T @ np.array([[cos(th), sin(th)]]).T)[0, 0]
        scal = max(-1., min(0., scal) / 2.) # increase circle radius if asked to slow down = lower angular speed
        vf = vfc  # vector field followed by the robot
        thd = np.arctan2(vf[1, 0], vf[0, 0])  # desired heading
        wd = control_heading(thd, th)+scal  # desired angular speed
        return wd

    def update_point_center(self, Lc, dt):
        # update the center of the circle
        # Lc: list of the position of the other circle center
        self.c = self.c + dt*point_consensus(self.c, Lc, 0)