# main script of the anger mission, the robot must follow a desired timed trajectory

from DDBOAT_controler_v1 import *
from DDBOAT_filter_v1 import *
from log_driver import LogRecorder, init_drivers, time
import json

#######################
# robot setup
#######################

print("robot setup ...")

# load mission script
file_script = open("angers_mission_script.json", "r")
data_script = json.load(file_script)
param = data_script["mission_param"]
traj = data_script["trajectory"]

ard, temperature, gps, encoddrv, imu = init_drivers()
# ~ synchronise_time_on_gps(gps) 
        
log_rec = LogRecorder()
lxm = param["lxm"]
lym = param["lym"]
b = np.reshape(np.array([param["b"]]), (3, 1))
A = np.reshape(np.array([param["A"]]), (3, 3))

filt = DdboatFilter(lxm, lym, A, b, encoddrv)

Gamma0 = np.diag(param["Gamma0"])
Gamma_alpha = np.diag(param["Gamma_alpha"])
Gamma_beta = np.diag(param["Gamma_beta"])
while True: # find initial pose
    _, _, _, gll_ok, val, _, _, mag, _, _ = log_rec.log_observe_update(temperature, ard, gps, encoddrv, imu)
    log_rec.log_update_write()
    if gll_ok:
        y_th = filt.cap(mag[0], mag[1], mag[2])
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        pos = filt.latlon_to_coord(lat, lon)
        X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
        break
    time.sleep(0.1)

dt = param["dt"]  # 10hz
print("initial state is", X0.T)
kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, dt)
print("robot setup done")

#####################
# wait for mission beginning
#####################
# ~ print("wait for the beginning of the mission")
time_mission_begin = param["time_mission_begin"]
# ~ while time.time() < time_mission_begin:
    # ~ time.sleep(1)
time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission
# ~ print("the mission begins")

#####################
# mission loop
#####################
mission = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, None, kal.p(), time.time(), 0, 0

print("Going to the initial waypoint")
wait_for_signal = True # after time mission begin, the robot start following the trajectory 

while mission:

    # measurements
    t = time.time()
    tl, tr, data, gll_ok, val, sync, data_encoders, mag, accel, gyro = log_rec.log_observe_update(temperature, ard, gps,
                                                                                                  encoddrv, imu)
    wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
    print(wmLeft,wmRight)
    print(data_encoders)
    y_th = filt.cap(mag[0], mag[1], mag[2])
    if gll_ok:  # correct kalman filter with new gps data
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        pos = filt.latlon_to_coord(lat, lon)
        kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)

    # reference update
    if wait_for_signal: # stand by initial waypoint
        traj_0 = traj[0]
        pd = np.reshape(np.array([traj_0["pd"]]), (2, 1))
        pd_dot,pd_ddot = np.zeros((2,1)), np.zeros((2,1))
        if np.linalg.norm(pd-kal.p())<3: # don't move if under 3m of distance to the waypoint
            pd = kap.p()
        if time.time() > time_mission_begin: # after time mission begin, the robot start following the trajectory 
            wait_for_signal = False
            print("Start following the trajectory")
    else:
        try:
            traj_k = traj[k]
            pd = np.reshape(np.array([traj_k["pd"]]), (2, 1))
            pd_dot = np.reshape(np.array([traj_k["pd_dot"]]), (2, 1))
            pd_ddot = np.reshape(np.array([traj_k["pd_ddot"]]), (2, 1))
            k += 1
        except:  # end of the trajectory
            print("end of the trajectory, break !")
            break

    # controler update
    u = control_feedback_linearization(pd, pd_dot, pd_ddot, dt, p=kal.p(), v=kal.X[2, 0], th=y_th, qx=kal.X[3, 0],
                                       qy=kal.X[4, 0])

    cmdL, cmdR = convert_motor_control_signal(u, kal.X[2, 0], wmLeft, wmRight, cmdL, cmdR, dt)
    ard.send_arduino_cmd_motor(0*cmdL, 0*cmdR)
    log_rec.log_control_update(u[0, 0], u[1, 0], wmLeft, wmRight, cmdL, cmdR, pd, y_th, kal)
    kal.Kalman_update(0*u, y_th)
    log_rec.log_update_write()  # write in the log file
    
    # loop update
    if not sync:
        print("arduino communication lost, break !")
        break
    if time.time() > time_mission_max:
        print("maximum allowed time passed, breaking !")
        break

    t_execution = time.time() - t
    delta_t = dt - t_execution
    if delta_t > 0:
        time.sleep(delta_t)
    else:
        print("LAG loop frequency reduced, t_execution ", t_execution)

ard.send_arduino_cmd_motor(0, 0)
