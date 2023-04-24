# M3 follow a list of waypoints defined in the mission script

from lib.DDBOAT_mission_v2 import *

mp = MissionBlock()
# init variables
k, pos_old, t_pos_old, cmdL, cmdR = 0, mp.kal.p(), time.time(), 0, 0
pd = mp.Lpd[:,[0]]

wait_for_signal = True  # after time mission begin, the robot start following the trajectory

SK = ControlStationKeeping(0,pd,5)

while time.time() < mp.time_mission_max:

    mp.measure(cmdL,cmdR)

    # control update
    if wait_for_signal:  # stand by initial waypoint
        vd,wd = SK.control_station_keeping_2(mp.kal.p(),mp.kal.th)
        cmdL, cmdR = convert_motor_control_signal(vd,wd,mp.wmLeft,mp.wmRight,cmdL,cmdR,mp.dt)
        if time.time() > mp.time_mission_begin:  # after time mission begin, the robot start following the trajectory
            wait_for_signal = False
            print("Start following the trajectory")
    else:  # follow reference

        # update reference
        try:
            traj_k = mp.traj[k]
            pd = np.reshape(np.array([traj_k["pd"]]), (2, 1))
            # pd_dot = np.reshape(np.array([traj_k["pd_dot"]]), (2, 1))
            # pd_ddot = np.reshape(np.array([traj_k["pd_ddot"]]), (2, 1))
            k += 1
        except:  # end of the trajectory
            if np.linalg.norm(mp.home_pos-mp.kal.p())>20: # return home
                print("end of the trajectory, return home")
                pd = mp.home_pos
                # pd_dot, pd_ddot = np.zeros((2,1)), np.zeros((2,1))
            else:
                print("end of the mission, break !")
                break

        vd = 1.
        wd = control_follow_point(mp.kal.p(),pd,mp.kal.th)
        cmdL, cmdR = convert_motor_control_signal(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(vd, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, pd, mp.y_th, mp.kal)
    mp.kal.Kalman_update(0, mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break

mp.auto_home(cmdL,cmdR)
mp.ard.send_arduino_cmd_motor(0, 0)
