# M2 square mission, add 90deg to heading reference every 30sec

from lib.DDBOAT_mission_v2 import *

mp = MissionBlock() # the trajectory is useless
# init variables
k, pos_old, t_pos_old, cmdL, cmdR = 0, mp.kal.p(), time.time(), 0, 0
print("Starting the square mission")
wait_for_signal = False  # after time mission begin, the robot start following the trajectory

heading_list = [np.pi/2,np.pi,-np.pi/2,0] # 0 is east
time_init = time.time()
while time.time() < mp.time_mission_max:
    mp.measure(cmdL, cmdR)

    # update reference
    t = time.time()-time_init
    i = int(t//30) # turn every 30 second
    try:
        th_d = heading_list[i]
    except:
        break # mission finished
    v_d = 5 # m/s
    print("th_d",th_d)

    # update heading controller
    vd = 1.
    wd = control_heading(th_d, mp.kal.th)
    cmdL, cmdR = convert_motor_control_signal(v_d, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(v_d, wd, mp.wmLeft, mp.wmRight, cmdL, cmdR, np.zeros((2,1)), mp.y_th, mp.kal) # note w and th_d replace u[0,0] and u[1,0]
    mp.kal.Kalman_update(np.zeros((2,1)), mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.wait_for_next_iteration():
        break

mp.auto_home(cmdL, cmdR)
mp.ard.send_arduino_cmd_motor(0, 0)
