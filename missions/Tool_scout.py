# print( heading and local position) while the robot is controller by the remote controller

from lib.DDBOAT_filter_v2 import *
from lib.DDBOAT_log_v2 import init_drivers, time, robot_number
import json

file_script = open("../mission_script/mission_script.json", "r")
file_script2 = open("compass_calibration/compass_calibration_ddboat"+robot_number+".json", "r")
data_script = json.load(file_script)
data_script2 = json.load(file_script2)
param = data_script["mission_param"]

lxm = param["lxm"]
lym = param["lym"]
b = np.reshape(np.array([data_script2["b"]]), (3, 1))
A = np.reshape(np.array([data_script2["A"]]), (3, 3))

ard, temperature, gps, encoddrv, imu = init_drivers()
filt = DdboatFilter(lxm, lym, A, b, encoddrv)

while True:
    gll_ok,gll_data=gps.read_gll_non_blocking()
    mag = imu.read_mag_raw()
    y_th = 360/(2*3.14)*filt.cap(mag[0], mag[1], mag[2])
    if gll_ok:
        lat,lon = filt.cvt_gll_ddmm_2_dd(gll_data)
        msg = ""
        msg = msg + "Heading "+str(y_th)
        msg = msg +" | Lat Lon "+str(lat)+" "+str(lon)
        msg = msg + " | Localpose x y " + str(filt.latlon_to_coord(lat, lon).T)
        print(msg)
    time.sleep(0.01)

gps.close()
    
