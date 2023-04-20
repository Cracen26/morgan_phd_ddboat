# Script to calibrate the compass

from lib.DDBOAT_filter_v2 import *
from lib.DDBOAT_log_v2 import init_drivers, robot_number

import json

# load mission script
filename = "compass_calibration/compass_calibration_ddboat"+robot_number+".json"
file_script = open(filename,"r")
data_script = json.load(file_script)

b = np.reshape(np.array([data_script["b"]]),(3,1))
A = np.reshape(np.array([data_script["A"]]),(3,3))

file_script.close()

print("Old values")
print("A = ")
print(A)
print("b = ")
print(b)

_, _, _, encoddrv, imu = init_drivers()
filt = DdboatFilter(0, 0, A, b,encoddrv)

filt.compass_calibration(imu)

print("New values")
print("A = ")
print(filt.A)
print("b = ")
print(filt.b)

print("Saving new values (y/n)")
test_final = input()
if test_final == "y":
    A_f = filt.A.flatten()
    b_f = filt.b.flatten()
    data = {"A":A_f.tolist(),"b":b_f.tolist()}
    with open(filename,"w") as write_file:
        json.dump(data, write_file, indent=4)
        write_file.close()
    
    
print("Calibration done")
