# M0 - the robot return the home gps position defined in the mission script

from lib.DDBOAT_controler_v2 import *
from lib.DDBOAT_mission_v2 import MissionBlock
import time

#######################
# robot setup
#######################

print("robot setup ...")
mb = MissionBlock()

#####################
# mission loop
#####################
mission = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mb.home_pos, mb.kal.p(), time.time(), 0, 0
print("Going Home")

mb.auto_home(0.,0.)

mb.ard.send_arduino_cmd_motor(0, 0)
