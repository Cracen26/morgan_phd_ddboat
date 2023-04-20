# set the current date of the DDBOAT iner clock
import sys
import datetime
import time
import os

if __name__ == "__main__":
    try:
        year = int(sys.argv[1])
    except:
        year = 2022
    try:
        month = int(sys.argv[2])
    except:
        month = 1

    try:
        day = int(sys.argv[3])
    except:
        day = 1
    t = time.localtime(time.time()) # incorrect
    h, m, s = t.tm_hour, t.tm_min, t.tm_sec  # year month day not updated
    dtStr = str(year) + "-" + str(month) + "-" + str(day) + "-" + str(h) + "-" + str(m) + "-" + str(s)
    dt = datetime.datetime.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
    print("CURRENT TIME is " + str(dt))
    try:
        os.system("sudo date -s '" + str(dt) + "'")
    except:
        print("need sudo to update time")
