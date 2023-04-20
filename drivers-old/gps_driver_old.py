import serial
import os
import time
import numpy as np
import datetime
# ~ import subprocess
# ~ import shlex

# the GPS sensor gives informations using the NMEA standard
# https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard
# https://en.wikipedia.org/wiki/NMEA_0183

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
        
class GpsIO:
    def __init__(self,tty_dev=0):
        # open serial line connected to the GPS sensor
        #time.sleep(1.0)
        #print(ser)
        self.tty_dev = "/dev/ttyGPS0"
        if tty_dev == 1:
          self.tty_dev =  "/dev/ttyGPS1"
        if tty_dev == 2:
          self.tty_dev =  "/dev/ttyGPS2"
        self.init_line()
   
    def init_line(self,timeout=1.0):
        self.ser = serial.Serial(self.tty_dev,timeout=timeout)

    def init_line_devname_baudrate(self,devname,baudrate,timeout=1.0):
        self.ser = serial.Serial(devname,baudrate=baudrate,timeout=timeout,xonxoff=False, rtscts=False, dsrdtr=False)
        
    def close(self):
        self.ser.close()

        
    def read_next_message(self):
        v=self.ser.readline().decode("utf-8")
        #print (v)
        return v

    # read the position in the GPGLL message
    # by default one GPGLL message is expected every 20 messages
    # warning: blocking function, not to use in control loops 
    def read_gll(self,n_try_max=20):
        val=[0.,'N',0.,'W',0.]
        for i in range(n_try_max):
            rdok = True
            try:
                v=self.ser.readline().decode("utf-8")
            except:
                print ("error reading GPS !!")
                rdok = False
                break # go out
            if rdok:
                if str(v[0:6]) == "$GPGLL":
                    vv = v.split(",")
                    if len(vv[1]) > 0:
                        val[0] = float(vv[1])
                    if len(vv[2]) > 0:
                        val[1] = vv[2]
                    if len(vv[3]) > 0:
                        val[2] = float(vv[3])
                    if len(vv[4]) > 0:
                        val[3] = vv[4]
                    if len(vv[5]) > 0:
                        val[4] = float(vv[5])
                    break # GPGLL found !  exit !
        return val

    def read_gll_non_blocking(self,timeout=0.01):
        self.ser.timeout=timeout
        v=""
        try:
            v=self.ser.readline()
        except:
            print ("error read GPS")
        msg=False
        val=[0.,'N',0.,'W',0.,'A']
        if len(v)>0:
            st=v.decode("utf-8")
            if str(st[0:6]) == "$GPGLL":
                vv = st.split(",")
                if len(vv[1]) > 0:
                    val[0] = float(vv[1])
                if len(vv[2]) > 0:
                    val[1] = vv[2]
                if len(vv[3]) > 0:
                    val[2] = float(vv[3])
                if len(vv[4]) > 0:
                    val[3] = vv[4]
                if len(vv[5]) > 0:
                    val[4] = float(vv[5])
                if len(vv[6]) > 0:
                    val[5] = vv[6]
                msg=True
        return msg,val
        
def synchronise_time_on_gps(gps): # use the gps to set hour , min , sec on UTC time
    while True: # syncronise time
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            t_UTC = gll_data[4] # time hour*10000+minuts*100+sec
            s = int(t_UTC%100)
            m = int(t_UTC%10000/100)
            h = int(t_UTC/10000)
            t = time.gmtime(time.time())
            y,mth,d = t.tm_year, t.tm_mon,t.tm_mday # year month day not updated
            dtStr = str(y)+"-"+str(mth)+"-"+str(d)+"-"+str(h)+"-"+str(m)+"-"+str(s)
            dt = datetime.datetime.strptime(dtStr, "%Y-%m-%d-%H-%M-%S") + datetime.timedelta(hours=2) # GMT1+1
            print("CURRENT TIME is "+str(dt))
            try:
                os.system("sudo date -s '"+str(dt)+"'")
            except:
                print("need sudo to update time")
            break
        time.sleep(0.01)
    
if __name__ == "__main__":
    gps = GpsIO()
    synchronise_time_on_gps(gps)
    # display the 20 first messages
    #for i in range(20):
    #    print (gps.read_next_message())

    # display the 20 positions (GPGLL) messages
    #for i in range(20):
    #    print (gps.read_gll())

    # test non blocking read for 20 positions

    cnt=0
    while True:
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            print (gll_data)
            print(cvt_gll_ddmm_2_dd(gll_data))
            # ~ print(gll_data[0]*np.pi/180,gll_data[2]*np.pi/180)
            cnt += 1
            if cnt==20:
                break
        time.sleep(0.01)

    gps.close()
