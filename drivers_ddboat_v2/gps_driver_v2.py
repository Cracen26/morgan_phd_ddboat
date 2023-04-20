import serial
import os
import time

# the GPS sensor gives informations using the NMEA standard
# https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard
# https://en.wikipedia.org/wiki/NMEA_0183

class GpsIO:
    def __init__(self, tty_dev=0):
        # default device for GPS sensor serial line is /dev/ttyGPS0 with tty_dev=0
        # 2 other devices can be used to get GPS data
        #    tty_dev=1 -> /dev/ttyGPS1
        #    tty_dev=2 -> /dev/ttyGPS2
        # so that 3 different programs can use GPS at the same time.
        self.tty_dev = "/dev/ttyGPS0"
        if tty_dev == 1:
          self.tty_dev =  "/dev/ttyGPS1"
        if tty_dev == 2:
          self.tty_dev =  "/dev/ttyGPS2"
        self.init_line()
        #time.sleep(1.0)
        #print(ser)
   
    def init_line(self,timeout=1.0):
        # do not use directly the serial device connected to the GPS
        # self.ser = serial.Serial('/dev/ttyS0',timeout=timeout)
        # use instead the first of the 3 copies /dev/ttyGPS0
        self.ser = serial.Serial(self.tty_dev,timeout=timeout)

    def init_line_devname_baudrate(self,devname,baudrate,timeout=1.0):
        self.ser = serial.Serial(devname,baudrate=baudrate,timeout=timeout,xonxoff=False, rtscts=False, dsrdtr=False)
        
    def close(self):
        self.ser.close()

        
    def read_next_message(self):
        v=self.ser.readline().decode("utf-8")
        #print (v)
        return v

    def compute_checksum(self,st):
        csk = 0
        for i in range(len(st)):
            csk = csk ^ ord(st[i])
        return hex(csk)[2:]

    def mtk_test(self):
        pmtk = "PMTK414"
        pmtk = "PMTK447"
        #pmtk = "PMTK386,0"
        csksum = self.compute_checksum(pmtk)
        msg = "$"
        for c in pmtk:
            msg += c 
        msg += "*"
        for c in csksum:
            msg += c
        print (msg)
        msg += chr(13)
        msg += chr(10)
        bmsg = msg.encode() 
        print (bmsg)
        self.ser.write(bmsg)
        self.ser.write(bmsg)
        while True:
            rcv = self.read_next_message()
            if rcv.startswith("$PMTK"):
                print (rcv)
                break

    def send_mtk_cmd (self,pmtk):
        csksum = self.compute_checksum(pmtk)
        msg = "$"
        for c in pmtk:
            msg += c
        msg += "*"
        for c in csksum:
            msg += c
        print (msg)
        msg += chr(13)
        msg += chr(10)
        bmsg = msg.encode()
        print (bmsg)
        self.ser.write(bmsg)

    def get_mtk_status (self):
        while True:
            rcv = self.read_next_message()
            if rcv.startswith("$PMTK"):
                print (rcv)
                break
        return rcv
        
    def get_filter_speed(self):
        pmtk = "PMTK447"
        self.send_mtk_cmd (pmtk)
        rcv = self.get_mtk_status ()
        return rcv

    def set_filter_speed(self,stv):
        # v can be 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, or 2.0
        pmtk = "PMTK386,"+stv
        self.send_mtk_cmd (pmtk)
        rcv = self.get_mtk_status ()
        return rcv


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
        val=[0.,'N',0.,'W',0.]
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
                msg=True
        return msg,val
        
    
if __name__ == "__main__":
    gps = GpsIO()

    # display the 20 first messages
    #for i in range(20):
    #    print (gps.read_next_message())

    # display the 20 positions (GPGLL) messages
    #for i in range(20):
    #    print (gps.read_gll())

    #print (gps.compute_checksum("PMTK605"))
    gps.set_filter_speed("0.4")
    gps.get_filter_speed()
    gps.set_filter_speed("0")
    gps.get_filter_speed()

    # test non blocking read for 20 positions
    cnt=0
    while True:
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            print (gll_data)
            cnt += 1
            if cnt==20:
                break
        time.sleep(0.01)

    gps.close()
