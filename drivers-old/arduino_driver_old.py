import serial
import time
import signal
import sys

class ArduinoIO():
    def __init__(self):
        self.arduino = None
        self.cmdl = 0
        self.cmdr = 0
        self.dirl = ' '
        self.dirr = ' '
        print ("Init Arduino ...")
        try:
            self.arduino = serial.Serial('/dev/ttyACM0',115200,timeout=10.0)
            #arduino = serial.Serial('/dev/ttyACM1',115200,timeout=1.0)
            data = self.arduino.readline()
            print ("init status",len(data),data)
        except:
            print ("Cannot initialize Arduino driver")
        signal.signal(signal.SIGINT, self.signal_handler)
        print ("Arduino OK ...")
    
    def signal_handler(self,sig, frame):
        print('You pressed Ctrl+C! set motors to 0!')
        #serial_arduino = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)
        data = self.send_arduino_cmd_motor(0,0)
        sys.exit(0)

    def send_arduino_cmd_motor(self,cmdl0,cmdr0):    
        self.cmdl = cmdl0
        self.cmdr = cmdr0
        self.dirl = ' '
        self.dirr = ' '
        # no backward motion
        #if cmdl < 0:     
        #    self.dirl = '-'
        #if cmdr < 0:
        #    self.dirr = '-'
        self.cmdl = abs(self.cmdl)
        self.cmdr = abs(self.cmdr)
        strcmd = "M%c%3.3d%c%3.3d;"%(self.dirl,self.cmdl,self.dirr,self.cmdr)
        #print (strcmd)
        self.arduino.write(strcmd.encode())

    def get_arduino_cmd_motor(self,timeout=1.0):    # set by RC command ?
        strcmd = "C;"
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data="\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_cmd_motor timeout",timeout)
                break
        return data[0:-1]

    # last motor command sent 
    def get_cmd_motor(self):
        return self.cmdl,self.cmdr

    def get_arduino_rc_chan(self,timeout=1.0):
        strcmd = "R;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        data = "\n"
        while True:
            data = self.arduino.readline()
            if data:
                #print (data)
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_rc_chan timeout",timeout)
                break
        return data
   
    def get_arduino_status(self,timeout=1.0):
        strcmd = "S;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        while True:
            data = self.arduino.readline()
            if data:
                #print (data.decode())
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_status timeout",timeout)
                break
        return data
   
    def get_arduino_energy_saver(self,timeout=1.0):
        strcmd = "E;"
        #print (strcmd.encode())
        self.arduino.write(strcmd.encode())
        t0 = time.time()
        while True:
            data = self.arduino.readline()
            if data:
                #print (data.decode())
                break
            if (time.time()-t0) > timeout:
                print ("get_arduino_status timeout",timeout)
                break
        return data

if __name__ == "__main__":
    ard = ArduinoIO()
    try:
        cmdl = int(sys.argv[1])
    except:
        cmdl = 30
    try:
        cmdr = int(sys.argv[2])
    except:
        cmdr = 30

    ard.send_arduino_cmd_motor (cmdl,cmdr)
    print ("Arduino status is",ard.get_arduino_status())
    print ("cmd motors l,r are",ard.get_cmd_motor())
    print ("RC channel is",ard.get_arduino_rc_chan())
    
    print ("Hit Ctrl+C to get out!")
    while True:
        print ("arduino motors rc",ard.get_arduino_cmd_motor())
        print ("RC channel is",ard.get_arduino_rc_chan())
        print ("ENSV: ",ard.get_arduino_energy_saver())

        time.sleep(1.0)
