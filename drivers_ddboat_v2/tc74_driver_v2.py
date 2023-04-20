
import struct
import time
import sys
import math
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# Microchip TC74 temperature sensor

class TempTC74IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr_left = 0x4d  # temp LC 74 sensor left
        self.__addr_right = 0x48  # temp LC 74 sensor right
        self.__dev_i2c_left=i2c.i2c(self.__addr_left,self.__bus_nb)
        self.__dev_i2c_right=i2c.i2c(self.__addr_right,self.__bus_nb)
        self.__temp_right = 0
        self.__cfg_right = 0
        self.__temp_left = 0
        self.__cfg_left = 0
        self.__err_cnt_left = 0
        self.__err_cnt_right = 0
        try:
            val = self.__read("left",0x00)
        except:
            print ("Error : left motor temperature sensor not on I2C")
            self.__dev_i2c_left = None
        try:
            ival = self.__read("right",0x00)
        except:
            print ("Error : right motor temperature sensor not on I2C")
            self.__dev_i2c_right=None
        self.set_mode(standby=False,side="both")

    def __write(self,side,reg,data):
        if side == "left":
            if not self.__dev_i2c_left is None:
                self.__dev_i2c_left.write(reg,data)
        if side == "right":
            if not self.__dev_i2c_right is None:
                self.__dev_i2c_right.write(reg,data)

    def __read(self,side,reg):
        val = None
        if side == "left":
            if not self.__dev_i2c_left is None:
                try:
                    val =self.__dev_i2c_left.read(reg,1)[0]
                except:
                    self.__err_cnt_left += 1
                    print ("error reading motor temperature left, error count =",self.__err_cnt_left)
        if side == "right":
            if not self.__dev_i2c_right is None:
                try:
                    val = self.__dev_i2c_right.read(reg,1)[0]
                except:
                    self.__err_cnt_right += 1
                    print ("error reading motor temperature right, error count =",self.__err_cnt_right)
        #print (side,val)
        return val

    def set_mode (self,standby=False,side="both"):
        if side == "left" or side =="both":
            if standby:
                self.__write("left",0x01,[0x80])
            else:
                self.__write("left",0x01,[0x00])
        if side == "right" or side =="both":
            if standby:
                self.__write("right",0x01,[0x80])
            else:
                self.__write("right",0x01,[0x00])
        self.__cfg_left, self.__cfg_right = self.get_config()

    def set_config(self,cfg_left, cfg_right):
        self.__cfg_left = cfg_left
        self.__write("left",0x01,[self.__cfg_left])
        self.__cfg_right = cfg_right
        self.__write("right",0x01,[self.__cfg_right])

    def get_config(self):
        self.__cfg_left = self.__read("left",0x01)
        self.__cfg_right = self.__read("right",0x01)
        return self.__cfg_left, self.__cfg_right

    def read_temp(self):
        self.__temp_left = self.__read("left",0x00)
        self.__temp_right = self.__read("right",0x00)
        return self.__temp_left, self.__temp_right 
 
if __name__ == "__main__":
    temperature = TempTC74IO()

    temperature.set_config(0x0,0x0)
    
    cfg_left, cfg_right = temperature.get_config()
    st = "Config:"
    if not cfg_left is None:
        st += " 0x%2.2x"%(cfg_left)
    else:
        st += " None"
    if not cfg_right is None:
        st += " 0x%2.2x"%(cfg_right)
    else:
        st += " None"        
    print (st)

    time.sleep(1.0)

    temperature.set_mode(standby=True,side="both")
    cfg_left, cfg_right = temperature.get_config()
    st = "Config:"
    if not cfg_left is None:
        st += " 0x%2.2x"%(cfg_left)
    else:
        st += " None"
    if not cfg_right is None:
        st += " 0x%2.2x"%(cfg_right)
    else:
        st += " None"        
    print (st) 

    for i in range(5):
        temp_left, temp_right = temperature.read_temp()
        st = "Temperature (stand by) : Left="
        if not temp_left is None:
            st += "%d deg.C, Right="%(temp_left)
        else:
            st += "None, Right="
        if not temp_right is None:
            st += "%d deg.C"%(temp_right)
        else:
            st += "None"
        print (st)
        time.sleep(1.0)

    temperature.set_mode(standby=False,side="both")
    cfg_left, cfg_right = temperature.get_config()
    st = "Config:"
    if not cfg_left is None:
        st += " 0x%2.2x"%(cfg_left)
    else:
        st += " None"
    if not cfg_right is None:
        st += " 0x%2.2x"%(cfg_right)
    else:
        st += " None"        
    print (st)

    for i in range(55):
        temp_left, temp_right = temperature.read_temp()
        st = "Temperature : Left="
        if not temp_left is None:
            st += "%d deg.C, Right="%(temp_left)
        else:
            st += "None, Right="
        if not temp_right is None:
            st += "%d deg.C"%(temp_right)
        else:
            st += "None"
        print (st)
        time.sleep(1.0)

    temperature.set_mode(standby=True,side="both")
    cfg_left, cfg_right = temperature.get_config()
    st = "Config:"
    if not cfg_left is None:
        st += " 0x%2.2x"%(cfg_left)
    else:
        st += " None"
    if not cfg_right is None:
        st += " 0x%2.2x"%(cfg_right)
    else:
        st += " None"        
    print (st)
