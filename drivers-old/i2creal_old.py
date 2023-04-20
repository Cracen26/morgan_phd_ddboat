import smbus
import time
class i2c():
    def __init__(self,addr,bus_nb=2):
        self.__bus_nb = bus_nb
        self.__addr = addr
        self.__bus = smbus.SMBus(self.__bus_nb)
        print ("i2c device addr = 0x%x on bus %d"%(addr,bus_nb))

    def read(self,offs,nbytes):
        v = self.__bus.read_i2c_block_data(self.__addr,offs,nbytes)
        time.sleep(0.001)
        return v

    def read_byte(self,offs):
        v = self.__bus.read_byte_data(self.__addr,offs)
        time.sleep(0.001)
        return v
   
    def write(self,cmd,vData):
        self.__bus.write_i2c_block_data(self.__addr,cmd,vData)
        time.sleep(0.001)

