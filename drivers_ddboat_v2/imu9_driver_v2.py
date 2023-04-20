import struct
import time
import sys
import math
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)

class Imu9IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr_mg = 0x1e  # mag sensor
        self.__addr_ag = 0x6b  # accelero - gyro

        self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb)
        self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb)

    # place your imu functions here
        self.magx_min = 0.0
        self.magx_max = 0.0
        self.magy_min = 0.0
        self.magy_max = 0.0
        self.magx_offs = 0.0
        self.magy_offs = 0.0
        self.magx_scale = 0.0
        self.magy_scale = 0.0

        self.__mag_raw = [0.0,0.0,0.0]
        self.__accel_raw = [0.0,0.0,0.0]
        self.__gyro_raw = [0.0,0.0,0.0]
        #
        # configure mag sensor
        # CTRL_REG1 (0x20) = 0b01110000
        # OM = 11 (ultra-high-performance mode for X and Y);
        # DO = 100 (10 Hz ODR)
        self.__dev_i2c_mg.write(0x20,[0x70])
        # CTRL_REG2 (0x21) = 0b00000000
        # FS = 00 (+/- 4 gauss full scale)
        self.__dev_i2c_mg.write(0x21,[0x00])
        # CTRL_REG3 (0x22) = 0b00000000
        # MD = 00 (continuous-conversion mode)
        self.__dev_i2c_mg.write(0x22,[0x00])
        # CTRL_REG4 (0x23) = 0b00001100
        # OMZ = 11 (ultra-high-performance mode for Z)
        self.__dev_i2c_mg.write(0x23,[0x0C])
        #
        # configure accelero + gyro
        # LSM6DS33 gyro
        # CTRL2_G (0x11) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_G = 11 (2000 dps)
        self.__dev_i2c_ag.write(0x11,[0x8C])
        #
        # CTRL7_G (0x16) = 0b00000000
        # defaults
        self.__dev_i2c_ag.write(0x16,[0x00])
        #
        # LSM6DS33 accelerometer
        # CTRL1_XL (0x10) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_XL = 11 (8 g full scale)
        # BW_XL = 00 (400 Hz filter bandwidth)
        #self.__dev_i2c_ag.write(0x10,[0x8C])
        # more filtering BW_XL = 11 (50 Hz filter bandwidth)
        self.__dev_i2c_ag.write(0x13,[0x00])
        self.__dev_i2c_ag.write(0x10,[0x8C])
        #
        # common
        # CTRL3_C (0x12) 0b00000100
        # IF_INC = 1 (automatically increment address register)
        self.__dev_i2c_ag.write(0x12,[0x04])

    def setup_accel_filter (self,mode):
        if mode == 0:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x00])
            self.__dev_i2c_ag.write(0x10,[0x8C])
        elif mode == 1:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])
        elif mode == 2:
            self.__dev_i2c_ag.write(0x17,[0x80])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])
                    
    def read_mag_raw(self):
        v = self.__dev_i2c_mg.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__mag_raw = [ix,iy,iz]
        return self.__mag_raw
 
    def read_gyro_raw(self):
        # OUTX_L_G (0x22)
        v = self.__dev_i2c_ag.read(0x22,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__gyro_raw = [ix,iy,iz]
        return self.__gyro_raw
 
    def read_accel_raw(self):
        # OUTX_L_XL (0x28)
        v = self.__dev_i2c_ag.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__accel_raw = [ix,iy,iz]
        return self.__accel_raw

    def cmpl2(self,lsByte,msByte):
        i = lsByte + (msByte << 8)
        if i >= (1<<15):
            i = i - (1<<16)
        return i

    def heading_raw(self,magx,magy):
        heading = math.atan2(magy,magx)
        return heading
        
    def heading_raw_deg(self,magx,magy):
        heading = self.heading_raw(magx,magy)*180.0/math.pi
        if heading < 0.0:
            heading += 360.0
        return heading

    def fast_heading_calibration (self, magx_min, magx_max, magy_min, magy_max):
        self.magx_min = magx_min
        self.magx_max = magx_max
        self.magy_min = magy_min
        self.magy_max = magy_max
        self.magx_offs = (magx_min+magx_max)/2.0
        self.magy_offs = (magy_min+magy_max)/2.0
        self.magx_scale = 2./(magx_max-magx_min)
        self.magy_scale = 2./(magy_max-magy_min)
        
    def heading(self,magx,magy):
        magx_cal = (magx - self.magx_offs) * self.magx_scale
        magy_cal = (magy - self.magy_offs) * self.magy_scale
        #print (self.magx_min,magx,self.magx_max,self.magy_min,magy,self.magy_max,magx_cal,magy_cal)
        magx_cal = 1.0 if magx_cal>1.0 else magx_cal
        magx_cal = -1.0 if magx_cal<-1.0 else magx_cal
        magy_cal = 1.0 if magy_cal>1.0 else magy_cal
        magy_cal = -1.0 if magy_cal<-1.0 else magy_cal
        heading = math.atan2(magy_cal,magx_cal)
        return heading
        
    def heading_deg(self,magx,magy):
        heading = self.heading(magx,magy)*180.0/math.pi
        if heading < 0.0:
            heading += 360.0
        return heading
    
                    
if __name__ == "__main__":
    imu = Imu9IO()
    for i in range(200):
        print (imu.read_mag_raw(),imu.read_accel_raw(),imu.read_gyro_raw())
        time.sleep(0.01)

