import arduino_driver_v2 as ardu
import tc74_driver_v2 as tc74
import encoders_driver_v3 as encod
import imu9_driver_v2 as imudv
import gps_driver_v3 as gpsdrv
import logging
import time
import sys

robot_number = str(encod.robot_id)
# record sesors data and store them in a log file under the LOG folder

# noinspection PyBroadException
class LogRecorder:
    def __init__(self,t=None):
        # create a log file
        if t is None:
            t=time.localtime(time.time())
        number = str(t.tm_year) + "_" + str(t.tm_mon) + "_" + str(t.tm_mday) + "_" + str(t.tm_hour) + "_" + str(
            t.tm_min)
        logging.basicConfig(filename='Log/DDBOAT' + robot_number + "_t_" + number + '.log', level=logging.DEBUG,
                            format='%(asctime)s - %(levelname)s - %(message)s')
        self.msg = ""  # string buffer for log messages

    def log_observe_update(self, temperature_, ard_, gps_, encoddrv_, imu_):
        try:
            tl, tr = temperature_.read_temp()
            # print("temperatures l,r %d %d"%(tl,tr))
            self.msg = self.msg + "Temperature l,r are %d %d deg.C /" % (tl, tr)
        except:
            logging.error("can't log the temprerature")
            tl, tr = -1, -1
        try:
            data = ard_.get_arduino_cmd_motor()
            self.msg = self.msg + "cmd motors l,r are " + str(data) + " /"
        except:
            data = -1
            logging.error("can't log the motor command")

        try:
            gll_ok, val = gps_.read_gll_non_blocking()
            if gll_ok:
                self.msg = self.msg + "GPS " + str(val) + " /"
            else:
                self.msg = self.msg + "GPS timeout /"
        except:
            gll_ok = False
            val = -1
            logging.error("can't log the GPS")

        try:
            sync, data_encoders = encoddrv_.read_packet()
            self.msg = self.msg + "Encoders " + str(data_encoders) + " /"
        except:
            sync = False
            data_encoders = []
            logging.error("can't log the encoders")

        try:
            mag = imu_.read_mag_raw()
            accel = imu_.read_accel_raw()
            gyro = imu_.read_gyro_raw()
            self.msg = self.msg + "IMU: MAG " + str(mag) + " ACCEL " + str(accel) + " GYRO " + str(gyro) + " /"
        except:
            mag, accel, gyro = [], [], []
            logging.error("can't log the imu")
        return tl, tr, data, gll_ok, val, sync, data_encoders, mag, accel, gyro

    def log_control_update(self, acc, wd, Wmleft, Wmright,CmdL,CmdR,pd, th,Kal=None):
        try:  # log desired pose
            self.msg = self.msg + "DESIREDPOSITION " + str(pd.tolist()) + " /"
        except:
            self.msg = self.msg
        
        self.msg = self.msg + "CONTROL: ACCD " + str(acc) + " WD " + str(wd) + " "
        self.msg = self.msg + "WMLEFT " + str(Wmleft) + " WMRIGHT " + str(Wmright) + " "
        self.msg = self.msg + "CMDL " + str(CmdL) + " CMDR " + str(CmdR) + " "
        self.msg = self.msg + "THETA " + str(th) + " /"

        if Kal:  # log kalman filter if it exist
            X = str(Kal.X.tolist())
            g = str(Kal.Gamma.tolist())
            U = str(Kal.u.tolist())
            Y = str(Kal.y.tolist())
            self.msg = self.msg + "KAL: STATE " + X + " COVARIANCE " + g + " INPUT " + U + " OUTPUT " + Y + " /"
        return

    def log_update_write(self):  # write and reset the message
        logging.info(self.msg)
        self.msg = ""
        return


# noinspection PyBroadException
def init_drivers():  # test connection to sensors and return sensor class
    try:
        ard_ = ardu.ArduinoIO()
    except:
        ard_ = 0
        print("arduino interface not working")

    try:
        temperature_ = tc74.TempTC74IO()
    except:
        temperature_ = 0
        print("temperature sensor not connected")

    try:
        gps_ = gpsdrv.GpsIO()
    except:
        gps_ = 0
        print("gpx not connected")

    try:
        encoddrv_ = encod.EncoderIO()
    except:
        encoddrv_ = 0
        print("encoder not connected")

    try:
        imu_ = imudv.Imu9IO()
    except:
        imu_ = 0
        print("imu not connected")

    return ard_, temperature_, gps_, encoddrv_, imu_


if __name__ == "__main__":
    log = LogRecorder()

    ard, temperature, gps, encoddrv, imu = init_drivers()

    # noinspection PyBroadException
    try:
        cmdl = int(sys.argv[1])
    except:
        cmdl = 0
    # noinspection PyBroadException
    try:
        cmdr = int(sys.argv[2])
    except:
        cmdr = 0
    ard.send_arduino_cmd_motor(cmdl, cmdr)
    print("Logging on")

    while True:
        log.log_observe_update(temperature, ard, gps, encoddrv, imu)
        log.log_update_write()
        time.sleep(0.5)
