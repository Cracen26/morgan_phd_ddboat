import time
import sys
import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpsdrv
import arduino_driver_v2 as arddrv
import encoders_driver_v2 as encoddrv

imu = imudrv.Imu9IO()
gps = gpsdrv.GpsIO()
ard = arddrv. ArduinoIO()
encod = encoddrv.EncoderIO()

# init encoder stream (get synchro characters)
encod.get_sync()

# change motor cmd with time
vtcmd = [  5, 10, 15, 20, 60] # time 
vvcmd = [ 40, 80, 120, 0,  0] # cmd
icmd = 0
ncmd = len(vtcmd)

tmax = 25.0
t0 = time.time()
while True:
    print ("---------------------------------------------------")
    t = time.time()
    if (t-t0) > tmax:
        # end, tmax reached
        break
    #test Arduino (motors)
    if icmd < ncmd:
        if (t-t0) > vtcmd[icmd]:
            cmdl = cmdr =  vvcmd[icmd]
            ard.send_arduino_cmd_motor(cmdl,cmdr)
            icmd += 1

    # test GPS
    gps_data_string = gps.read_gll()
    print ("GPS:",gps_data_string)

    # test IMU9
    magx,magy,magz = imu.read_mag_raw()
    aclx,acly,aclz = imu.read_accel_raw()
    gyrx,gyry,gyrz = imu.read_gyro_raw() 
    print ("Mag %5d %5d %5d"%(magx,magy,magz)," || Accel  %5d %5d %5d"%(aclx,acly,aclz)," || Gyro %5d %5d %5d"%(gyrx,gyry,gyrz))
    
    # test encoders
    sync,data_encoders = encod.read_packet(debug=False)
    if not sync:   # if sync lost try to get it again ...
        encod.get_sync()
    print ("Encoder:",data_encoders)

    time.sleep (0.2)
    
gps.close()

