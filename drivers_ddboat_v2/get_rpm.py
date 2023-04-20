import encoders_driver_v2 as encdrv
import arduino_driver_v2 as ardudrv
import sys
import time

def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


if __name__ == "__main__":
    
    timeout = 1.0
    cmdl = 50
    cmdr = 50
    duration = 60.0
    tloop = 5.0 # 0.2 Hz loop
    ard = ardudrv.ArduinoIO()
    ard.send_arduino_cmd_motor (cmdl,cmdr)
    encoddrv = encdrv.EncoderIO(old=True)      

    print ("ok!")
    #sync0, timeAcq0, sensLeft0, sensRight0, posLeft0, posRight0 =  encodrv.read_single_packet(debug=True)
    while True:
        sync0,data_encoders0 = encoddrv.read_packet(debug=False)
        #print (sync0)
        if sync0:
            break
        time.sleep(0.1)

    print ("ok!")
    timeAcq0 = data_encoders0[0]
    sensLeft0 = data_encoders0[1]
    sensRight0 = data_encoders0[2]
    posLeft0 = data_encoders0[3]
    posRight0 = data_encoders0[4]
    print (sync0, timeAcq0, sensLeft0, sensRight0, posLeft0, posRight0)

    t0 = time.time()
    while (time.time()-t0) < duration:
        t0loop = time.time()
        while (time.time()-t0loop) < tloop:
            sync1,data_encoders1 = encoddrv.read_packet(debug=False)
            time.sleep(0.001)

        #sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1 =  encodrv.read_single_packet(debug=True)
        sync1,data_encoders1 = encoddrv.read_packet(debug=True)
        timeAcq1 = data_encoders1[0]
        sensLeft1 = data_encoders1[1]
        sensRight1 = data_encoders1[2]
        posLeft1 = data_encoders1[3]
        posRight1 = data_encoders1[4]

        #print sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1
        print ("dTime",timeAcq0,timeAcq1,timeAcq1-timeAcq0)
        print ("dOdoL",posLeft0,posLeft1,posLeft1-posLeft0)
        print ("dOdoR",posRight0,posRight1,posRight1-posRight0)

        rpmL = delta_odo(posLeft1,posLeft0)/8.0/tloop*60.0
        rpmR = delta_odo(posRight1,posRight0)/8.0/tloop*60.0
        print ("RPM Left",rpmL,"RPM Right",rpmR)
       
        timeAcq0 = timeAcq1
        posRight0 = posRight1
        posLeft0 = posLeft1

    ard.send_arduino_cmd_motor (0,0)
 
