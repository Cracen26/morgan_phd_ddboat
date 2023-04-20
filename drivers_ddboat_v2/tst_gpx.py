import time
import sys
import gps_driver_v2 as gpsdrv
import gpxpy.gpx

# record GPS (GPGLL ) in a GPX file
# requires install of gpxpy
# sudo apt install python3-gpxpy

def cvt_gll_ddmm_2_dd (st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat/100))
    olon = float(int(ilon/100))
    olat_mm = (ilat%100)/60
    olon_mm = (ilon%100)/60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat,olon

gps = gpsdrv.GpsIO()
# debug with USB GPS
#gps.init_line_devname_baudrate("/dev/ttyUSB0",9600)

# open gpx buffer
gpx = gpxpy.gpx.GPX()
# Create first track in our GPX:
gpx_track = gpxpy.gpx.GPXTrack()
gpx.tracks.append(gpx_track)
# Create first segment in our GPX track:
gpx_segment = gpxpy.gpx.GPXTrackSegment()
gpx_track.segments.append(gpx_segment)

tmax = 10*60.0 # 10 minutes max (can be stopped before the end with Ctrl+C)
t0 = time.time()
while True:
    print ("---------------------------------------------------")

    # test GPS
    gps_data_string = gps.read_gll()
    print ("GPS:",gps_data_string)
    if gps_data_string[0] == 0.0:
        break
    lat,lon = cvt_gll_ddmm_2_dd (gps_data_string)
    print (lat,lon)
    
    gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))

    if (time.time()-t0) > tmax:
        break

fp = open("tst.gpx","w")
fp.write(gpx.to_xml())
fp.write("\n")
fp.close()
