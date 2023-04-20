#
# requires  : gpxpy and pyproj
# sudo apt install python3-pyproj
# sudo apt install python3-gpxpy

import gpxpy
from pyproj import Proj, transform
import numpy as np
import sys

# define the projection from WGS84 (lon,lat in degrees) to UTM (meters)
# UTM zone for Brittany is 30N 
projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

try:
    iref = int(sys.argv[2])
except:
    iref = 0  # ENSTA Bretagne's flag
# select reference point
if iref==0:
    # reference point "the flag"
    reference_lat = 48.4188
    reference_lon = -4.4725
elif iref==1:
    # reference point center of the rugby field
    reference_lat = 48.4185
    reference_lon = -4.4739
elif iref==2:
    # reference point "guerledan"
    reference_lat = 48.199111
    reference_lon = -3.014930
reference_x,reference_y = projDegree2Meter(reference_lon,reference_lat)
print ("ref",reference_x,reference_y,reference_lon,reference_lat)

# open the ddboat gps log file (gpx format) 
try:
    gpx_file = open(sys.argv[1], 'r')
except:
    gpx_file = open('tst_ensta_stadium_20210929.gpx', 'r')

gpx = gpxpy.parse(gpx_file)
i = 0
for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            #print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
            x,y = projDegree2Meter(point.longitude, point.latitude)
            lat,lon = projDegree2Meter(x,y,inverse=True)
            #print (x,y,lat,lon,point.latitude,point.longitude)
            dx = x - reference_x
            dy = y - reference_y
            distance = np.sqrt(dx*dx+dy*dy)
            # print  first distance
            if i == 0:
                print (dx,dy,distance)
            i += 1
# print last distance
print (dx,dy,distance)
