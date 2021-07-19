#from dronekit import connect, Command, LocationGlobal
#from pymavlink import mavutil
import io, time, sys, argparse, math, socket, math
#from subprocess import Popen
#import pynmea2

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s/60))
     #return [d, m, s]

    # DMS:          510208.07294,N; 0134409.6438,E
    # DDDMM.MMMM:   5102.140N; 01344.177,E
    # Decimal:      Latitude=51.0355758333; Longitude=13.7360121667
lat = 51.0355758333
lng = 13.7360121667
lat_dms = decTodms(lat)
lng_dms = decTodms(lng)

print(lat_dms)
print(lng_dms)