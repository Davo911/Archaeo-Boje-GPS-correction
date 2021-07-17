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
     return [d, m, s]

#$GPGGA,102311.996,5102.140,N,01344.177,E,1,12,1.0,0.0,M,0.0,M,,*68
#Latitude=51.0355758333and Longitude=13.7360121667
lat = 51.0355758333
lng = 13.7360121667
lat_dms = decTodms(lat)
lng_dms = decTodms(lng)

print(str(lat_dms))
print(str(lng_dms))