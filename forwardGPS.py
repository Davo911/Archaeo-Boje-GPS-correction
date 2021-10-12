from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math
from datetime import datetime, timedelta
from subprocess import Popen
import pynmea2

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s / 60))

BOOT_IP = "192.168.2.2"
BOOT_PORT = 27000
sock_boot = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

print ("Connecting...")
boje = connect('localhost:14550',"baud=57600")#, wait_ready=False)
#boje.wait_ready(True, timeout=180)

while True:
    time.sleep(0.1)
    lat_dir = 'N' if boje.location.global_frame.lat > 0 else 'S'
    lon_dir = 'E' if boje.location.global_frame.lon > 0 else 'W'

    GPS_boje = pynmea2.GGA('GP', 'GGA', (str(datetime.utcnow().timestamp()) , decTodms(boje.location.global_frame.lat), lat_dir,decTodms(boje.location.global_frame.lon), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(boje.gps_0.eph), str(boje.location.global_frame.alt), 'M', '0.0', 'M', '', '0000'))
    
    print(str(GPS_boje))
    sock_boot.sendto(bytes(str(GPS_boje)), (BOOT_IP, BOOT_PORT))
