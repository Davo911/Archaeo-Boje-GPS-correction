from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, socket
from datetime import datetime
from subprocess import Popen
import pynmea2

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:02d}{:07.4f}".format(d, m + (s / 60))

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

    GPS_boje = pynmea2.GGA('GP', 'GGA', ('203639.01' , decTodms(boje.location.global_frame.lat), lat_dir,decTodms(boje.location.global_frame.lon), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(boje.gps_0.eph), '0', 'M', '0', 'M', '', ''))

#$GPGGA,203639.01,5555.2846,N,1133.1222,E,1,8,0,20,M,0,M,,*7F
#$GPGSA,A,3,17,15,19,24,32,10,12,25,,,,,1.77,1.00,1.46*09

#$GPGGA,203639.01,5102.1364,N,1344.1689,E,3,7,259,0,M,0,M,,0000*47
#$GPGSA,A,3,17,15,19,24,32,10,12,25,,,,,0,259,10*1B



    GSA_boje = pynmea2.GSA('GP', 'GSA', (boje.mode.name[1], str(boje.gps_0.fix_type),'17','15','19','24','32','10','12','25','','','','','0',str(boje.gps_0.eph ),str(10)))
    
    print(str(GPS_boje))
    print(str(GSA_boje))
    sock_boot.sendto(bytes(str(GPS_boje)), (BOOT_IP, BOOT_PORT))
    sock_boot.sendto(bytes(str(GSA_boje)), (BOOT_IP, BOOT_PORT))

