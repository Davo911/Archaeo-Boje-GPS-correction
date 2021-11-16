from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, socket, sys, math
from datetime import datetime
from subprocess import Popen
import pynmea2

def add_offset(lng, lat, ang, os):

    #CALCULATION
    lat0 = math.cos(math.PI / 180.0 * lat)
    lng_new = lng + (180/math.PI) * (os / 6378137) / math.cos(lat0) * math.cos(ang)
    lat_new = lat  + (180/math.PI) * (os / 6378137) * math.sin(ang)

    return [lng_new, lat_new]

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:02d}{:07.4f}".format(d, m + (s / 60))

BOOT_IP = sys.argv[1]
BOOT_PORT = sys.argv[2]
sock_boot = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

print ("Connecting...")
boje = connect('localhost:14550',"baud=57600")#, wait_ready=False)
#boje.wait_ready(True, timeout=180)
string_length = 2
while True:
    time.sleep(2)

    latitude = boje.location.global_frame.lat
    longitude = boje.location.global_frame.lon
    depth = 3                                           #TODO: How to read I2C Pressure Sensor
    angle = math.radians(boje.heading)                #TODO: Angle == Boot.angle?
    lat_dir = 'N' if latitude > 0 else 'S'
    lon_dir = 'E' if longitude > 0 else 'W'

    
    offset = math.sqrt((string_length**2)-(depth**2))  

    new_lnglat = add_offset(longitude,latitude,angle,offset)
    print("old: "+ str([longitude,latitude]))
    print("new: "+ str(new_lnglat))
    #GPS_boje = pynmea2.GGA('GP', 'GGA', ('203639.01' , decTodms(boje.location.global_frame.lat), lat_dir,decTodms(boje.location.global_frame.lon), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(float(boje.gps_0.eph)/100), '0', 'M', '0', 'M', '', ''))
    #GSA_boje = pynmea2.GSA('GP', 'GSA', (boje.mode.name[1], str(boje.gps_0.fix_type),'17','15','19','24','32','10','12','25','','','','','0',str(boje.gps_0.eph ),str(10)))


   # print(str(GPS_boje)+"\n")
   # print(str(GSA_boje)+"\n")
   # sock_boot.sendto(bytes(str(GPS_boje)+"\n"), (BOOT_IP, BOOT_PORT))
   # sock_boot.sendto(bytes(str(GSA_boje)+"\n"), (BOOT_IP, BOOT_PORT))

