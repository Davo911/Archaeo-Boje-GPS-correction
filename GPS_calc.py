from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math
from datetime import datetime, timedelta
from subprocess import Popen
import pynmea2
import requests

alt_url = "http://192.168.2.2:4777/mavlink/VFR_HUD/alt"


def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s / 60))

def add_offset(lat,lng, ang, os):

    #CALCULATION
    lat0 = math.cos(math.pi / 180.0 * lat)
    lng_new = lng + (180/math.pi) * (os / 6378137) / math.cos(lat0) * math.cos(ang)
    lat_new = lat  + (180/math.pi) * (os / 6378137) * math.sin(ang)

    return [lat_new,lng_new]

def parseArguments():
    if len(sys.argv)>1 and sys.argv[1].split("=")[0]=="-s":
        try:
            string_length = float(sys.argv[1].split("=")[1])
            return string_length
        except ValueError:
            print("No String length defined ( -s=2.0 )")
            return 2.0
    else:
        print("No String length defined ( -s=2.0 ) -2")
        return 2.0

def start_mavProxy():
    # mavproxy.py --out 127.0.0.1:14550 --show-errors --baudrate 115200
    print ("Start MavProxy Server...")
    logfile = open("./mavprxy.log", "w")
    server_proc = Popen(["mavproxy.py", "--out", "udp:127.0.0.1:14550"], shell=True,stdout=logfile)
    time.sleep(15)

###MAIN ENTRY POINT###
string_length = parseArguments()
print("String length:"+str(string_length))

# Setup GPS serial port
boje_serial_port = "/dev/ttyUSB0"
ser_boje = serial.Serial(boje_serial_port, baudrate=115200,timeout=0.5)

#   GC socket
GC_IP = "192.168.2.1"
GC_PORT = 27000
sock_gc = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

#   Open socket to Boot-pi
BOOT_IP = "192.168.2.2"
BOOT_PORT = 27000
sock_boot = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Connect to the MavProxies
#start_mavProxy()
print ("Connecting boje...")
# connection_boje = 'localhost:14550'
# boje = connect(connection_boje,"baud=57600")#, wait_ready=False)
#boje.wait_ready(True, timeout=180)

# connection_boot = 'tcp:192.168.2.2:5760'
# boot = None
# try:
#     boot = connect(connection_boot, wait_ready=True)
# except Exception as e:
#     print("Connection Error to Boot-FC")
#     print('Parse error: {}'.format(e))

while True:
    time.sleep(1)
    resp = requests.get(alt_url)
    print(resp)

    #get BOJE parameter
    # latitude = boje.location.global_frame.lat
    # longitude = boje.location.global_frame.lon
    # lat_dir = 'N' if latitude > 0 else 'S'
    # lon_dir = 'E' if longitude > 0 else 'W'
    # angle_boje = math.radians(boje.heading)

    #get BOOT parameter
    #depth = boot.location.global_relative_frame.alt
    #print("lat|lng: " + str(latitude) + " | " + str(longitude))
    #print("depth: "+ depth)

    # offset = math.sqrt((string_length**2)-(depth**2))
    # GPS_boje_data = pynmea2.parse("$GPGGA,150559.00,5102.85348,N,01345.01389,E,1,05,2.79,153.2,M,43.7,M,,*59")
    # #print(str(GPS_boje_data))
    # print("sending ...")
    # sock_boot.sendto(bytes(str(GPS_boje_data)), (BOOT_IP, BOOT_PORT))
    # #sock_gc.sendto(bytes(str(GPS_boje_data)), (GC_IP, GC_PORT))

print("Connection lost")










#        depth = boot.location.global_relative_frame.alt
#        compass_boot = boot.heading
#        speed_boot = boot.groundspeed
#        #GPS_boje_data = pynmea2.parse(ser.readline())
#        GPS_boje_data = pynmea2.parse()
#        depth = 1.5
#        offset = math.sqrt((string_length**2)-(depth**2))       
#        compass_boje = boje.heading
#        speed_boje = boje.groundspeed
#        angle = Math.toRadians(compass_boot)
#        print("Battery: %s" % boje.battery)
#        #print("offset: "+str(offset))
#        print("compass: "+str(compass_boje))
#        print("speed: "+str(speed_boje))
#        print("GPS: %s" % boje.gps_0)
#        print("Location: %s" % boje.location.global_frame)
#
#        if (math.isclose(speed_boje, speed_boot,rel_tol=0.2) and (compass_boot - compass_boje) < 5 ) : #roughly same speed in same direction
#                    GPS_boot_new = add_offset(GPS_boje_data, angle, offset)
#
#                    #send corrected data to uboot
#                    sock_boot.sendto(bytes(str(GPS_boot_new)), (BOOT_IP, UDP_PORT))
#                    #send og data
#                    sock_gc.sendto(bytes(str(GPS_boje_data)))
#
#        
#            #except KeyboardInterrupt:
#                #ctrl+c cleanup
#                
#                #server_proc.kill()
#
#
#            # Display basic boje state
#            #print "==========SAMPLE VALUES=========="
#            #print " Type: %s" %boje._vehicle_type
#            #print " Armed: %s" % boje.armed
#            #print " System status: %s" % boje.system_status.state
#            #print " GPS: %s" % boje.gps_0
#            #print " Alt: %s" % boje.location.global_relative_frame.alt
#            #print " Battery: %s" % boje.battery
#            #print " Groundspeed : %s" % boje.groundspeed
#            #print " heading : %s" % boje.heading
#            #print "================================="
#            #time.sleep(3)
#    except pynmea2.ParseError as e:
#        print('Parse error: {}'.format(e))
#        continue
#    #except serial.SerialException:
#    #    print("Couldnt send to the socket-server")
#    #    continue