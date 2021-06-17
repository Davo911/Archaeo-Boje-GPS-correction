from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math
from subprocess import Popen
import pynmea2

def roughly_equal(a,b):
    #TODO: SMARTER LOGIC
    return a==b

#CALCULATION
def add_offset_lnglat(lng, lat, ang, os):
    lat0 = math.cos(math.PI / 180.0 * lat)
    lng_new = lng + (180/math.PI) * (os / 6378137)/math.cos(lat0) * math.cos(ang)
    lat_new = lat + (180/math.PI) * (os / 6378137) * math.sin(angle)

#cable length
string_length = 2.0 

# Start MavProxy
print "Start MavProxy Server..."
logfile = open("./mavprxy.log", "w")
server_proc = Popen(["mavproxy.py", "--out", "127.0.0.1:14550"],stdout=logfile)
time.sleep(5)

# Setup GPS serial port
port_GPS = "/dev/serial0"
ser = serial.Serial(port_GPS,baudrate=9600,timeout=0.5)
dataout = pynmea2.NMEAStreamReader()

# open socket to Boot-pi
BOOT_IP = "192.168.2.2"
UDP_PORT = 27000
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Connect to the MavProxies
print "Connecting..."
connection_boje = '127.0.0.1:14550'
connection_boot = '192.168.2.2:14550'
boje = connect(connection_boje, wait_ready=True)
boot = connect(connection_boot, wait_ready=True)

try:
    while True:
        depth = boot.location.global_relative_frame.alt
        offset = sqrt((string**2)-(depth**2))
        compass_boot = boot.heading
        compass_boje = boje.heading
        speed_boot = boot.groundspeed
        speed_boje = boje.groundspeed
        GPS_obj = pynmea2.parse(ser.readline())
        angle = math.toRadians(50)                 #TODO: calculate angle from compasses
        
        if (roughly_equal(speed_boje, speed_boot)) :
            #TODO: find a way to generate NMEA
            add_offset_lnglat(GPS_obj.longitude, GPS_obj.latitude, angle, offset)
        
        bytestosend = bytes(newmsg)
        sock.sendto(bytestosend, (BOOT_IP, UDP_PORT))
except KeyboardInterrupt:
    #ctrl+c cleanup
    server_proc.kill()


    # Display basic boje state
    #print "==========SAMPLE VALUES=========="
    #print " Type: %s" %boje._vehicle_type
    #print " Armed: %s" % boje.armed
    #print " System status: %s" % boje.system_status.state
    #print " GPS: %s" % boje.gps_0
    #print " Alt: %s" % boje.location.global_relative_frame.alt
    #print " Battery: %s" % boje.battery
    #print " Groundspeed : %s" % boje.groundspeed
    #print " heading : %s" % boje.heading
    #print "================================="
    #time.sleep(3)