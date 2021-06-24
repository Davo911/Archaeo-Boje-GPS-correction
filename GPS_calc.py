from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math
from subprocess import Popen
import pynmea2

def roughly_equal(a,b):
    #TODO: SMARTER LOGIC
    
    return a==b

def add_offset(GPS_obj, ang, os):

    #CALCULATION
    lat0 = math.cos(math.PI / 180.0 * lat)
    lng_new = GPS_obj.longitude + (180/math.PI) * (os / 6378137)/math.cos(lat0) * math.cos(ang)
    lat_new = GPS_obj.latitude + (180/math.PI) * (os / 6378137) * math.sin(angle)

    #TODO:GENERATE & RETURN NEW GPS OBJECT                   
    GPS_new_data = pynmea2.GGA('GP', 'GGA', ('184353.07', '1929.045', 'S', '02410.506', 'E', '1', '04', '2.6', '100.00', 'M', '-33.9', 'M', '', '0000'))

    return GPS_new_data

#cable length                                                               TODO: MAKE IT AN ARGUMENT
string_length = 2.0 

# Start MavProxy
print "Start MavProxy Server..."
logfile = open("./mavprxy.log", "w")
server_proc = Popen(["mavproxy.py", "--out", "127.0.0.1:14550"],stdout=logfile)
time.sleep(5)

# Setup GPS serial port
port_GPS = "/dev/serial0"
ser = serial.Serial(port_GPS,baudrate=9600,timeout=0.5)

#GC socket
BOOT_IP = "192.168.2.1"
UDP_PORT = 27000

# Open socket to Boot-pi
BOOT_IP = "192.168.2.2"
UDP_PORT = 27000
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Connect to the MavProxies
print "Connecting..."
connection_boje = '127.0.0.1:14550'
connection_boot = '192.168.2.2:14550'

try:
    boje = connect(connection_boje, wait_ready=True)
except Exception:
    print("Connection Error to Boje-FC")
#try:
#    boot = connect(connection_boot, wait_ready=True)
#except Exception:
#    print("Connection Error to Boot-FC")

while True:
    try:
        #depth = boot.location.global_relative_frame.alt
        depth = 1.5
        offset = math.sqrt((string_length**2)-(depth**2))       
        #compass_boot = boot.heading
        compass_boje = boje.heading
        speed_boot = 1 #boot.groundspeed
        speed_boje = boje.groundspeed
        GPS_boje_data = pynmea2.parse(ser.readline())
        angle = compass_boje                                    #TODO: calculate angle from compasses
        print("Battery: %s" % boje.battery)
        print("offset: "+str(offset))
        print("compass: "+str(compass_boje))
        print("speed: "+str(speed_boje))
        print("GPSdata: "+str(GPS_boje_data))
        
        
        if (roughly_equal(speed_boje, speed_boot) and (compass_boot - compass_boje) < 5 ) :
            GPS_boot_new = add_offset(GPS_boje_data, angle, offset)
            bytestosend = bytes(newmsg)
            sock.sendto(bytestosend, (BOOT_IP, UDP_PORT))

    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue
    except serial.SerialException:
        print("Couldnt send to the socket-server")
        continue
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