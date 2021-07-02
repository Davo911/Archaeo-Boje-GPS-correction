from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math
from subprocess import Popen
import pynmea2


def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return [d, m, s]

def roughly_equal(a,b):
    #TODO: SMARTER LOGIC
    return a==b

def add_offset(GPS_obj, ang, os):

    #CALCULATION
    lat0 = math.cos(math.PI / 180.0 * lat)
    lng_new = GPS_obj.longitude + (180/math.PI) * (os / 6378137)/math.cos(lat0) * math.cos(ang)
    lat_new = GPS_obj.latitude + (180/math.PI) * (os / 6378137) * math.sin(angle)

    #TODO:GENERATE & RETURN NEW GPS OBJECT
    #     
    #$GPGGA,102311.996,5102.140,N,01344.177,E,1,12,1.0,0.0,M,0.0,M,,*68
    #Latitude=51.0355758333and Longitude=13.7360121667    
    # ('Timestamp', 'timestamp', timestamp),
    #('Latitude', 'lat'),
    #('Latitude Direction', 'lat_dir'),
    #('Longitude', 'lon'),
    #('Longitude Direction', 'lon_dir'),
    #('GPS Quality Indicator', 'gps_qual', int),
    #('Number of Satellites in use', 'num_sats'),
    #('Horizontal Dilution of Precision', 'horizontal_dil'),
    #('Antenna Alt above sea level (mean)', 'altitude', float),
    #('Units of altitude (meters)', 'altitude_units'),
    #('Geoidal Separation', 'geo_sep'),
    #('Units of Geoidal Separation (meters)', 'geo_sep_units'),
    #('Age of Differential GPS Data (secs)', 'age_gps_data'),
    #('Differential Reference Station ID', 'ref_station_id'),
          
    GPS_new_data = pynmea2.GGA('GP', 'GGA', ('102311.996', '5102.140', 'N', '01344.177', 'E', '1', '12', '1.0', '0.0', 'M', '0.0', 'M', '', '*68'))

    return GPS_new_data

def parseArguments():
    if len(sys.argv)>1 and sys.argv[1].split("=")[0]=="-s":
        try:
            string_length = float(sys.argv[1].split("=")[1])
        except ValueError:
            print("No String length defined ( -s=2.0 )")
            string_length = 2.0
    else:
        print("No String length defined ( -s=2.0 )2")
        string_length = 2.0

    print("String length:"+str(string_length))



# Start MavProxy
print ("Start MavProxy Server...")
#logfile = open("./mavprxy.log", "w")
#server_proc = Popen(["mavproxy.py", "--out", "127.0.0.1:14550"], shell=True)#,stdout=logfile)
#time.sleep(5)

# Setup GPS serial port
port_GPS = "/dev/ttyAMA0"
ser = serial.Serial(port_GPS,baudrate=9600,timeout=0.5)

#GC socket
BOOT_IP = "192.168.2.1"
UDP_PORT = 27000
sock_gc = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Open socket to Boot-pi
BOOT_IP = "192.168.2.2"
UDP_PORT = 27000
sock_boot = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Connect to the MavProxies
print ("Connecting...")
connection_boje = '127.0.0.1:14550'
connection_boot = '192.168.2.2:14550'
boje = connect(connection_boje,"baud=57600", wait_ready=False)
#boje.wait_ready(True, timeout=180)

#try:
#    boot = connect(connection_boot, wait_ready=True)
#except Exception:
#    print("Connection Error to Boot-FC")

while True:
    time.sleep(2)
    try:
#        depth = boot.location.global_relative_frame.alt
#        compass_boot = boot.heading
#        speed_boot = boot.groundspeed
#        GPS_boje_data = pynmea2.parse(ser.readline())
        depth = 1.5
        #offset = math.sqrt((string_length**2)-(depth**2))       
        compass_boje = boje.heading
        speed_boje = boje.groundspeed
        #angle = Math.toRadians(compass_boje)
                                            #TODO: HOW DO WE GET THIS ANGLE???! --> north/east = Math.toRadians(45 * (2 * n - 1)); | where n = [1, 2, 3, 
        print("Battery: %s" % boje.battery)
        #print("offset: "+str(offset))
        print("compass: "+str(compass_boje))
        print("speed: "+str(speed_boje))
        print("GPS: %s" % boje.gps_0)
        print("Location: %s" % boje.location.global_frame)

        if (roughly_equal(speed_boje, speed_boot) and (compass_boot - compass_boje) < 5 ) : #roughly same speed in same direction
                    GPS_boot_new = add_offset(GPS_boje_data, angle, offset)

                    #send corrected data to uboot
                    sock_boot.sendto(bytes(str(GPS_boot_new)), (BOOT_IP, UDP_PORT))
                    #send og data
                    sock_gc.sendto(bytes(str(GPS_boje_data)))

            
            #except KeyboardInterrupt:
                #ctrl+c cleanup
                
                #server_proc.kill()


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
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue
    except serial.SerialException:
        print("Couldnt send to the socket-server")
        continue