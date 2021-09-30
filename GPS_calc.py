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

def add_offset(GPS_obj, ang, os):

    #CALCULATION
    lat0 = math.cos(math.PI / 180.0 * lat)
    lng_new = GPS_obj.longitude + (180/math.PI) * (os / 6378137)/math.cos(lat0) * math.cos(ang)
    lat_new = GPS_obj.latitude + (180/math.PI) * (os / 6378137) * math.sin(ang)

    """ 
    $GPGGA,102311.996,5102.140,N,01344.177,E,1,12,1.0,0.0,M,0.0,M,,*68

    5102.140N; 01344.177,E
    Latitude=51.0355758333and Longitude=13.7360121667    

    ('Timestamp', 'timestamp', timestamp),
    ('Latitude', 'lat'),
    ('Latitude Direction', 'lat_dir'),
    ('Longitude', 'lon'),
    ('Longitude Direction', 'lon_dir'),
    ('GPS Quality Indicator', 'gps_qual', int),
    ('Number of Satellites in use', 'num_sats'),
    ('Horizontal Dilution of Precision', 'horizontal_dil'),
    ('Antenna Alt above sea level (mean)', 'altitude', float),
    ('Units of altitude (meters)', 'altitude_units'),
    ('Geoidal Separation', 'geo_sep'),
    ('Units of Geoidal Separation (meters)', 'geo_sep_units'),
    ('Age of Differential GPS Data (secs)', 'age_gps_data'),
    ('Differential Reference Station ID', 'ref_station_id'),
    # ###### http://www.hiddenvision.co.uk/ez/
    """
    #GPS_boje = pynmea2.GGA('GP', 'GGA', ('timestamp', 'lat', 'lat_dir','lon', 'lon_dir', 'gps_qual', 'num_sats', 'horizontal_dil', 'altitude', 'altitude_units', 'geo_sep', 'geo_sep_units', 'age_gps_data', 'ref_station_id'))      
    
    GPS_new_data = pynmea2.GGA('GP', 'GGA', ('102311.996', decTodms(lat_new), GPS_obj.lat_dir,decTodms(lng_new), GPS_obj.lon_dir, GPS_obj.gps_qual, str(GPS_obj.num_sats), str(GPS_obj.horizontal_dil), str(GPS_obj.altitude), str(GPS_obj.altitude_units), str(GPS_obj.geo_sep), str(GPS_obj.geo_sep_units), str(GPS_obj.age_gps_data), str(GPS_obj.ref_station_id)))

    return GPS_new_data

def parseArguments():
    if len(sys.argv)>1 and sys.argv[1].split("=")[0]=="-s":
        try:
            string_length = float(sys.argv[1].split("=")[1])
        except ValueError:
            print("No String length defined ( -s=2.0 )")
            string_length = 2.0
    else:
        print("No String length defined ( -s=2.0 ) -2")
        string_length = 2.0

    print("String length:"+str(string_length))



# Start MavProxy
# mavproxy.py --out 127.0.0.1:14550 --show-errors --baudrate 115200
#print ("Start MavProxy Server...")
#logfile = open("./mavprxy.log", "w")
#server_proc = Popen(["mavproxy.py", "--out", "127.0.0.1:14550"], shell=True,stdout=logfile)
#time.sleep(5)

# Setup GPS serial port
#port_GPS = "/dev/ttyAMA0"
#ser = serial.Serial(port_GPS,baudrate=9600,timeout=0.5)

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
print ("Connecting...")
connection_boje = '127.0.0.1:14550'
connection_boot = '192.168.2.2:14550'
boje = connect(connection_boje,"baud=57600", wait_ready=False)
boje.wait_ready(True, timeout=180)
global gps_timestamp
gps_timestamp = '102311.996'

# def receivedPos(self, name, posMsg):
#     print (posMsg)
# boje.add_message_listener("GPS_INPUT",receivedPos)

# @boje.on_message('GPS_INPUT')
# def listener(self, name, message):
#     gps_timestamp = str(message.time_usec)
#try:
#    boot = connect(connection_boot, wait_ready=True)
#except Exception as e:
#    print("Connection Error to Boot-FC")
#    print('Parse error: {}'.format(e))
print("mavlink messages:")
@boje.on_message('GPS_RAW_INT')
def listener(self, name, message):
    utc = datetime(1970, 1, 1) + timedelta(seconds=int(message.time_usec)/1000)
    gps_timestamp = str(int(message.time_usec)/1000)
    #print("Date: "+utc.strftime("%H%M%S"))
    #print(str(utc)+"==>durch1000: "+gps_timestamp+"og: "+str(message.time_usec))

while True:
    time.sleep(2)

    #print("lat: "+str(boje.location.global_frame.lat))
    #print("lon: "+str(boje.location.global_frame.lon))
    

    #lat_dir = 'N' if boje.location.global_frame.lat > 0 else 'S'
    #lon_dir = 'E' if boje.location.global_frame.lon > 0 else 'W'
    #print("time= "+ gps_timestamp)
    #GPS_boje = pynmea2.GGA('GP', 'GGA', (gps_timestamp, decTodms(boje.location.global_frame.lat), lat_dir,decTodms(boje.location.global_frame.lon), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(boje.gps_0.eph), str(boje.location.global_frame.alt), 'M', '0.0', 'M', '', '0000'))
    #print(str(GPS_boje))

    GPS_boje_data = pynmea2.parse("$GPRMC,150559.00,A,5102.85348,N,01345.01389,E,0.212,,300421,,,A*7C")
    #print(str(GPS_boje_data))
    #print("sending ")
    #sock_boot.sendto(bytes(str(GPS_boje_data)), (BOOT_IP, BOOT_PORT))
    sock_gc.sendto(bytes(bytes(str(GPS_boje_data)), (GC_IP, GC_PORT))



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