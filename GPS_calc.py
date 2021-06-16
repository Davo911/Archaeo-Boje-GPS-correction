from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
from subprocess import Popen

# Start MavProxy
print "Start MavProxy Server..."
logfile = open("./mavprxy.log", "w")
server_proc = Popen(["mavproxy.py", "--out", "127.0.0.1:14550"],stdout=logfile)
time.sleep(5)

# Connect to the boje
print "Connecting..."
connection_boje = '127.0.0.1:14550'
connection_boot = '192.168.2.2:14550'
boje = connect(connection_boje, wait_ready=True)
boot = connect(connection_boot, wait_ready=True)



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