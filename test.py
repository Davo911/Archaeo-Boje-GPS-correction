#from dronekit import connect, Command, LocationGlobal
#from pymavlink import mavutil
import io, time, sys, argparse, math, socket, math
#from subprocess import Popen
#import pynmea2

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