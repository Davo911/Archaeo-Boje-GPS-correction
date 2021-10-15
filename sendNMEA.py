import os,sys,time
import socket

#f = open(sys.argv[1], 'r')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    with open(sys.argv[1]) as f:
        for line in f:
            time.sleep(0.05)
            print(line)
            sock.sendto(bytes(line, 'utf-8'), (sys.argv[2], 27000))
    
    
    
    
    
    
    

