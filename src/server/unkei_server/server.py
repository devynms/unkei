import socket
import os
# import pdb
# import subprocess
import time
# import rostopic
from image_converter import img_converter
from purge import purge
from ros_scripts import checkROS
from ros_scripts import launchROS
from ros_scripts import create_bag
from ros_scripts import play_bag
from ros_scripts import startDPPTAM

launchROS()
s = socket.socket()
port = 8080
s.bind(('', port))
r = socket.socket()
rport = 8082
s.listen(5)
while checkROS():
    print 'waiting'
    c, addr = s.accept()
    print 'Connected to', addr
    f = open('stored.mp4', 'wb')
    packet = c.recv(1024)
    print 'receiving video ...'
    while (packet):
        f.write(packet)
        packet = c.recv(1024)
    f.close()
    print 'Video Received'
    c.close()
    print 'purge old images...'
    purge('images')
    img_converter('stored.mp4')
    print 'checking ROS...'
    create_bag()
    startDPPTAM()
    time.sleep(60)
    play_bag()
    print 'creating .stl file'
    # time.sleep(30)
    stl = open('banister_knob.stl', 'rb')
    size = os.fstat(stl.fileno()).st_size
    print 'connecting to device'
    r.connect((addr[0], rport))
    print 'sending size: ' + str(size)
    r.send(str(size).encode())
    buf = stl.read(1024)
    print 'sending .stl file'
    # b = 1024
    while(buf):
        r.send(buf)
        buf = stl.read(1024)
        # print b
        # b = b + 1024
    print 'file sent'
    stl.close()
    r.close()
