import socket
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
    time.sleep(60)
    startDPPTAM()
    play_bag()
    print 'creating .stl file'
    # time.sleep(30)
    r.bind((addr[0], rport))
    stl = open('banister_knob.stl', 'rb')
    buf = stl.read(1024)
    print 'sending .stl file'
    while(buf):
        r.send(buf)
        buf = stl.read(1024)
    print 'file sent'
    stl.close()
    r.close()

