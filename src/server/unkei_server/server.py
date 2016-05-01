import socket
# import pdb
# import subprocess
import time
# import rostopic
from image_converter import img_converter
from purge import purge
from ros_scripts import checkROS
# from ros_scripts import launchROS
from ros_scripts import create_bag
from ros_scripts import play_bag

s = socket.socket()
port = 8080
s.bind(('', port))
r = socket.socket()
rport = 8082
s.listen(5)
while True:
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
    checkROS()
    time.sleep(20)
    create_bag()

    time.sleep(10)
    launchROS()
    time.sleep(2)

    play_bag()
    print 'creating stl'
    r.bind((addr[0], rport))
    # time.sleep(30)
    # stl = open('banister_knob.stl', 'rb')
    # buf = stl.read(1024)
    # print 'sending stl man'
    # while(buf):
        # r.send(buf)
        # buf = stl.read(1024)
