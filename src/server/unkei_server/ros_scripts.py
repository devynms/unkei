import socket
import subprocess
import rostopic
import time
import os

UnkeiServerRoot = os.getcwd()
ImageDir = os.path.expanduser('~') + UnkeiServerRoot + '/images'
BagFile = os.path.expanduser('~') + UnkeiServerRoot + '/pipeline/scan.bag'
LaunchFile = os.path.expanduser('~') + UnkeiServerRoot + '/pipeline/DPPTAM/dpptam.launch'
SourceFile = os.path.expanduser('~') + UnkeiServerRoot + '/pipeline/DPPTAM/devel/setup.bash'

def create_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        create = subprocess.Popen(['rosrun', 'bag_from_images', 'bag_from_images', ImageDir + '/', '.png', '30', BagFile]).wait()
    except socket.error:
        print "could not create bag file"
        #print 'trying again'
        #create_bag()
    else:
        return create

def play_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        play = subprocess.Popen(['rosbag', 'play', BagFile]).wait()
    except socket.error:
        raise rostopic.ROSTopicIOException("can't run ORB_SLAM")
    else:
        return play

def checkROS():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
    except socket.error:
        launchROS()
    else:
        return True

def launchROS():
    try:
        launch = subprocess.Popen(['roslaunch', LaunchFile], stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    except:
        raise rostopic.ROSTopicException("Can't launch ROS Master")
    else:
        return launch

def source(script):
    pipe = subprocess.Popen(". %s; env" % script, stdout=subprocess.PIPE, shell=True)
    output = pipe.communicate()[0]
    env = dict((line.split("=", 1) for line in output.splitlines()))
    os.environ.update(env)

def sourceROS():
    source(SourceFile)

if __name__ == '__main__':
    create_bag()
    launchROS()
    #play_bag()
