import socket
import subprocess
import rostopic
import time

def create_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        create = subprocess.Popen(['rosrun', 'BagFromImages', 'BagFromImages', 'images/', '.png', '30', 'ORB_SLAM/Data/Example.bag']).wait()
    except socket.error:
        print 'trying again'
        create_bag()
    else:
        return create

def play_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        play = subprocess.Popen(['rosbag', 'play', 'ORB_SLAM/Data/Example.bag']).wait()
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
        launch = subprocess.Popen(['roslaunch', 'ORB_SLAM/ExampleGroovyOrNewer.launch'], stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    except:
        raise rostopic.ROSTopicException("Can't launch ROS Master")
    else:
        return launch
