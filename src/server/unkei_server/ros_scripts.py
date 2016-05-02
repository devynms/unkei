import socket
import subprocess
import rostopic
import os
import sys #sys.argv

UnkeiServerRoot = os.getcwd()
ImageDir = UnkeiServerRoot + '/images'
BagFile = UnkeiServerRoot + '/pipeline/DPPTAM/data/bags/scan.bag'
LaunchFile = UnkeiServerRoot + '/pipeline/DPPTAM/dpptam.launch'
SourceFile = UnkeiServerRoot + '/pipeline/DPPTAM/devel/setup.bash'

def create_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        create = subprocess.Popen(['rosrun', 'bag_from_images', 'bag_from_images', ImageDir, '.png', '30', BagFile])#.wait()
    except socket.error:
        print "could not create bag file"
        #print 'trying again'
        #create_bag()
    else:
        return create

def play_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        play = subprocess.Popen(['rosbag', 'play', BagFile])#.wait()
    except socket.error:
        raise rostopic.ROSTopicIOException("can't run ORB_SLAM")
    else:
        return play

def checkROS():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
    except socket.error:
        return False
    else:
        return True

def launchROS():
    if not checkROS():
        try:
            print "starting ROS core"
            subprocess.Popen(['roscore'])
        except socket.error:
            print "can't start ROS core"
            return False

def startDPPTAM():
    try:
        # print "launching", LaunchFile
        launch = subprocess.Popen(['rosrun', 'dpptam', 'dpptam'])#, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    except:
        raise rostopic.ROSTopicException("Can't launch ROS Master")
    else:
        return launch

if __name__ == '__main__':
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "checkROS":
            checkROS()
        elif arg == "launchROS":
            if len(sys.argv) > 2:
                LaunchFile = sys.argv[2]
            launchROS()
        elif arg == "create_bag":
            if len(sys.argv) > 2:
                ImageDir = sys.argv[2]
                if len(sys.argv) > 3:
                    BagFile = sys.argv[3]
            create_bag()
        elif arg == "play_bag":
            if len(sys.argv) > 2:
                BagFile = sys.argv[2]
            play_bag()
        else:
            print "Unknown argument"
    else:
        print "Please enter the name of the function you wish to run"

