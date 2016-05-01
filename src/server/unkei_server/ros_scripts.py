import socket
import subprocess
import rostopic
import time
import os
import sys #sys.argv

UnkeiRoot = '/projects/eecs395/unkei'
ImageDir = os.path.expanduser('~') + UnkeiRoot + '/src/server/images'
BagFile = os.path.expanduser('~') + UnkeiRoot + '/src/server/bag.bag'
LaunchFile = os.path.expanduser('~') + UnkeiRoot + '/src/server/pipeline/DPPTAM/dpptam.launch'
SourceFile = os.path.expanduser('~') + UnkeiRoot + '/src/server/pipeline/DPPTAM/devel/setup.bash'

def create_bag():
    try:
        rostopic.rosgraph.Master('/rostopic').getPid()
        create = subprocess.Popen(['rosrun', 'BagFromImages', 'BagFromImages', ImageDir, '.png', '30', BagFile])#.wait()
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
        launchROS()
    else:
        return True
    
def launchROS():
    try:
        print "launching", LaunchFile
        launch = subprocess.Popen(['roslaunch', LaunchFile])#, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    except:
        raise rostopic.ROSTopicException("Can't launch ROS Master")
    else:
        return launch


def sourceROS():
    pipe = subprocess.Popen(". %s; env" % SourceFile, stdout=subprocess.PIPE, shell=True)
    output = pipe.communicate()[0]
    env = dict((line.split("=", 1) for line in output.splitlines()))
    os.environ.update(env)

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
