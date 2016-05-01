#! /bin/bash

# project directories
UnkeiRoot="/projects/eecs395/unkei"
ImageDir=$HOME$UnkeiRoot"/src/server/3DScanner_Server/images"
BagFile=$HOME"/workspaces/ros_catkin/src/dpptam/data/lab_unizar.bag"
#LaunchFile=$HOME"/workspaces/ros_catkin/src/dpptam/dpptam.launch"
#BagFile=$HOME$UnkeiRoot"/src/server/pipeline/DPPTAM/data/example.bag"
LaunchFile=$HOME$UnkeiRoot"/src/server/pipeline/DPPTAM/dpptam.launch"
ImExt=".png"
Rate=30

_DpptamSourceFile=$HOME$UnkeiRoot"/src/server/pipeline/DPPTAM/devel/setup.bash"
#_BagSourceFile=$HOME"/workspaces/ros_catkin/src/BagFromImages/build/devel/setup.bash"
_RosSourceFile="/opt/ros/indigo/setup.bash"
SourceFiles=($_DpptamSourceFile $_BagSourceFile $_RosSourceFile)

# ros variables
rosmaster_pid=""


get_rosmaster_pid() {
    rosmaster_pid=`ps -A | grep rosmaster | awk '{print $1}'`
    echo "rosmaster pid: "$rosmaster_pid
}

kill_ros() {
    get_rosmaster_pid 
    count=0
    pid=`ps -A | grep ros | awk '{print $1}' | head -n1`
    while [ "$pid" -a $count -lt 20 ]; do
        kill $pid & disown
        pid=`ps -A | grep ros | awk '{print $1}' | head -n1`
        let count=count+1
    done
    sleep 3
    get_rosmaster_pid 
}

check_ros() {
    get_rosmaster_pid 
    
    if [ -z $rosmaster_pid ]; then
        roscore & disown
        sleep 3
        get_rosmaster_pid 
        if [ -z $rosmaster_pid ]; then
            echo "Unable to launch ros"
            exit
        fi
    fi
    
    echo "rosmaster up and running. pid: "$rosmaster_pid
    source_ros
}

source_ros() {
    for f in ${SourceFiles[*]}; do
        source $f
        sleep 1
        echo "sourced "$f
    done
}

purge() {
    if [ "$1" ]; then
        rm -rf $1
        mkdir $1
    fi
}

create_bag() {
    check_ros
    if [ $rosmaster_pid ]; then
        #usage: rosrun BagFromImages BagFromImages PATH_TO_IMAGES IMAGE_EXTENSION FREQUENCY PATH_TO_OUPUT_BAG
        imagedir=${1:-$ImageDir}
        bagfile=${2:-$BagFile}
        imext=${3:-$ImExt}
        rate=${4:-$Rate}
        #source_ros
        echo "running command: rosrun BagFromImages BagFromImages "$imagedir" "$imext" "$rate" "$bagfile" & disown"
        rosrun BagFromImages BagFromImages $imagedir $imext $rate $bagfile & disown
    else
        echo "error: no ros process is executing"
    fi
}

play_bag() {
    get_rosmaster_pid
    if [ $rosmaster_pid ]; then
        bagfile=${1:-$BagFile}
        source_ros
        echo "running command: rosbag play "$bagfile" & disown"
        rosbag play $bagfile & disown 
        sleep 3
        echo "playing rosbag: "$bagfile
    else
        echo "error: no ros process is executing"
    fi
}

launch_ros() {
    get_rosmaster_pid
    if [ $rosmaster_pid ]; then
        launchfile=${1:-$LaunchFile}
        source_ros
        echo "running command: roslaunch "$launchfile" & disown"
        roslaunch $launchfile & disown
        sleep 3
        echo "launching roslaunch file: "$launchfile
    else
        echo "error: no ros process is executing"
    fi
}

# main: execute the specified function with any additional arguments
$1 $2 $3 $4 $5 $6 $7 $8 $9

