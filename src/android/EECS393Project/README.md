# EECS393Project

This is the Android app side of Fragonard, our 3D scanning app.  In total, the app's functionality relies on 2 parts, the app and the server that are the server side and the app side. This functionality consists of 4 github repos : 
- The app : https://github.com/a-hacker/EECS393Project
- The server : https://github.com/Loeing/3DScanner_Server
- ORB-SLAM  : https://github.com/Loeing/ORB_SLAM   (which is a project by Raúl Mur Artal that we modified to try to implement this paper(http://webdiis.unizar.es/~raulmur/MurTardosRSS15.pdf)
- BagFromImages : https://github.com/raulmur/BagFromImages (which we did not write but is necessary to run the code for the server)

This app utilizes OpenCV. Running it on a device will require having downloaded OpenCV Manager onto your device (you will be prompted to do so if it isn't).

## Installations

To run the app, clone the app and install OpenCV following this guide: http://stackoverflow.com/questions/27406303/opencv-in-android-studio (note that steps 4 through 6 have already been completed).

Warning: the server has only been implemented and tested in Ubuntu 14.04 with ROS Indigo. We cannot garantie that this will work on any other system.

To successfully run the server you need to clone the server, ORB-SLAM and BagFrom images into the same folder, that we'll call Scanner3D. Follow the installation instructions in the ORB-SLAM readme to install ROS and ORB-SLAM. Make sure you set the ROS_PACKAGE_PATH to go to Scanner3D.

## Running the code

To run the app, you must build the apk using android studio and then run it as any other android app.

To run the server, simply go to Scanner3D (your folder above all of the repos) and run this line in the terminal: 

	python 3DScanner_Server/server.py

This will wait for the app to send it a scan.

If you just want to test that ORB_SLAM is functional you, in 2 separate terminals you can run:

	roslaunch ExampleGroovyOrNewer.launch 

	rosbag play --pause ORB_SLAM/Example.bag 
