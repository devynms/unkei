import socket
import subprocess
import rostopic
import unittest
import time
import shutil
import os
from purge import purge
from image_converter import img_converter
from ros_scripts import launchROS
from ros_scripts import checkROS
from ros_scripts import play_bag
from ros_scripts import create_bag


class TestServer(unittest.TestCase):
    
    def test_img_converter(self):
        purge("images")
        img_converter('test.mp4')
        self.assertNotEqual(os.listdir('images'),[])


    def test_purge(self):
        shutil.move("/home/josh/Workspace/Scanner3D/TestPics/tears.jpg", "/home/josh/Workspace/Scanner3D/images/")
        purge("images")
        self.assertEqual(os.listdir('images'),[])

    def test_ros_launch(self):
        launch = launchROS()
        time.sleep(5)
        try:
            rostopic.rosgraph.Master('/rostopic').getPid()
        except socket.error:
            self.fail("couldn't start Ros")
        else:
            launch.kill()
    
    def test_checkROS(self):
        launch = launchROS()
        time.sleep(5)
        check = checkROS
        self.assertTrue(check)
        launch.kill()

    def test_create_bag(self):
        try:
            launch = launchROS()
            time.sleep(3)
            img_converter("Test.mp4")
            create_bag()
            time.sleep(5)
            
        except:
            self.fail(" create_bag failed to create bag")

    def test_play_bag(self):
        launch = launchROS()
        time.sleep(5)
        try:

            play_bag()
            #subprocess.Popen(['rosrun', 'image_view', 'image_view', 'image:=/ORB_SLAM/Frame' '_autosize:=true'], stdin = subprocess.PIPE, stderr = subprocess.PIPE, stdout = subprocess.PIPE)
        except:
            self.fail("can't find last image")
        
        


    def test_server(self):
        try:
            server = subprocess.Popen(['python','3DScanner_Server/server.py'])
            time.sleep(5)
            s = socket.socket()
            s.connect(("172.19.94.120",8080))
            s.close()
            server.kill()
        except socket.error:
            self.fail("couldn't connect to server")





if __name__ == '__main__':
    unittest.main()
