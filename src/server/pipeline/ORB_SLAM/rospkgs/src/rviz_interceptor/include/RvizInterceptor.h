#ifndef RVIZ_INTERCEPTOR_H
#define RVIZ_INTERCEPTOR_H 

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <tf2_msgs/TFMessage.h>


// map points
#define MPOINTS_ID 0
// keyframes
#define KFRAMES_ID 1
// covisibility graph
#define COVIS_ID 2
// keyframes spanning tree
#define MST_ID 3
// current camera
#define CAM_ID 4
// reference map points
#define REF_ID 5

class RvizInterceptor {
    public:
        RvizInterceptor();

    private:
        // listen to messages published by MapPublisher.h
        ros::NodeHandle nh;
        ros::Subscriber mapListener; 
        ros::Subscriber frameListener; 
        ros::Subscriber tfListener; 

        // callback methods
        // ORB_SLAM/Map
        void MapMsgCallback(const boost::shared_ptr<visualization_msgs::Marker const>& msg);
        // ORB_SLAM/Frame
        void FrameMsgCallback(const boost::shared_ptr<sensor_msgs::Image const>& msg);
        // ORB_SLAME/World and ORB_SLAM/Camera
        void TFMsgCallback(const boost::shared_ptr<tf2_msgs::TFMessage const>& msg);

        // processing methods
        void MapProcessMapPoints(const boost::shared_ptr<visualization_msgs::Marker const>& m);
        void MapProcessKeyFrames(const boost::shared_ptr<visualization_msgs::Marker const>& m);
        void MapProcessGraph(const boost::shared_ptr<visualization_msgs::Marker const>& m);
        void MapProcessMST(const boost::shared_ptr<visualization_msgs::Marker const>& m);
        void MapProcessCamera(const boost::shared_ptr<visualization_msgs::Marker const>& m);
        void MapProcessRefPoints(const boost::shared_ptr<visualization_msgs::Marker const>& m);

        

        // message holder variables
        //visualization_msgs::Marker MapPoints;
        //visualization_msgs::Marker ReferencePoints;
        //visualization_msgs::Marker KeyFrames;
        //visualization_msgs::Marker CovisibilityGraph;
        //visualization_msgs::Marker MST;
        //visualization_msgs::Marker CurrentCamera;

};
#endif
