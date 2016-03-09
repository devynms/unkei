#include "RvizInterceptor.h"
#include <string.h>

RvizInterceptor::RvizInterceptor() {
    // configure node listener
    mapListener = nh.subscribe("ORB_SLAM/Map", 10, &RvizInterceptor::MapMsgCallback, this);
    frameListener = nh.subscribe("ORB_SLAM/Frame", 10, &RvizInterceptor::FrameMsgCallback, this);
    tfListener = nh.subscribe("/tf", 10, &RvizInterceptor::TFMsgCallback, this);
}

void RvizInterceptor::MapMsgCallback(const boost::shared_ptr<visualization_msgs::Marker const>& msg) {
    // namespaces used by MapPublisher:
    // const char* POINTS_NAMESPACE = "MapPoints";
    // const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    // const char* GRAPH_NAMESPACE = "Graph";
    // const char* CAMERA_NAMESPACE = "Camera";

    if (!strcmp(msg->ns.c_str(), "MapPoints")) {
        if (msg->id == MPOINTS_ID) {
            MapProcessMapPoints(msg);
        } else if (msg->id == REF_ID) {
            MapProcessRefPoints(msg);
        }
    } else if (!strcmp(msg->ns.c_str(), "KeyFrames")) {
        MapProcessKeyFrames(msg);
    } else if (!strcmp(msg->ns.c_str(), "Graph")) {
        if (msg->id == COVIS_ID) {
            MapProcessGraph(msg);
        } else if (msg->id == MST_ID) {
            MapProcessMST(msg);
        }
    } else if (!strcmp(msg->ns.c_str(), "Camera")) {
        MapProcessCamera(msg);
    } else {
        ROS_INFO_STREAM("RvizInterceptor: Unknown Message Type: " << msg->ns.c_str());
    }
}

void RvizInterceptor::FrameMsgCallback(const boost::shared_ptr<sensor_msgs::Image const>& msg) {
    // the last processed frame:
    //cv::Mat im(msg->width, msg->height, CV_8UC1, msg->data, msg->step);
    ROS_INFO_STREAM("Received Frame message\n");
}

void RvizInterceptor::TFMsgCallback(const boost::shared_ptr<tf2_msgs::TFMessage const>& msg) {
    ROS_INFO_STREAM("Received TF message\n");
}

void RvizInterceptor::MapProcessMapPoints(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from Map Points\n" << m->points.size());
}

void RvizInterceptor::MapProcessKeyFrames(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from KeyFrame\n" << m->points.size());
}

void RvizInterceptor::MapProcessGraph(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from Covisibility Graph\n" << m->points.size());
}

void RvizInterceptor::MapProcessMST(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from Minimum Spanning Tree\n" << m->points.size());
}

void RvizInterceptor::MapProcessCamera(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from Camera \n" << m->points.size());
}

void RvizInterceptor::MapProcessRefPoints(const boost::shared_ptr<visualization_msgs::Marker const>& m) {
    ROS_INFO_STREAM("Received %d geometry_msgs::Point's from Reference Points\n" << m->points.size());
}

