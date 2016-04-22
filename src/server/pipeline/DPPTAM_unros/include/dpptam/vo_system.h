#ifndef __VO_SYSTEM_H
#define __VO_SYSTEM_H

#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include <stdio.h>
#include <cstdio>
#include <vector>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>

#include <dpptam/superpixel.h>
#include <dpptam/DenseMapping.h>
#include <dpptam/SemiDenseTracking.h>
#include <dpptam/SemiDenseMapping.h>
#include <dpptam/PlyListener.h>
//class PlyListener;
//class SemiDenseMapping;
//class SemiDenseTracking;
//class DenseMapping;
//class Imagenes;

using namespace std;
class vo_system
{
public:
    vo_system();
    static std::string GetCwd();

    void imgcb(const cv::Mat& image);

    PlyListener ply_listener;
    DenseMapping dense_mapper;
    SemiDenseTracking semidense_tracker;
    SemiDenseMapping semidense_mapper;
    MapShared Map;
    Imagenes images,images_previous_keyframe;

    int cont_frames;
    double stamps;
    cv::Mat image_frame,image_frame_aux;
    double depth_stamps;
    clock_t current_time, time_stamps;
    //ros::Time current_time,stamps_ros;

    //ros::NodeHandle nh;
    //image_transport::Subscriber sub1;
    //image_transport::Publisher pub_image;
    
    //MainWrapper *thing; // this is a pointer to the thing that gives out the images.

    //ros::Publisher odom_pub;


    //ros::Publisher pub_cloud;
    //ros::Publisher pub_poses;
    //ros::Publisher vis_pub;

    bool done;
    boost::mutex done_mutex;
    boost::condition_variable done_cond;

};
#endif



