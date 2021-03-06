/**
* This file is part of DPPTAM.
*
* Copyright (C) 2015 Alejo Concha Belenguer <alejocb at unizar dot es> (University of Zaragoza)
* and Javier Civera Sancho   <jcivera at unizar dot es> (University of Zaragoza)
*
* DPPTAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DPPTAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DPPTAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fstream>
#include <iomanip>    // Needed for stream modifiers fixed and set precision

#include <dpptam/vo_system.h>
//#include <dpptam/superpixel.h>
//#include <dpptam/DenseMapping.h>
//#include <dpptam/SemiDenseTracking.h>
//#include <dpptam/SemiDenseMapping.h>
//#include <dpptam/PlyListener.h>

std::string vo_system::GetCwd() {
    char buf[512];
    getcwd(buf, 511);
    return std::string(buf);
}

vo_system::vo_system(){


    ///vo_system launch the three threads, tracking, semidense mapping and dense mapping (3D superpixels)

    cv::FileStorage  fs2( (GetCwd()+"/src/data.yml"), cv::FileStorage::READ);

    std::string camera_path;
    fs2["camera_path"] >> camera_path;
    cont_frames = 0;

    int calculate_superpixels = (int)fs2["calculate_superpixels"];



    //image_transport::ImageTransport it(nh);
    //sub1 = it.subscribe(camera_path,1, & vo_system::imgcb,this);
    //KEYWORD
    // MainWrapper thing = new thing;....


    //odom_pub = nh.advertise<nav_msgs::Odometry>("odom1", 50);



    /// advertising 3D map and camera poses in rviz
    //pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("dpptam/map", 1);
    //pub_poses = nh.advertise<sensor_msgs::PointCloud2> ("points_poses", 1);
    //vis_pub = nh.advertise<visualization_msgs::Marker>( "dpptam/visualization_marker", 0 );
    /// advertising 3D map and camera poses in rviz


    /// pubishing current frame and the reprojection of the 3D map
    //pub_image = it.advertise("dpptam/camera/image",1);
    /// pubishing current frame and the reprojection of the 3D map


    semidense_tracker.cont_frames = &cont_frames;
    semidense_tracker.stamps = &stamps;
    semidense_tracker.image_frame = &image_frame_aux;
    semidense_tracker.time_stamps = &time_stamps;


    ///Launch semidense tracker thread*/
    boost::thread thread_semidense_tracker(&ThreadSemiDenseTracker,&images,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map, this);

    ///Launch semidense mapper thread
    boost::thread thread_semidense_mapper(&ThreadSemiDenseMapper,&images,&images_previous_keyframe,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map, this);


    if (calculate_superpixels > 0.5)
    {
        ///launch dense mapper thread
         boost::thread thread_dense_mapper(&ThreadDenseMapper,&dense_mapper, this);
    }

    // KEYWORD
    // launch monitor ply to pcd thread
    std::string pcd_dir = GetCwd() + "/data/meshes";//getcwd()+"/data/meshes";
    std::string mesh_file = GetCwd() + "/data/out.stl"; //getcwd()+"/data/out.stl";
    boost::thread thread_ply_listener(&ThreadPlyListener, &ply_listener, &dense_mapper, &semidense_mapper, this, pcd_dir, mesh_file);
    cout << "PlyListener thread launched!\n";
    
    cout << "***    DPPTAM is working     *** " <<  endl << endl;
    cout << "***    Launch the example sequences or use your own sequence / live camera and update the file 'data.yml' with the corresponding camera_path and calibration parameters    ***"  << endl;

}



void vo_system::imgcb(const cv::Mat& image) 
{
    ///read images
    //try {
        //cv_bridge::CvImageConstPtr cv_ptr;

        //cv_bridge::toCvShare(msg);
        //cv_ptr = cv_bridge::toCvShare(msg);

        //time_stamps =  cv_ptr->header.stamp;
        //stamps = cv_ptr->header.stamp.toSec();
        //current_time = cv_ptr->header.stamp;
        stamps = ((float)clock())/CLOCKS_PER_SEC;
        time_stamps = clock();
        current_time = clock();
        
        image_frame_aux =  image.clone();
        cont_frames++;
    //} 
    //catch (const std::exception& e)
    //{
        //ROS_ERROR("cv_bridge exception: %s", e.what());
    //    fprintf(stderr, "cv_bridge_exception: %s", e.what());
    //}
}

//remplaces imgcb
bool runVideo(string filepath)
{
    cv::VideoCapture capture(filepath);
    cv::Mat frame;

    if(!capture.isOpened())
        throw "Error reading the video file";

    while(!frame.empty()){
        capture >> frame;
        image_frame_aux = frame.clone();
    }
    return true;
}

