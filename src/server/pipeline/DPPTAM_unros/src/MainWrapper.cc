/*
 * =====================================================================================
 *
 *       Filename:  MainWrapper.cc
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  04/21/2016 06:44:16 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  josh
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include <MainWrapper.h>

bool runVideo(string filepath){
  cv::VideoCapture capture(filepath);
  cv::Mat frame;

  if(!capture.isOpened())
    throw "Error reading the video file";

  while(!frame.empty()){
    capture >> frame;

  }
  return true;
}
