/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:  
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederic 
 *   Organization:  
 *
 * =====================================================================================
 */
#include "ProbabilityMapping.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <vector>


void firstLoop(KeyFrame kf, depthHo**, std::vector<depthHo>&){;}

void stereoSearch_constraints(){;}
	        
void epipolarSearch(){;}

////////////////////////
// Utility functions
////////////////////////

void getImageGradient(cv::Mat& image, cv::Mat* grad){
	cv::Mat gradx, grady;
	
	cv::Scharr(image, gradx, CV_16S,1,0);
	cv::Scharr(image, grady, CV_16S,0,1);
	
	cv::Mat absgradx, absgrady;

	cv::convertScaleAbs(gradx, absgradx);
	cv::convertScaleAbs(grady, absgrady);
	cv::addWeighted(absgradx,0.5,absgrady,0.5,grad,0);
}

void getGradientOrientation(cv::Mat& grad, float th){

}

void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2){

}
		         
