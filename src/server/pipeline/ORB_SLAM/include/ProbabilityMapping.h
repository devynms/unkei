/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.h
 *
 *    Description:  Semi-dense Probability Mapping Module for ORB-SLAM
 *    inspired by Raul-Mur Artal's paper
 *
 *        Version:  0.01
 *        Created:  01/21/2016 03:48:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Authors: Josh Tang, Rebecca Frederick 
 *   Organization:  Unkei
 *
 * =====================================================================================
 */

#ifndef PROBABILITYMAPPING_H
#define PROBABILITYMAPPING_H

#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <numeric>

#define covisN 7
#define sigmaI 20
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3
#define histo_length 30
#define th_high = 100
#define th_low = 50
#define nnratio 0.6

namespace ORB_SLAM {
class KeyFrame;
} 

namespace cv {
class Mat;
}


class ProbabilityMapping {
public:

	struct depthHo {
		float depth;
		float sigma;
        };

        ProbabilityMapping();
	/* * \brief void first_loop(ORB_SLAM::KeyFrame kf, depthHo**, std::vector<depthHo>*): return results of epipolar search (depth hypotheses) */
	void FirstLoop(ORB_SLAM::KeyFrame *kf, depthHo*** ho);
        /* * \brief void stereo_search_constraints(): return min, max inverse depth */
        void StereoSearchConstraints(ORB_SLAM::KeyFrame* kf, float* min_depth, float* max_depth);
	/* * \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
        void EpipolarSearch(ORB_SLAM::KeyFrame *kf1, ORB_SLAM::KeyFrame *kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh);
	/* * \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist): 
	 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
        void InverseDepthHypothesisFusion(const std::vector<depthHo*>& h, depthHo* dist);
	/* * \brief void intraKeyFrameDepthChecking(depthHo** h, int imrows, int imcols): intra-keyframe depth-checking, smoothing, and growing. */
        void IntraKeyFrameDepthChecking(depthHo*** ho, int imrows, int imcols);
	/* * \brief void interKeyFrameDepthChecking(ORB_SLAM::KeyFrame* currentKF, depthHo** h, int imrows, int imcols): 
         * *         inter-keyframe depth-checking, smoothing, and growing. */
        void InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM::KeyFrame* currentKF, depthHo*** h);//int imrows, int imcols);

private:

        void GetTR(ORB_SLAM::KeyFrame* kf, cv::Mat* t, cv::Mat* r);
        void GetXp(const cv::Mat& K, int x, int y, cv::Mat* Xp);
        void GetParameterization(const cv::Mat& F12, const int x, const int y, float* a, float* b, float* c);
        void ComputeInvDepthHypothesis(ORB_SLAM::KeyFrame* kf, int pixel, float ustar, float ustar_var, float a, float b, float c, depthHo* dh);
        void GetImageGradient(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad);
        void GetGradientOrientation(int x, int y, const cv::Mat& gradx, const cv::Mat& grady, float* th);
        void GetInPlaneRotation(ORB_SLAM::KeyFrame* k1, ORB_SLAM::KeyFrame* k2, float* th);
        void GetIntensityGradient(cv::Mat im, float* g);
        void PixelNeighborSupport(depthHo*** H, int x, int y, std::vector<depthHo*>* support);
        void PixelNeighborNeighborSupport(depthHo*** H, int px, int py, std::vector<std::vector<depthHo*> >* support);
        void GetIntensityGradient_D(const cv::Mat& ImGrad, float a, float b, float c, int px, float* q);
        void GetPixelDepth(int px, int py, ORB_SLAM::KeyFrame* kf, float* p);
        //void GetPixelDepth(const cv::Mat& Im, const cv::Mat& R, const cv::Mat& T, ORB_SLAM::KeyFrame* kF, int u, float *p);
	bool ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val);
	void GetFusion(const std::vector<depthHo*>& best_compatible_ho, depthHo* hypothesis, float* min_sigma);
        void Equation14(depthHo*& dHjn, float& depthp, cv::Mat& xp, cv::Mat& rji, cv::Mat& tji, float* res);
};

#endif
