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

#define N 7
#define sigmaI 20
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3

class ProbabilityMapping {

private:

	struct depthHo {
		float depth;
		float sigma;
	};

	void getImageGradient(cv::Mat& image, cv::Mat* grad);
    	void getGradientOrientation(cv::Mat& grad, float th);
        void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2);
	void getIntensityGradient(cv::Mat im, float* g);
        void getIntensityGradient_D(cv::Mat im, float* q);				
	void getPixelDepth(cv::Mat& R,cv::Mat& T, cv::Mat& K, cv::Mat& Im, int x, int y, float *p);
	bool chiTest(const depthHo ha, const depthHo hb, float* chi_val);
	void depthDistribution(float p, float sigma, depthHo* hypothesis);

public:

	/* * \brief void first_loop(KeyFrame kf, depthHo**, std::vector<depthHo>&): return results of epipolar search (depth hypotheses) */
	void firstLoop(KeyFrame kf, depthHo**, std::vector<depthHo>&);
        /* * \brief void stereo_search_constraints(): return min, max inverse depth */
	void stereoSearch_constraints();
	/* * \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
	void epipolarSearch();
	/* * \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist): 
	 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
	void inverseDepthHypothesisFusion(const vector<depthHo> H, depthHo* dist);
	/* * \brief void intraKeyframeDepthChecking(depthHo** H): intra-keyframe depth-checking, smoothing, and growing. 
	 * *         Modifies the double-array of depth hypotheses provided as input */
	void intraKeyframeDepthChecking(depthHo** H);
	/* * \brief void interKeyframeDepthChecking(depthHo** H): inter-keyframe depth-checking, smoothing, and growing. 
	 * *         Modifies the double-array of depth hypotheses provided as input */
	void interKeyframeDepthChecking(depthHo** H);

};

#endif
