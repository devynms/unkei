//parameters for semi-dense probability mapping
int N = 7;
int covisN = 20;
int sigmaI = 20;
int lambdaG = 8;
int lambdaL = 80;
int lambdaTheta = 45;
int lambdaN = 3;

struct depth_ho {
    float depth; // pixel depth hypothesis
    float sigma; // pixel sigma depth hypothesis
};

for(size_t i=0; i<vpKFs.size(); i++) {
    
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
	
	
	//finding the Keyframes that best match pKF
	vector<ORB_SLAM::KeyFrame*> closestMatches = pKF->GetBestCovisibilityKeyFrames(covisN);
	set<MapPoint*> orbMP = pKF->GetMapPoints();
	vector<float> vDepths;
	vDepths.reserve(orbMP.size());
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2,3);
	float maxDepth;
	float minDepth;

	for(size_t point=0; point<orbMP.size(); i++){
		if(orbMP[point]){
			MapPoint* pMP = orbMP[point];
			cv::Mat x3Dw = pMP->GetWorldPos();
			float z = Rcw2.dot(x3Dw)+zcw;
			vDepths.push_back(z);
		}
	}
        //use mean and variance of depths to find min/max depths for inverse depth hypothesis
        // see stereo search constaints, Fig 3
        boost::variance::accumulator_set<double, stats<tag::variance> > acc;
	for_each(vDepths.begin(), vDepth.end(), bind<void>(ref(acc), _1));

	maxDepth = mean(acc) + 2*sqrt(variance(acc));
	minDepth = mean(acc) - 2*sqrt(variance(acc));

	cv::Mat pGradX, pGradY;
	cv::Mat original = pKF->GetImage();

	cv::Scharr(original, pGradX, CV_16S,1,0);
	cv::Scharr(original, pGradY, CV_16S,0,1);
	
	cv::Mat absPGradX, absPGradY;
	//cv::multiply(pGradX,pGradX,pGradX2);
	//cv::multiply(pGradY,pGradY,pGradY2);	
	//float pGrad = cv::sqrt(cv::multiply(pGradX,pGradX)+square(pGradY));
	cv::convertScaleAbs(pGardX,absPGradX);
	cv::convertScaleAbs(pGradY,absPGradY);
	cv::Mat pGrad;
       	cv::addWeighted(absPGradX,0.5,absPGradY,0.5,pGrad,0);
        
        struct depth_ho depthsArr[original.rows][original.cols];
        std::vector<depth_ho> depths;
	
        //find depth of all keypoints with epipolar search
	for(size_t j=0; j<closestMatches.size(); j++){
		KeyFrame* pKF2 = closestMatches[j];
		cv::Mat image = pKF2->GetImage();
		cv::Mat sigmaI, mean;
		cv::meanStdDev(image,mean,sigmaI)

                // need to compute in-plane rotation between the two keyFrames
                // compute from median rotation of corresponding ORB features between both frames
                // note: look at methods in ORBmatcher (SearchByXXX())
                float deltatheta;

		for(int x = 0; x < image.rows; x++){
			for(int y = 0; y < image.cols; y++){
                                depths[x][y] = NULL;

				if(pGrad.at<float>(x,y) < lambdaG){
					continue;
				}
				cv::Mat pj = image.at<Mat>(x,y);
			
                                // caculate the Pixel Gradient

				//calculate angle
				float valueX = absPGradX.at<float>(x,y);
				float valueY = absPGradY.at<float>(x,y);
				float thetap = cv::fastAtan2(valueX,valueY);
			
                                // compute fundamental matrix of the two keyFrames
				cv::Mat F12 = LocalMapping::ComputeF12(pKF,pKF2);
                                
                                // parameterization of the fundamental matrix (function of horizontal coordinate)
				//could probably use the opencv built in function instead
				float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
				float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
				float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
				//since a*x + b*y + c
				//x = uj
                                //y = vj
				//and we have y = (a/b)*x + (c/b)
				//actually let's try that
				float old_err = 1000.0;
				float best_ri = 0.0;
				float best_rg = 0.0;
				int best_pixel = 0;

                                //search along parameterization of epipolar line for best fit
				for(int uj = minDepth; uj < maxDepth; uj++){
					vj = (a/b)*uj + (c/b);
					cv::Mat pGrad2, pGradX2, pGradY2, absPGradX2, absPGradY2;
                                        
                                        // don't want to do all of this again
                                        // approximation of the pixel gradient in x and y directions
					cv::Scharr(image, pGradX2, CV_16S,1,0);
					cv::Scharr(image, pGradY2, CV_16S,0,1);
                                        // normalize the pixel gradient
					cv::convertScaleAbs(pGardX2,absPGradX2);
					cv::convertScaleAbs(pGradY2,absPGradY2);
       					cv::addWeighted(absPGradX2,0.5,absPGradY2,0.5,pGrad2,0);
                                        // compare gradient value to Lambda_G: must lie in high gradient area
					if(pGrad2.at<float>(uj,vj) < lambdaG){
						continue;
					}

                                        // gradient orientation-- don't want this to be perpendicular to EL 
					float valueX2 = absPGradX2.at<float>(uj,vj);
					float valueY2 = absPGradY2.at<float>(uj,vj);
					float thetaUj = cv::fastAtan2(valueX,valueY);
                                        
                                        // angle of epipolar line-- can be precomputed
					float thetaL = cv::fastAtan2(uj,vj);
					
                                        // intensity gradient along epipolar line
					if(std::abs(thetaUj - thetaL + M_PI) < lambdaTheta)
						continue;
					if(std::abs(thetaUj - thetaL - M_PI) < lambdaTheta)
						continue;
					//if(std::abs(thetaUj - (thetap + deltaTheta)) < lambdaTheta)
					//	continue;		
				
                                        // calculate similarity error
                                        // photometric error
                                        float ri = original.at<float>(x,y) - pj;
                                        // gradient modulo error
					float rg = pGrad - pGrad2;
                                        // this is equation (3) in the paper (using Scharr operator)
			     		float err = (ri*ri + (rg*rg)/0.23)/(sigmaI*sigmaI);
					if(err < old_err){
						best_pixel = uj;
						old_err = err;
						best_ri = ri;
						best_rg = rg;
					}
				}

				int uplusone = best_pixel + 1; 
				int vplusone = (a/b)*uplusone + (c/b);
				int uminone = best_pixel - 1; 
				int vminone = (a/b)*uminone + (c/b);
				
				float g = (image.at<float>(uplusone,vplusone) -image.at<float>(uminone,vminone))/2;  

				float q = (pGrad2.at<float>(uplusone,vplusone) - pGrad2.at<float>(uminone,vminone))/2;
			        
                                // README: should we be saving these?
				float ustar = best_pixel + (g*best_ri + (1/0.23)*q*best_rg)/(g*g + (1/0.23)*q*q);

				float sigma_ustar_sqr = (2*sigmaI*sigmaI)/(g*g + (1/0.23)*q*q);

				cv::Mat Rcw2 = pKF2->GetRotation();
				cv::Mat Rwc2 = Rcw2.t();
			        cv::Mat tcw2 = pKF2->GetTranslation();
				cv::Mat Tcw2(3,4,CV_32F);
				Rcw2.copyTo(Tcw2.colRange(0,3));
				tcw2.copyTo(Tcw2.col(3));


				const float fx2 = pKF2->fx;
			        const float cx2 = pKF2->cx;

				cv::Mat K2 = pKF2 -> GetCalibrationMatrix;
				
				cv::Mat xp = K2*image;
				int ujcx = best_pixel - cx2;
				int vjcx = (a/b)*ujcx + (c/b);
                                // eq (8)
                                // need to make a function for this
				float depthp = (Rcw2[2]*xp.at<float>(ujcx,vjcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ujcx][vjcx]+fx2*Tcw2[0]);
			
				int ustarcx = (ustar - cx2) + sqrt(sigma_ustar_sqr);
				int vstarcx = (a/b)*ustarcx + (c/b);
				float depthj_plus = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2[0]);

				int ustarcx = (ustar - cx2) - sqrt(sigma_ustar_sqr);
				int vstarcx = (a/b)*ustarcx + (c/b);
				float depthj_min = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2[0]);
			
                                float sigma_depthj = max(abs(depthj_min - depthp), abs(depthj_plus - depthp));
                               
                                // store the depth hypotheses for inverse depth hypothesis fusion
                                struct depth_ho dh;
                                dh.depth = depthp;
                                dh.sigma = sigma_depthj;
                                depthsArr[x][y] = dh;
                                depths.push_back(dh);
			}
		}
	}

        // INVERSE DEPTH HYPOTHESIS FUSION //

        // check compatibility of each hypothesis with each of the others
        // Fuse groups of compatible hypotheses which are at least lambda_N in size
        // chi test (compatibility) should be a function 
        std::vector<depth_ho> best_compatible_ho;
        for (int a=0; a < depths.size(); a++) {
            std::vector<depth_ho> compatible_ho;
            for (int b=0; b < depths.size(); b++) {
                
                float pa = depths[a].depth;
                float pb = depths[b].depth;
                
                float chi_test = (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma) + (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma);
                
                // test if the hypotheses a and b are compatible
                if (chi_test < 5.99) {
                    compatible_ho.push_back(depths[b]); 
                }
            }
            // test if hypothesis 'a' has the required support
            if (compatible.size()-1 >= lambda_N && compatible.size() > best_compatible_depth.size()) {
                compatible_ho.push_back(depths[a]); 
                best_compatible_ho = compatible_ho;
            }
        }

        // calculate the parameters of the normal distribution by fusing hypotheses
        float ndepthp = -1;
        float nsigma_sqr_p = -1;
        if (best_compatible_ho.size() >= lambda_N) {
            // make this a method?
            float pjsj =0; // numerator
            float rsj =0; // denominator
            for (int j = 0; j < best_compatible_ho.size(); j++) {
                pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
            }
            ndepthp = pjsj / rsj;
            nsigma_sqr_p = 1 / rsj;
        }

        // INTRA - KEYFRAME DEPTH CHECKING, SMOOTHING, AND GROWING //

       // run through all the pixels again
       depth_ho depthsArr2[original.rows][original.cols];
       for (int px = 1; px < (original.rows - 1); px++) {
            for (int py = 1; py < (original.cols - 1); py++) {
                depthsArr2[px][py] = NULL;
                if (depthsArr[px][py] == NULL) {
                    // check if this pixel is surrounded by at least two pixels that are compatible to each other.
                    std::vector<std::vector<depth_ho>> best_compatible_ho;
                    std::vector<float> min_sigmas;
                    for (int nx = px - 1; nx <= px + 1; nx++) {
                        for (int ny = py - 1; ny <= py + 1; ny++) {
                            if (nx == px && ny == py) continue;
                            std::vector<depth_ho> temp;
                            // go through the neighbors again!
                            float min_sigma = 100;
                            for (int nnx = px - 1; nnx <= px + 1; nnx++) {
                                for (int nny = py - 1; nny <= py + 1; nny++) {
                                    if ((nnx == nx && nny == ny) || (nnx == px && nny == py)) continue;
                                    float pa = depthsArr[nx][ny].depth; 
                                    float pb = depthsArr[nnx][nny].depth; 
                                    float chi_test = (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma) + (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma);
                                    if (chi_test < 5.99) {
                                        temp.push_back(depthsArr[nnx][nny]);
                                        if (depthsArr[nnx][nny].sigma * depthsArr[nnx][nny].sigma < min_sigma * min_sigma) {
                                            min_sigma = depthsArr[nnx][nny].sigma;
                                        }
                                    }
                                }
                            }
                            min_sigmas.push_back(min_sigma); 
                            best_compatible_ho.push_back(temp);
                        }
                    }
                    // potentially grow the reconstruction density
                    int max_support = 0;
                    int max_support_index = 0;
                    for (int c = 0; c < best_compatible_ho.size(); c++) {
                        if (best_compatible_ho[c].size() > max_support) {
                            max_support = best_compatible_ho[c].size();
                            max_support_index = c;
                        }
                    }
                    if (max_support >= 2) {
                        // assign this previous NULL depth_ho the average depth and min sigma of its compatible neighbors
                        float avg = 0;

                        float pjsj =0; // numerator
                        float rsj =0; // denominator
                        for (int j = 0; j < best_compatible_ho[max_support_index].size(); j++) {
                            pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                            rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                        }
                        avg = pjsj / rsj;

                        depthsArr2[px][py].depth = avg;
                        depthsArr2[px][py].sigma = min_sigma;
                    }
                } else {
                    // calculate the support of the pixel's  8 neighbors
                    int support_count = 0;
                    float chi_test, pa, pb;
                    float min_sigma = 100;
                    std::vector<depth_ho> best_compatible_ho;
                    for (int nx = px - 1; nx <= px + 1; nx++) {
                        for (int ny = py - 1; ny <= ny + 1; ny++) {
                            if (nx == px && ny == py) continue;
                            if (depthsArr[nx][ny] == NULL) continue;
                            pa = depthsArr[px][py].depth;
                            pb = depthsArr[nx][ny].depth;
                            chi_test = (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma) + (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma);
                            if (chi_test < 5.99) {
                                support_count++;
                                best_compatible_ho.push_back(depthsArr[nx][ny]);
                                if (depthsArr[nx][ny].sigma * depthsArr[nx][ny].sigma < min_sigma * min_sigma) {
                                    min_sigma = depthsArr[nx][ny].sigma;
                                }
                            }
                        }
                    }
                    if (support_count < 2) {
                        // average depth of the retained pixels
                        // set stdev to minimum of neighbor pixels
                        float avg = 0;

                        float pjsj =0; // numerator
                        float rsj =0; // denominator
                        for (int j = 0; j < best_compatible_ho.size(); j++) {
                            pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                            rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                        }
                        avg = pjsj / rsj;

                        depthsArr2[px][py].depth = avg;
                        depthsArr2[px][py].sigma = min_sigma;
                    } else {
                        depthsArr2[px][py] = depthsArr[px][py];
                    }
                }
            }
 	    for (int px = 1; px < (original.rows - 1); px++) {
		for (int py = 1; py < (original.cols - 1); py++) {
		
		}
	    }	
       }
<<<<<<< HEAD
=======
/*
	int support_count = 0;
	depth_ho depth_arr2[original.rows][original.cols];
	for(int px=1; px < original.rows - 1; px++){
		for(int py=1; py < original.cols -1; py++){
			depth_arr2[px][py] = NULL;
			float chi_test,pa,pb;
			float min_sigma = 100;
			//calculate the support for the pixels
			if(depths_arr[px][py] == NULL)
				continue;
			for(int nx=px-1; nx <= px+1; nx++){
				for(int ny = py-1; ny <= py+1; ny++){ 
				if(nx == ny)
					continue;
				if(depths_arr[nx][ny] == NULL) continue;
				pa = depth_arr[px][py].depth;
				pb = depth_arr[nx][ny].depth;
				chi_test = (pa-pb)*(pa-pb)/(depth_arr[px][py].sigma*depth_arr[px][py])+(pa-pb)*(pa-pb)/(depth_arr[nx][ny]*depth_ar[nx][ny]);
				if(chi_test < 5.99){
					support_count++;
					if(depth_arr[nx][ny].sigma * depth_arr.sigma < min_sigma *min_sigma){
						min_sigma = depth_arr[nx][ny].sigma;
					}
				} 
			}			
		}		
	}
*/
	
>>>>>>> fb3f00d49890a11a546906271234163fa0748111

       // INTER - KEYFRAME DEPTH CHECKING AND SMOOTHING

        // get neighbor keyframes:
        // set current keyframe of LocalMapping
        SearchInNeighbors() // (LocalMapping.h)
        getConnectedKeyFrames() // (KeyFrame.h) of LocalMapping::mCurrentKeyFrame and iterator through those
        std::vector<KeyFrame*> neighbors;
        
        // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
        // and propagate inverse depth
        for (int px = 0; px < original.rows; px++) {
            for (int py = 0; py < original.cols; py++) {
                if (depthsArr[px][py] == NULL) continue; 
                float depthp = depthsArr[px][py];
                int neighbor_keyframes_count = 0; // count of neighboring keyframes in which there is at least one compatible pixel
	        for(int j=0; j<neighbors.size(); j++){ // see comment above about getting neighbor keyframes
		    KeyFrame* pKFj = neighbors[j];
                    // calibration matrix
	            cv::Mat Kj = pKFj -> GetCalibrationMatrix;
		    cv::Mat Xp = Kj*image;
                    // rotation matrix
                    cv::Mat Rcwj = Tcw_.row(2).colRange(0,3);
                    Rcwj = Rcwj.t();
                    // translation matrix
                    cv::Mat tcwj = pKF2->GetTranslation();
                    cv::Mat Tcwj(3,4,CV_32F);
                    Rcwj.copyTo(Tcwj.colRange(0,3));
                    tcwj.copyTo(Tcwj.col(3));
                    
                    // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                    xj = (Kj * Rcwj * (1 / depthp) * xp) + Kj * Tcwj; 
                    float depthj = depthp / (Rcwj[2] * xp + depthp * Tcwj[2]); 

                    // find the (float) coordinates of the new point in keyframe_j
                    cv::Mat xyzj = xj * cv::Mat(px, py, depthp);
                    float xj = xyzj[0];  
                    float yj = xyzj[1]; 
                    
                    int compatible_points_count = 0; // count of compatible neighbor pixels
                    std::vector<cv::Point> compatible_points;
                    // look in 4-neighborhood pixels p_j,n around xj for compatible inverse depth
                    for (int nj = floor(xj); nj <= nj + 1; nj++) {
                        for (int ny = floor(yj); ny < ny + 1; ny++) {
                            if (depthsArr[nx][ny] == NULL) continue;
                            float depthjn = depthsArr[nx][ny].depth; 
                            float sigmajn = depthsArr[nx][ny].sigma; 
                            float test = (depthp - depthjn) * (depthp - depthjn) / (sigmajn * sigmajn);
                            if (test < 3.84) {
                                compatible_points_count++;
                                compatible_points.push_back(cv::Point(nx, ny));
                            }
                        }
                    }
                    
                    // at least one compatible pixel p_j,n must be found in at least lambda_N neighbor keyframes
                    if (compatible_points_count) {
                        neighbor_keyframes_count++;
                        float depthp_star = 1000;
                        float sum_depth = 0;
                        for (int p; p < compatible_points.size(); p++) {
                            float depthjn = depthsArr[compatible_points[p].x][compatible_points[p].y].depth;
                            float sigmajn = depthsArr[compatible_points[p].x][compatible_points[p].y].sigma;
                            // equation (14)
                            sum_depth += pow((depthjn - depthp * Rcwj[2] * xp * Tcwj[2]), 2) / (pow(depthjn, 4) * pow(sigmajn, 2)); 
                        } 
                    } 
                }
                // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
                if (neighbor_keyframes_count < lambda_N) {
                    depthsArr[px][py] = NULL;
                }
            }
        }
}
