
//parameters for semi-dense probability mapping
int N = 7;
int sigmaI = 20;
int lambdaG = 8;
int lambdaL = 80;
int lambdaTheta = 45;
int lambdaN = 3;

struct depth_ho {
	float depth;
	float sigma;
}

for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
	
	
	//finding the Keyframes that best match pKF
	vector<ORB_SLAM::KeyFrame*> closestMatches = pKF->GetBestCovisibilityKeyFrames(20);
	set<MapPoint*> orbMP = pKF->GetMapPoints();
	//defines vector for the depths
	vector<float> vDepths;
	vDepths.reserve(orbMP.size());
	//does something to relate the camera position to the depth of the point in the world (needs to be changed)
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
	//calculate the mean and the variance
	boost::variance::accumulator_set<double, stats<tag::variance> > acc;
	for_each(vDepths.begin(), vDepth.end(), bind<void>(ref(acc), _1));
	
	//determine min and max depth
	maxDepth = mean(acc) + 2*sqrt(variance(acc));
	minDepth = mean(acc) - 2*sqrt(variance(acc));

	//
	cv::Mat pGradX, pGradY;
	cv::Mat original = pKF->GetImage();

	cv::Scharr(original, pGradX, CV_16S,1,0);
	cv::Scharr(original, pGradY, CV_16S,0,1);
	
	cv::Mat absPGradX, absPGradY;
	//cv::multiply(pGradX,pGradX,pGradX2);
	//cv::multiply(pGradY,pGradY,pGradY2);	
	//float pGrad = cv::sqrt(cv::multiply(pGradX,pGradX)+square(pGradY));
	
	//normalizes the gradient
	cv::convertScaleAbs(pGardX,absPGradX);
	cv::convertScaleAbs(pGradY,absPGradY);
	cv::Mat pGrad;
       	cv::addWeighted(absPGradX,0.5,absPGradY,0.5,pGrad,0);
	
	vector<float> depth_hypotheses;
	vector<float> sigma_depth_hypotheses;
		
	vector<depth_ho> depths;
	depth_ho depths_arr[original.rows][original.cols];

	for(size_t j=0; j<closestMatches.size(); i++){
		KeyFrame* pKF2 = closestMatches[j];
		cv::Mat image = pKF2->GetImage();
		cv::Mat sigmaI, mean;
		cv::meanStdDev(image,mean,sigmaI)

		//NEED TO CALCULATE INPLANE ROTATION BETWEEN KEYFRAMES
		for(int x = 0; x < image.rows; x++){
			for(int y = 0; y < image.cols; y++){
				depths_arr[x][y] = NULL;
				if(pGrad.at<float>(x,y) < lambdaG){
					continue;
				}
				cv::Mat pj = image.at<Mat>(x,y);
				
				//calculate angle
				float valueX = absPGradX.at<float>(x,y);
				float valueY = absPGradY.at<float>(x,y);
				float thetap = cv::fastAtan2(valueX,valueY);
				
				cv::Mat F12 = LocalMapping::ComputeF12(pKF,pKF2);
				//could probably use the opencv built in function instead
				float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
				float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
				float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
				//since a*x + b*y + c
				//x = uj
				//and we have y = (a/b)*x + (c/b)
				//actually let's try that
				float old_err = 1000.0;
				float best_ri = 0.0;
				float best_rg = 0.0;
				int best_pixel = 0;

				//search the epipolar line for the best fit
				for(int uj = minDepth; uj < maxDepth; uj++){
					vj = (a/b)*uj + (c/b);
					cv::Mat pGrad2, pGradX2, pGradY2, absPGradX2, absPGradY2;
					
					//bad already done for the image, shouldn't be computed in the loop
					cv::Scharr(image, pGradX2, CV_16S,1,0);
					cv::Scharr(image, pGradY2, CV_16S,0,1);
					cv::convertScaleAbs(pGardX2,absPGradX2);
					cv::convertScaleAbs(pGradY2,absPGradY2);
       					cv::addWeighted(0.5,absPGradX2,0.5,absPGradY2,0,pGrad2);

					if(pGrad2.at<float>(uj,vj) < lambdaG){
						continue;
					}
					//calculate angle 
					float valueX2 = absPGradX2.at<float>(uj,vj);
					float valueY2 = absPGradY2.at<float>(uj,vj);
					float thetaUj = cv::fastAtan2(valueX,valueY);
					
					//calculate the angle of the epipolar line. we can precompute this
					float thetaL = cv::fastAtan2(uj,vj);
					
							
					if(std::abs(thetaUj - thetaL + M_PI) < lambdaTheta)
						continue;
					if(std::abs(thetaUj - thetaL - M_PI) < lambdaTheta)
						continue;
					//GET THE ORBMATCHER SEARCH BY PROJECTION AND SEARCH BY BAG OF WORDS TO GET US AN ORIENTATION
					if(std::abs(thetaUj - (thetap + deltaTheta)) < lambdaTheta)
						continue;		
					float ri = original.at<float>(x,y) - pj;
					float rg = pGrad - pGrad2;
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
				
				float ustar = best_pixel + (g*best_ri + (1/0.23)*q*best_rg)/(g*g + (1/0.23)*q*q);

				float sigma_ustar_sqr = (2*sigmaI*sigmaI)/(g*g +(1/.23)*q*q);

				cv::Mat Rcw2 = pKF2->GetRotation();
				cv::Mat Rwc2 = Rcw2.t();
			        cv::Mat tcw2 = pKF2->GetTranslation();
				cv::Mat Tcw2(3,4,CV_32F);
				Rcw2.copyTo(Tcw2.colRange(0,3));
				tcw2.copyTo(Tcw2.col(3));


				const float fx2 = pKF2->fx;
			        const float cx2 = pKF2->cx;

				cv::Mat K2 = pKF2 -> GetCalibrationMatrix;
				
				//NEEDS TO BE ABSTRACTED OUT INTO A FUNCTION
				cv::Mat xp = K2*image;
				int ujcx = best_pixel - cx2;
				int vjcx = (a/b)*ujcx + (c/b);
				//is depth j
				float depthp = (Rcw2[2]*xp.at<float>(ujcx,vjcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ujcx][vjcx]+fx2*Tcw2i[0]);
			
				//NEEDS TO BE FIXED

				  
				int ustarcx_min = ustar - cx2 - sqrt(sigma_ustar_sqr);
				int vstarcx_min = (a/b)*ustarcx + (c/b);
				
				float depthj_min = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2[0]);

				int ustarcx = ustar - cx2 + sqrt(sigma_ustar_sqr);
				int vstarcx = (a/b)*ustarcx + (c/b);
				
				float depthj_plus = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2[0]);
				float sigma_depthj = max(abs(depthj_plus, depthj_min);

				struct depthho dh;
				dh.depth = depthp
				dh.sigma = sigma_depthj;
				depths.push_back(dh);
				depths_arr[x][y] = dh;

			}
		}
	}
	// we want to check the compability of the different hypotheses O(N^2) we can improve runtime by memoizing
	// we store all of the results of the comparisons and we tell it only  to look at the future ones because the past ones are already
	// computed
	//chi-test should be a function
	vector<depth_ho> best_compatible_ho;
	for(size_t a=0; a < depths; a++){
		vector<depth_ho> compatible_depth;
		for(size_t b=0; b <  depths;b++){
			if(
			pa = depths[a].depth;
			pb = depths[b].depth;
			chi_sqr_test = (pa-pb)*(pa-pb)/(depths[a].sigma*depth[a].sigma)+(pa-pb)*(pa-pb)/(depths[b].sigma*depths[b].sigma);
			if(chi_sqr_test < 5.99){
				compatible_depth.push_back(depths[b]);
			}
		}
		if(compatible_depth.size() > lambdaN && compatible_depth.size() > best_compatible_ho.size()){
	 		best_compatible_ho = compatible_depth; 
			best_compatible_depth.push_back(depths[a]);
		}
	}

	//FUSING THE HYPOTHESES
	float norm_depthp = -1;
	float sigma_norm_depth = -1;
	if(best_compatible_ho.size() > lambdaN){
		float numerator = 0;	//for norm_depthp
		float denominator = 0;// for norm_depthp
		
		for(size_t j = 0;j < best_compatible.size(); j++){
			numerator += best_compatible_ho[j].depth/(best_compatible_ho[j].sigma*best_compatible_ho[j].sigma);
			denominator += 1/(best_compatible_ho[j].sigma*best_compatible_ho[j].sigma);	 
		}
		norm_depthp = numerator/denominator;
		sigma_norm_depth = 1/denominator;
	}

	//INTRA - KEYFRAME DEPTH CHECKING, SMOOTHING AND GROWING //
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

