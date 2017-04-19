#include "StopMotion.h"

#include "Grabcut/MinCut/energy.h"



#define DBACKG

#define UNKNOWN_FLOW_THRESH 1e9  


void StopMotion::run()
{
	Loggger<<"Let's start creating a stop motion world!\n.";
}

StopMotion::~StopMotion()
{

	if (m_pieceHomo)
	{
		delete m_pieceHomo;
		m_pieceHomo = NULL;
	}

	if (m_autoSelect)
	{
		delete m_autoSelect;
		m_autoSelect = NULL;
	}

}
void StopMotion::testChangeROI(IndexType pairId)
{
	SingleFrameSet& set_ = SingleFrameSet::get_instance();

	cv::Mat tMat = *set_[pairId].body;

	emit outPutAnImage(false,tMat);

}

void StopMotion::initBaseInfo()
{

}

void StopMotion::copyROIFromKey(IndexType fIdx, Mat& desIMg)
{
	desIMg.release();

	Mat* curF = SingleFrameSet::get_instance()[fIdx].body;

	//vector<Rect> roi = SingleFrameSet::get_instance()[fIdx].sltRect;
	vector<Rect> roi = m_sltRect;

	(*curF)(roi[0]).copyTo(desIMg);
}

void StopMotion::diffRoiBackG(IndexType fIdx,cv::Mat& resROI)
{
	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;

	Mat* curF = SingleFrameSet::get_instance()[fIdx].body;

	vector<Rect> roi = SingleFrameSet::get_instance()[fIdx].sltRect;

	if (roi[0].width > 0 && roi[0].height > 0)
	{
		labelBGforRoi(*bg, *curF, roi[0],resROI);

	}else
	{
		Loggger<<"Does not select a ROI firstly.\n";
	}
}

void StopMotion::makeDifferenceWithBG(IndexType fIdx, MatrixXXi& labels, MatrixXXi& smoothLabels)
{
	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;

	Mat* curF = SingleFrameSet::get_instance()[fIdx].body;

	vector<Rect> roi = SingleFrameSet::get_instance()[fIdx].sltRect;

	IndexType heigt = roi[0].height;

	IndexType width = roi[0].width;

	assert(heigt > 0 && width > 0);

	labels.resize(heigt,width);

	labels.setZero();

	Mat signImg;

	if (roi[0].width > 0 && roi[0].height > 0)
	{
		calDiffROIWithBG(*bg, *curF, roi[0],labels, signImg);

	}else
	{
		Loggger<<"Do not select a ROI firstly.\n";
	}


	Mat grayI,temp,tp2,tp3;

	cvtColor(signImg,grayI,CV_RGB2GRAY);

    Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(9,9));

	morphologyEx(grayI, tp3,CV_MOP_CLOSE,erodeStruct);

 	imshow("afterDifference", tp3);

	smoothLabels.resize(heigt,width);

	smoothLabels.setZero();

	for (IndexType hId = 0; hId < heigt; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType gVal = (int)tp3.at<uchar>(hId,wId);

			// do not fix this label of each pixel

			if (gVal == 156)
			{
				smoothLabels(hId,wId) = 0;

			}else if(gVal == 76)
			{
				smoothLabels(hId,wId) = 1;

			}else
			{
				Loggger<<"Some errors.\n";
			}

		}
	}

}

void StopMotion::calDiffROIWithBG(Mat& bg, Mat& curF, Rect& rect_, MatrixXXi& labels,Mat& resROI)
{
	IndexType wid = rect_.width;
	IndexType hig = rect_.height;

	IndexType xOff = rect_.x;
	IndexType yOff = rect_.y;

	//bg(rect_).copyTo(resROI);
	resROI.create(hig,wid, bg.type());

	for (IndexType i = 0; i < hig; ++ i)
	{
		for (IndexType j = 0; j < wid; ++ j)
		{
			Vec3b bgPix = bg.at<Vec3b>(i + yOff,j + xOff);

			Vec3b curPix = curF.at<Vec3b>(i + yOff,j + xOff);

			if ( isDifferent(bgPix,curPix))
			{
				labels(i,j) = 1;
				resROI.at<Vec3b>(i, j) = fgMask;

			}else
			{
				labels(i,j) = 0;
				resROI.at<Vec3b>(i, j) = bgMask;
			}

		}
	}
}

void StopMotion::labelBGforRoi(Mat& bg, Mat& curF, Rect& rect_, Mat& resROI)
{
	IndexType wid = rect_.width;
	IndexType hig = rect_.height;

	IndexType xOff = rect_.x;
	IndexType yOff = rect_.y;

	bg(rect_).copyTo(resROI);

	for (IndexType i = 0; i < hig; ++ i)
	{
		for (IndexType j = 0; j < wid; ++ j)
		{
			Vec3b bgPix = bg.at<Vec3b>(i + yOff,j + xOff);

			Vec3b curPix = curF.at<Vec3b>(i + yOff,j + xOff);

#ifdef DBACKG
			if ( isDifferent(bgPix,curPix))
			{
				//resROI.at<Vec3b>(i, j) = curPix;
				resROI.at<Vec3b>(i, j) = fgMask;

			}else
			{
				//resROI.at<Vec3b>(i, j) = bgMask;
				resROI.at<Vec3b>(i, j) = bgMask;

			}
#else
			resROI.at<Vec3b>(i, j) = curPix;
#endif 

		}
	}

}

bool StopMotion::isDifferent(Vec3b& bgPix, Vec3b& cufPix)
{
	IndexType thres = 15;

	//m_thresDiff = 1;

	if (abs(bgPix[0] - cufPix[0]) > m_thresDiff &&  abs(bgPix[1] - cufPix[1]) > m_thresDiff &&
		abs(bgPix[2] - cufPix[2]) > m_thresDiff)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void StopMotion::matchByROIOnly(Mat& srcImg, Mat& desImg, Mat& outImg, MatrixXXi& markLabels)
{
	MatrixXXi labels, smoothLabels;
	//makeDifferenceWithBG(m_showPairIdx[0], labels, smoothLabels);
	itaHandMarkers(srcImg,smoothLabels); //do nothing for labels  all of the label equal to 2;
	//graph cut method with interaction in order to segment the objects of the scene.
	MatrixXXi outBgFgMark;

	if (markLabels.cols() > 0)
	{
		setInteractiveStatue(initImageLabelWithInteractive(markLabels));
	}

	trainGMM(srcImg,markLabels); 

	minCut(srcImg,smoothLabels,outBgFgMark);

	//calculate the optical flow  and translate the foreground objects

	matchUsingFlowAndForeLabels(srcImg,desImg,outBgFgMark,outImg);

	//using feature points
	//Mat outFeaImg;
	//matchAndDeform(srcImg,desImg,outFeaImg);

}

void StopMotion::detectFeatures(Mat& oriImg, cv::vector<Point2f>& conners)
{
	Mat fraImage;

	cvtColor(oriImg,fraImage, CV_RGB2GRAY);

	int maxCorners = 10;
	double qualityLevel = 0.01;
	double minDis = 20.;
	cv::Mat mk;
	int blockSize = 3;
	bool useHarrisD = false;
	double k = 0.04;


	if (conners.size() > 0)
	{
		conners.clear();
	}

	cv::goodFeaturesToTrack(fraImage,conners,maxCorners,qualityLevel,minDis,mk,blockSize,useHarrisD,k);

	for (IndexType i = 0; i < conners.size(); ++i)
	{
		cv::circle(oriImg,conners[i],10,cv::Scalar(255.), -1);
	}

}

void StopMotion::matchWithFlow(Mat& srcImg, Mat& desImg, Mat& outImg)
{
	//find the object region

	//MatrixXXi labels, smoothLabels;
	//makeDifferenceWithBG(m_showPairIdx[0], labels, smoothLabels);

	//
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;
	
	//Mat motion2Color;

	cvtColor(srcImg, srcGray, CV_RGB2GRAY);
	cvtColor(desImg, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	//flowSover->calc(desGray,srcGray, outputFlow);

	//motionToColor(outputFlow,motion2Color);

	//imshow("Flow",motion2Color);

	vector<Point2f> srcMatch, desMatch;

	IndexType height = srcImg.rows;
	IndexType width = srcImg.cols;

	for (IndexType hId = 0; hId < height; ++ hId /*+= 5*/ )
	{
		for (IndexType wId = 0; wId < width; ++ wId /*+= 5*/)
		{
			// only for foreground pixels

//  			if ( 1 == smoothLabels(hId,wId))
//  			{
				Vec2f& elem = outputFlow.at<Vec2f>(hId,wId);

				Point2f curPos(hId,wId);
				Point2f nexPos(hId + elem[0], wId + elem[1]);

				if (nexPos.x <0 || nexPos.x > height || nexPos.y <0 || nexPos.y > width)
				{
					continue;
				}

				srcMatch.push_back(curPos);
				desMatch.push_back(nexPos);
			//}

		}
	}

	//Mat homo2 = findHomography(srcMatch,desMatch,CV_RANSAC);

	m_glbHomo = findHomography(srcMatch,desMatch,CV_RANSAC);

	outImg.create(srcImg.rows, srcImg.cols, srcImg.type() );

	//warpPerspective(srcImg,outImg,homo2,outImg.size() );

	warpPerspective(srcImg,outImg,m_glbHomo,outImg.size() );
	//checkBlackRegion(outImg);

//  	Mat temp;
//  
//  	cvtColor(outImg,temp,CV_RGB2BGR);
//  
//  	imshow("All-optical-Flow", temp);

}

void StopMotion::checkBlackRegion(Mat& warpedImg)
{
	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;

	Mat* curF = SingleFrameSet::get_instance()[m_showPairIdx[1] ].body;

	Mat bgROI;
	(*bg)(m_sltRect[0]).copyTo(bgROI);

	IndexType height = warpedImg.rows;
	IndexType width  = warpedImg.cols;

	Mat gayImg;
	cvtColor(warpedImg,gayImg,CV_RGB2GRAY); //0.2989 * R + 0.5870 * G + 0.1140 * B 

	imshow("grayImg",gayImg);

	Mat gayBgFg;
	gayBgFg.create(gayImg.rows,gayImg.cols,gayImg.type() );

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType val = gayImg.at<uchar>(hId,wId);

			if (val < 30) 
			{
				//warpedImg.at<Vec3b>(hId,wId) = bgROI.at<Vec3b>(hId,wId);
				//warpedImg.at<Vec3b>(hId,wId) = bgMask;

			}else
			{

			}
		}
	}

	//contourArea(gayImg,false);

	
	//opening processing for gray image for 
	
	

	Mat temp;
	Mat ker;
	IndexType sSize = 7;
	ker = Mat::ones(sSize,sSize,CV_32F)/(float)(sSize*sSize);
	filter2D(warpedImg,temp,-1,ker);
	
   Mat sTemp;
   cvtColor(temp,sTemp,CV_RGB2BGR);
   imshow("smooth5*5",sTemp);
}

void StopMotion::matchUsingFlowAndForeLabels(Mat& srcImg, Mat& desImg, 
											 MatrixXXi& foreLabels, Mat& outImg)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow, tempImg;

	cvtColor(srcImg, srcGray, CV_RGB2GRAY);
	cvtColor(desImg, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	vector<Point2f> srcMatch, desMatch;

	IndexType height = srcImg.rows;
	IndexType width = srcImg.cols;

	tempImg.create(srcImg.rows, srcImg.cols, srcImg.type());

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			
			if ( 1 == foreLabels(hId,wId)) // only for foreground pixels
			{

				Vec2f& elem = outputFlow.at<Vec2f>(hId,wId);

				Point2f curPos(hId,wId);
				Point2f nexPos(hId + elem[0], wId + elem[1]);

				if (nexPos.x <0 || nexPos.x > height || nexPos.y <0 || nexPos.y > width)
				{
					continue;
				}

				srcMatch.push_back(curPos);
				desMatch.push_back(nexPos);

				tempImg.at<Vec3b>(hId,wId) = srcImg.at<Vec3b>(hId,wId);

		   }else
			{
				tempImg.at<Vec3b>(hId,wId) = bgMask;
		   }

		}
	}

	Mat sTempImg;
	cvtColor(tempImg,sTempImg,CV_RGB2BGR);
	imshow("foregroundImg",sTempImg);

	Mat homo2 = findHomography(srcMatch,desMatch,CV_RANSAC);

 	outImg.create(srcImg.rows, srcImg.cols, srcImg.type() );
 
 	warpPerspective(tempImg,outImg,homo2, outImg.size() );

	Mat temp;

	cvtColor(outImg,temp,CV_RGB2BGR);

	imshow("Partial-optical-Flow", temp);

}

void StopMotion::matchFlowAndDeform(Mat& srcImg, Mat& desImg, Mat& outImg)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;

	cvtColor(srcImg, srcGray, CV_RGB2GRAY);
	cvtColor(desImg, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	vector<Point2f> srcMatch, desMatch;

	IndexType height = srcImg.rows;
	IndexType width = srcImg.cols;

	Mat resMap(srcImg.size(), CV_32FC2);

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			Point2f& elem = outputFlow.at<Point2f>(hId,wId);

			resMap.at<Point2f>(hId,wId) = Point2f(hId + elem.x, wId + elem.y);

		}
	}

	remap(srcImg, outImg,resMap,cv::Mat(), CV_INTER_LINEAR);//by means of 

	Mat temp;

	cvtColor(outImg,temp,CV_RGB2BGR);

	imshow("DF-optical-Flow", temp);

}
void StopMotion::matchAndDeform(Mat& srcImg, Mat& desImg, Mat& outImg)
{

	string detectorType = "SIFT";  
	string descriptorType = "SIFT";  
	string matcherType = "FlannBased";

	Ptr<FeatureDetector> featureDetector;  
	Ptr<DescriptorExtractor> descriptorExtractor;  
	Ptr<DescriptorMatcher> descriptorMatcher; 

	if( !createDetectorDescriptorMatcher( detectorType, descriptorType,
		                                  matcherType, featureDetector, 
										  descriptorExtractor,descriptorMatcher ) )  
	{  
		Loggger<<"Creat Detector Descriptor Matcher False!"<<endl;  
		return ;  
	} 

	//get the srcImg keypoints

	vector<KeyPoint> queryKeyPs, refQueryKP;
	vector<KeyPoint> trainKeyPs, refTrainKP;
	Mat queryDesc;
	Mat trainDesc;

	queryKeyPs.clear();
	trainKeyPs.clear();
	queryDesc.setTo(0);
	trainDesc.setTo(0);

	featureDetector->detect(srcImg, queryKeyPs);

	refineKeyps(srcImg,queryKeyPs,refQueryKP);

	if (refQueryKP.size() > 3/*queryKeyPs.size() > 0*/)
	{
	   //descriptorExtractor->compute(srcImg,queryKeyPs,queryDesc);

	   descriptorExtractor->compute(srcImg,refQueryKP,queryDesc);

//  	   for (IndexType i = 0; i < refQueryKP.size(); ++i)
//  	   {
//  		   cv::circle(srcImg, refQueryKP[i].pt, 5, cv::Scalar(255.), -1);
//  	   }
	}


	featureDetector->detect(desImg,trainKeyPs);

	refineKeyps(desImg,trainKeyPs,refTrainKP);

	if (refTrainKP.size() > 3/*trainKeyPs.size() > 0*/)
	{
    	//descriptorExtractor->compute(desImg,trainKeyPs,trainDesc);

		descriptorExtractor->compute(desImg,refTrainKP,trainDesc);

// 		for (IndexType i = 0; i < refTrainKP.size(); ++i)
// 		{
// 			cv::circle(desImg, refTrainKP[i].pt, 5, cv::Scalar(255.), -1);
// 		}
		
	}else
	{
		Loggger<<"The feature points are very small.\n";
		return;
	}


	Mat sTemp,tTemp;

	cvtColor(srcImg,sTemp,CV_RGB2BGR);
	cvtColor(desImg,tTemp,CV_RGB2BGR);

// 	imshow("Sr-feature",sTemp);
// 	imshow("Tg-feature",tTemp);

	Mat homo;

	vector<DMatch> matchPs;

	//bool isFound = matchingDescriptor(queryKeyPs,trainKeyPs,queryDesc,trainDesc,descriptorMatcher,homo, matchPs);

	bool isFound = matchingDescriptor(refQueryKP,refTrainKP,queryDesc,trainDesc,descriptorMatcher,homo, matchPs);

	vector<Point2f> srcMatch, desMatch;

	Loggger<<"Matches size = "<<matchPs.size()<<endl;

	for (IndexType i = 0; i < matchPs.size(); ++i)
	{
		//Point2f pt1 = queryKeyPs[matchPs[i].queryIdx].pt;
		//Point2f pt2 = trainKeyPs[matchPs[i].trainIdx].pt;

		Point2f pt1 = refQueryKP[matchPs[i].queryIdx].pt;
		Point2f pt2 = refTrainKP[matchPs[i].trainIdx].pt;

		srcMatch.push_back(pt1);
		desMatch.push_back(pt2);

		//cv::circle(desImg, trainKeyPs[matchPs[i].trainIdx].pt, 5, cv::Scalar(255.), -1);
	}

	Mat drawIMg, drawTemp;

	drawMatches(srcImg, refQueryKP,desImg,refTrainKP, matchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("matches", drawTemp);

	if (srcMatch.size() > 3 && desMatch.size() > 3)
	{
		Mat homo2 = findHomography(srcMatch,desMatch,CV_RANSAC);

		outImg.create(srcImg.rows, srcImg.cols, srcImg.type() );

		warpPerspective(srcImg,outImg,homo2,outImg.size() );

		Mat temp;

		cvtColor(outImg,temp,CV_RGB2BGR);

		imshow("DEF-RANSAC", temp);

	}else
	{
		Loggger<<"The size of the matching points is small!.\n";
	}

}


bool StopMotion::createDetectorDescriptorMatcher(const string& detectorType, 
												 const string& descriptorType, 
												 const string& matcherType, 
												 Ptr<FeatureDetector>& featureDetector,
												 Ptr<DescriptorExtractor>& descriptorExtractor,
												 Ptr<DescriptorMatcher>& descriptorMatcher )
{
	cout << "< Creating feature detector, descriptor extractor and descriptor matcher ..." << endl;  
	if (detectorType=="SIFT"||detectorType=="SURF")  
		initModule_nonfree();  

	featureDetector = FeatureDetector::create( detectorType );  
	descriptorExtractor = DescriptorExtractor::create( descriptorType );  
	descriptorMatcher = DescriptorMatcher::create( matcherType );  
	cout << ">" << endl;  
	bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );  
	if( !isCreated )  
		cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;  
	return isCreated; 

}

bool StopMotion::matchingDescriptor(const vector<KeyPoint>& queryKeyPoints, 
									const vector<KeyPoint>& trainKeyPoints, 
									const Mat& queryDescriptors,
									const Mat& trainDescriptors, 
									Ptr<DescriptorMatcher>& descriptorMatcher, 
									Mat& homo,
									vector<DMatch>& matchPs,
									bool enableRatioTest /* = true */)
{
	vector<vector<DMatch>> m_knnMatches;  

	if (enableRatioTest)  
	{  
		cout<<"KNN Matching"<<endl;  
		const float minRatio = 1.f / 1.5f;  
		descriptorMatcher->knnMatch(queryDescriptors,trainDescriptors,m_knnMatches,2);  
		for (size_t i=0; i<m_knnMatches.size(); i++)  
		{  
			const cv::DMatch& bestMatch = m_knnMatches[i][0];  
			const cv::DMatch& betterMatch = m_knnMatches[i][1];  
			float distanceRatio = bestMatch.distance / betterMatch.distance;  
			if (distanceRatio < minRatio)  
			{  
				matchPs.push_back(bestMatch);  
			}  
		}  

	}  
	else  
	{  
		cout<<"Cross-Check"<<endl;  
		Ptr<cv::DescriptorMatcher> BFMatcher(new cv::BFMatcher(cv::NORM_HAMMING, true));  
		BFMatcher->match(queryDescriptors,trainDescriptors, matchPs );  
	}  

	float homographyReprojectionThreshold = 1.0;  
	//bool homographyFound = refineMatchesWithHomography(  
		//queryKeyPoints,trainKeyPoints,homographyReprojectionThreshold,matchPs,homo);  

// 	if (!homographyFound) 
// 	{
// 		return false; 
// 
// 	}else  
// 	{  

		return true;
	//}
}

bool StopMotion::refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints, 
											 const std::vector<cv::KeyPoint>& trainKeypoints, 
											 float reprojectionThreshold, 
											 std::vector<cv::DMatch>& matches, 
											 cv::Mat& homography )
{
	const int minNumberMatchesAllowed = 4;    
	if (matches.size() < minNumberMatchesAllowed)    
		return false;    
	// Prepare data for cv::findHomography    
	std::vector<cv::Point2f> queryPoints(matches.size());    
	std::vector<cv::Point2f> trainPoints(matches.size());    
	for (size_t i = 0; i < matches.size(); i++)    
	{    
		queryPoints[i] = queryKeypoints[matches[i].queryIdx].pt;    
		trainPoints[i] = trainKeypoints[matches[i].trainIdx].pt;    
	}    
	// Find homography matrix and get inliers mask    
	std::vector<unsigned char> inliersMask(matches.size());    
	homography = cv::findHomography(queryPoints,     
		trainPoints,     
		CV_FM_RANSAC,     
		reprojectionThreshold,     
		inliersMask);    
	std::vector<cv::DMatch> inliers;    
	for (size_t i=0; i<inliersMask.size(); i++)    
	{    
		if (inliersMask[i])    
			inliers.push_back(matches[i]);    
	}    
	matches.swap(inliers);  

	return matches.size() > minNumberMatchesAllowed;   
}

void StopMotion::refineKeyps(Mat& cufImg, vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs)
{
	assert(oriKeyPs.size() > 0 && m_sltRect.size() > 0);

	if (m_sltRect[0].width == 0 || m_sltRect[0].height == 0)
	{
		Loggger<<"Fail to initialize the select regions.\n";
		return;
	}

	Mat resROI;
	Mat bg;

	bg = * (SingleFrameSet::get_instance().getBackGround().body);

	bg(m_sltRect[0]).copyTo(resROI);

	for (auto vIt = oriKeyPs.begin(); vIt != oriKeyPs.end(); ++ vIt )
	{
		KeyPoint kPs = * vIt;

		Point2f keyPosk = kPs.pt;

		Vec3b bgPix = resROI.at<Vec3b>(keyPosk.y,keyPosk.x);

		Vec3b curPix = cufImg.at<Vec3b>(keyPosk.y,keyPosk.x);

		if ( isDifferent(bgPix,curPix))
		{
			outKeyPs.push_back(kPs);

		}else
		{
			resROI.at<Vec3b>(keyPosk.y,keyPosk.x) = bgMask;
	        cv::circle(resROI, keyPosk, 5, cv::Scalar(255.), -1);
		}
	}



// 	Mat temp;
// 
// 	cvtColor(resROI,temp,CV_RGB2BGR);
// 
// 	imshow("Outliers", temp);


}


void StopMotion::initImageLabelWithHandDetect(Mat& inputImg)
{
	IndexType hight = inputImg.rows;
	IndexType width = inputImg.cols;

	m_initLabel.resize(hight,width);

	m_initLabel.Zero(hight,width);

	for (int y = 0; y < hight; y++) 
	{
		for (int x = 0; x <width; x++)
		{
			Vec3b clr = inputImg.at<cv::Vec3b>(y,x);

			bool isH = isHandPixel(clr);
			if (isH)
			{
				m_initLabel(y,x) = 1;
			}
		}
	}

}
bool StopMotion::trainGMM(Mat& inputImg, MatrixXXi& initalLabels)
{

  	IndexType m_nWidth = inputImg.cols;
  	IndexType m_nHeight = inputImg.rows;


	m_imgHight = inputImg.rows;
	m_imgWidth = inputImg.cols;

  
//   	const int cov_mat_type = cv::EM::COV_MAT_DIAGONAL;
//   	cv::TermCriteria term(CV_TERMCRIT_ITER |CV_TERMCRIT_EPS, 150, 1e-6);
//   	cv::EM gmm(5,cov_mat_type,term),gmm_b(5,cov_mat_type,term);
//   	cv::Mat labels, posterior, logLIkelihood;
//   
//	cv::Mat labels_b, posterior_b, logLIkelihood_b;
//
//   	cv::Mat patterns(0,0,CV_64F);
//    cv::Mat patterns_b(0,0,CV_64F);
//
//   	IndexType hSize = 0;
//   
//   	 	for (int y = 0; y <  m_nHeight; y++) 
//   	 	{
//   	 		for (int x = 0; x < m_nWidth; x++)
//   	 		{
//   
//   				if (initalLabels(y,x) == 1 )
//   				{
//   					cv::Vec3b cpv = inputImg.at<Vec3b>(y,x);
//   
//   					cv::Mat smp = (cv::Mat_<float>(1,3)<<cpv[0],cpv[1],cpv[2]);
//   
//   					patterns.push_back(smp);
//   
//   					hSize++;
//   
//                       //Loggger<<cpv<<endl;
//   				}
//
//				if (initalLabels(y,x) == 0)
//				{
//					cv::Vec3b cpv = inputImg.at<Vec3b>(y,x);
//
//					cv::Mat smp = (cv::Mat_<float>(1,3)<<cpv[0],cpv[1],cpv[2]);
//
//					patterns_b.push_back(smp);
//				}
//   
//               }
//           }
//   
//   	printf("The size of the hands pixel = %d.\n",hSize);
//   
//   	gmm.train(patterns,logLIkelihood,labels,posterior);
//   
//	gmm_b.train(patterns_b,logLIkelihood_b,labels_b,posterior_b);
//
//
//    //cv:Mat means = gmm.get<cv::Mat>("means");
//   
//    //vector<cv::Mat> covs = gmm.get<vector<cv::Mat> >("covs");
//   
//   	//Loggger<<means<<endl;
//   
//   	//Loggger<<covs[0]<<endl;
//   
//   	Loggger<<patterns.row(0)<<endl;
//    Loggger<<patterns_b.row(0)<<endl;
// 
//
//	Loggger<<logLIkelihood.row(0)<<endl;
//	Loggger<<posterior.row(0)<<endl;
//
//	Loggger<<logLIkelihood_b.row(0)<<endl;
//	Loggger<<posterior_b.row(0)<<endl;
//
//
////  	MatrixXX cov;
////  	MatrixXX meanEig;
////  	MatrixXX tSMP;
////  
////  	cv2eigen(covs[0],cov);
////  	cv2eigen(means, meanEig);
////  	cv2eigen(patterns.row(0),tSMP);
////  
////  	Loggger<<calculateProb(3,meanEig,cov,tSMP);
// 
// // 	cov.resize(covs[0].rows, covs[0].cols);
// // 		<< covs[0];
// 
// 	//ScalarType prob = 0.;
// 
// 	Mat prob1(1,5,CV_64F), prob2(1,5,CV_64F),prob3(1,5,CV_64F);
// 
// 	cv::Mat test = (cv::Mat_<float>(1,3)<<0,11,15);
//
//	Loggger<<gmm.get<cv::Mat>("weights")<<endl;
//	Loggger<<gmm_b.get<cv::Mat>("weights")<<endl;
//
// 	Loggger<< gmm.predict(patterns.row(0),prob1)<<endl;
// 	Loggger<< gmm.predict(patterns_b.row(0),prob2)<<endl;
//    Loggger<< gmm.predict(test,prob3)<<endl;
//
//	Loggger<<prob1<<endl;
//	Loggger<<prob2<<endl;
//	Loggger<<prob3<<endl;
//
//
//	Loggger<< gmm_b.predict(patterns.row(0),prob1)<<endl;
//	Loggger<< gmm_b.predict(patterns_b.row(0),prob2)<<endl;
//	Loggger<< gmm_b.predict(test,prob3)<<endl;
//
//	Loggger<<prob1<<endl;
//	Loggger<<prob2<<endl;
//	Loggger<<prob3<<endl;



// codes are from Xiaoguang

// 	int m_nWidth = inputImg.cols;
// 	int m_nHeight = inputImg.rows;
 
 	ofstream filefg,filebg;
 
 	filefg.open("datafg.bin", ios_base::out | ios_base::binary);
 	filebg.open("databg.bin", ios_base::out | ios_base::binary);
 
 	int nFrameSize = 3;

 
	int buffersize = m_imgWidth * m_nHeight * 3 * sizeof(float);
	float *fg = new float [m_imgWidth * m_nHeight * 3 ];
	float *bg = new float [m_imgWidth * m_nHeight * 3];

 	int fg_size = 0;
 	int bg_size = 0;
 	float Ez2 = 0;
 	float Ez[3];
 
 	memset(&Ez, 0, 3*sizeof(float));
 
	//0-bg-nonhand
	//1-fg-hand

 	for (int y = 0; y < m_nHeight; y++) 
 	{
 		for (int x = 0; x < m_imgWidth; x++)
 		{
 			//Vec3b temp = inputImg.at<Vec3b>(y,x);
 
			Ez2 += inputImg.data[y*m_imgWidth*3 + x*3] * inputImg.data[y*m_imgWidth*3 + x*3] +
				inputImg.data[y*m_imgWidth*3 + x*3 + 1] * inputImg.data[y*m_imgWidth*3 + x*3 + 1] +
				inputImg.data[y*m_imgWidth*3 + x*3 + 2] * inputImg.data[y*m_imgWidth*3 + x*3 + 2];

			Ez[0] += inputImg.data[y*m_imgWidth*3 + x*3];
			Ez[1] += inputImg.data[y*m_imgWidth*3 + x*3 + 1];
			Ez[2] += inputImg.data[y*m_imgWidth*3 + x*3 + 2];

 			if (initalLabels(y,x) == 0) 
 			{
				bg[bg_size*3] = inputImg.data[y*m_imgWidth*3 + x*3];
				bg[bg_size*3 + 1] = inputImg.data[y*m_nWidth*3 + x*3 + 1];
				bg[bg_size*3 + 2] = inputImg.data[y*m_nWidth*3 + x*3 + 2];

 				++ bg_size;

 				//m_Alpha[y*m_nWidth + x] = 1;
 			}
 
 			if(initalLabels(y,x) == 1) 
 			{
				fg[fg_size*3] = inputImg.data[y*m_nWidth*3 + x*3];
				fg[fg_size*3 + 1] = inputImg.data[y*m_nWidth*3 + x*3 + 1];
				fg[fg_size*3 + 2] = inputImg.data[y*m_nWidth*3 + x*3 + 2];

 				++ fg_size;
 				//m_Alpha[y*m_nWidth + x] = 0;
 			}
 		}
 	}
 
 	Ez2 = Ez2 / (m_nWidth * m_nHeight);
 	Ez2 -= (Ez[0]*Ez[0] + Ez[1]*Ez[1] + Ez[2]*Ez[2]) / (m_nWidth * m_nHeight) / (m_nWidth * m_nHeight);
 
    m_beta = 0.25 / Ez2;
 
 	filefg.write((const char*)(&fg_size), sizeof(int));
 	filefg.write((const char*)(&nFrameSize), sizeof(int));
 	filebg.write((const char*)(&bg_size), sizeof(int));
 	filebg.write((const char*)(&nFrameSize), sizeof(int));
 
	filefg.write((const char*)fg, fg_size*3*sizeof(float));
	filebg.write((const char*)bg, bg_size*3*sizeof(float));

 	filefg.close();
 	filebg.close();
 
 	delete [] fg;
 	delete [] bg;
 
 	GMM("datafg.bin", m_meansFG, m_varFG, m_logwFG);
 	GMM("databg.bin", m_meansBG, m_varBG, m_logwBG);

	return true;
}

void StopMotion::updateGMM(Mat& inputImg, MatrixXXi& lastLabels)
{
	int bgsize = 0;
	int fgsize = 0;

	memset(m_meansFG, 0, sizeof(float)*15);
	memset(m_varFG, 0, sizeof(float)*15);
	memset(m_logwFG, 0, sizeof(float)*N_GAUSS);

	memset(m_meansBG, 0, sizeof(float)*15);
	memset(m_varBG, 0, sizeof(float)*15);
	memset(m_logwBG, 0, sizeof(float)*N_GAUSS);

	ScalarType curRGB[3];

	for (int y = 0; y < m_imgHight; y++) 
	{
		for (int x = 0; x < m_imgWidth; x++) 
		{
			curRGB[0] = inputImg.data[y*m_imgWidth*3 + x*3];
			curRGB[1] = inputImg.data[y*m_imgWidth*3 + x*3 + 1];
			curRGB[2] = inputImg.data[y*m_imgWidth*3 + x*3 + 2];

            IndexType classId = getLabel(lastLabels(y,x), curRGB);

			if (lastLabels(y,x) == 1 )
			{		
				m_meansFG[classId][0] += curRGB[0];
				m_meansFG[classId][1] += curRGB[1];
				m_meansFG[classId][2] += curRGB[2];

				m_varFG[classId][0] += curRGB[0]*curRGB[0];
				m_varFG[classId][1] += curRGB[1]*curRGB[1];
				m_varFG[classId][2] += curRGB[2]*curRGB[2];

				m_logwFG[classId] += 1;

				fgsize++;
			}

			//else 
			if (lastLabels(y,x) == 0)
			{
				m_meansBG[classId][0] += curRGB[0];
				m_meansBG[classId][1] += curRGB[1];
				m_meansBG[classId][2] += curRGB[2];

				m_varBG[classId][0] += curRGB[0]*curRGB[0];
				m_varBG[classId][1] += curRGB[1]*curRGB[1];
				m_varBG[classId][2] += curRGB[2]*curRGB[2];

				m_logwBG[classId] += 1;

				bgsize++;
			}	

		}
	}

	for (int j = 0; j < N_GAUSS; j++) 
	{
		for (int i = 0; i < 3; i++)
		{
			m_meansBG[j][i] /= m_logwBG[j];
			m_meansFG[j][i] /= m_logwFG[j];
			m_varBG[j][i] /= m_logwBG[j];
			m_varFG[j][i] /= m_logwFG[j];
			m_varBG[j][i] -= m_meansBG[j][i] * m_meansBG[j][i];
			m_varFG[j][i] -= m_meansFG[j][i] * m_meansFG[j][i];
		}
		m_logwBG[j] /= bgsize;
		m_logwFG[j] /= fgsize;
		m_logwBG[j] = log( m_logwBG[j] );
		m_logwFG[j] = log( m_logwFG[j] );
	}

}

void StopMotion::minCut(Mat& inputImg, MatrixXXi& lastLabels, MatrixXXi& outBgFgMark)
{
	//segment the leftImg image.

	IndexType hight = inputImg.rows;
	IndexType width = inputImg.cols;

	assert(hight>0 && width >0);

	IndexType vSize = hight * width;

	Energy::Var *varAlpha;

	Energy* totEgy = new Energy();

	varAlpha = new Energy::Var[vSize];

	for (IndexType vIdx = 0; vIdx < vSize; ++ vIdx)
	{
		varAlpha[vIdx] = totEgy->add_variable(); 
	}

	ScalarType dis2Prob[2];
	ScalarType rgbColor[3], neigRGBCoor[3];

	//0-bg-non-hand - small
	//1-fg-hand - change large

	//inner region
	for (int y = 0; y < hight ; y++) 
	{
		for (int x = 0; x < width ; x++)
		{
			IndexType vIdx = y* width + x;

			rgbColor[0] = inputImg.data[y*width*3 + x*3];
			rgbColor[1] = inputImg.data[y*width*3 + x*3 + 1];
			rgbColor[2] = inputImg.data[y*width*3 + x*3 + 2];

			//data item

			if (m_isWithInteractive)
			{
				if (m_initLabel(y,x) == GC_FGD)//interactive to assign the labels
				{

					totEgy->add_term1(varAlpha[vIdx], 100, 0);

				}else if (m_initLabel(y,x) == GC_BGD)
				{

					totEgy->add_term1(varAlpha[vIdx], 0, 100);

				}else
				{

					dis2Prob[0] = m_gWeight * minProb(0,rgbColor);

					dis2Prob[1] = m_gWeight * minProb(1,rgbColor);

// 					dis2Prob[0] = m_gWeight * maxProb(0,rgbColor);
// 
// 					dis2Prob[1] = m_gWeight * maxProb(1,rgbColor);

	// 				ScalarType p0 = dis2Prob[1]/totP;
	// 
	// 				ScalarType p1 = dis2Prob[0]/totP;

					totEgy->add_term1(varAlpha[vIdx], dis2Prob[0],dis2Prob[1]);

				}

			}else//isInteractive ?
			{
				dis2Prob[0] = m_gWeight * minProb(0,rgbColor);

				dis2Prob[1] = m_gWeight * minProb(1,rgbColor);

				totEgy->add_term1(varAlpha[vIdx], dis2Prob[0],dis2Prob[1]);

			}

			//smooth item
			for (IndexType yStep = -1; yStep <= 1; ++ yStep)
			{
				for (IndexType xStep = -1; xStep <= 1; ++ xStep)
				{
					IndexType curX = x + xStep;
					IndexType curY = y + yStep;

					if (curX >= width || curX < 0 || curY >= hight || curY < 0)
					{
						continue;
					}

					if (abs(curX - x) == 1 && abs(curY - y) == 1)
					{
						continue;
					}

					if ( xStep ==0 && yStep ==0)
					{
						continue;
					}

					IndexType nVarIdx = curY * width + curX;

					neigRGBCoor[0] = inputImg.data[curY*width*3 + curX*3];
					neigRGBCoor[1] = inputImg.data[curY*width*3 + curX*3 + 1];
					neigRGBCoor[2] = inputImg.data[curY*width*3 + curX*3 + 2];

					IndexType aLabel = lastLabels(y,x);

					IndexType bLabel = lastLabels(curY,curX);// neighborhood

					ScalarType sValue = smoothItemOri(aLabel,bLabel,rgbColor,neigRGBCoor);

					totEgy->add_term2(varAlpha[vIdx], varAlpha[nVarIdx], 0, sValue, sValue,0);

				}
			}
		}
	}

	//fclose(in_label_prob);

	Energy::TotalValue eMin = totEgy->minimize();

	outBgFgMark.setZero(hight,width);

	Mat visuBgFg(hight,width,CV_8UC3);

	for (int y = 0; y < hight; y++) 
	{
		for (int x = 0; x < width; x++)
		{
			IndexType vIdx = y* width + x;

			outBgFgMark(y,x) = totEgy->get_var(varAlpha[vIdx]);

			if (outBgFgMark(y,x) == 1)
			{
				visuBgFg.at<cv::Vec3b>(y,x) = fgMask;
			}else
			{
				visuBgFg.at<cv::Vec3b>(y,x) = bgMask;
			}

		}
	}


	// iterate the labels

	//m_initLabel = outBgFgMark; // after One iteration, the m_initLabel now obtains a full rank value.

	//

	Mat tImg;

	//tImg.release();

	cvtColor(visuBgFg,tImg,CV_RGB2BGR);

	if (m_isOkDeformation)
	{
	  imshow("MC_Deform",tImg);
	}else
	{
      imshow("MC_HandRemoval",tImg);
	}


	delete totEgy;

	delete [] varAlpha;



}

void StopMotion::imageGenerate(Mat& leftImg, Mat& rightImg, 
							   MatrixXXi& BgFgMark, Mat& outPutImg)
{
	//based on the rightImg

	IndexType hight = leftImg.rows;
	IndexType width = leftImg.cols;

	outPutImg.release();
	outPutImg.create(hight,width,CV_8UC3);
	outPutImg.zeros(hight,width,CV_8UC3);
	//rightImg.copyTo(outPutImg);

// original version
	Mat maskImg;
	maskImg.create(hight,width,CV_8UC3);

 	for (int y = 0; y < hight; y++) 
 	{
 		for (int x = 0; x < width; x++)
 		{
 
  			if (BgFgMark(y,x) == 1)
  			{
 				outPutImg.at<cv::Vec3b>(y,x) = leftImg.at<cv::Vec3b>(y,x); // large change
				maskImg.at<cv::Vec3b>(y,x) = fgMask;

  			}else
  			{
 				outPutImg.at<cv::Vec3b>(y,x) = rightImg.at<cv::Vec3b>(y,x); //small change-keep itself.
				maskImg.at<cv::Vec3b>(y,x) = bgMask;
  			}
 		}
 	}


	//using background pixels for hand

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	//Mat* curF = SingleFrameSet::get_instance()[m_showPairIdx[1] ].body;

	vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;
// 
// 	Mat bgROI;
// 	(*bg)(roi[0]).copyTo(bgROI);

	if (roi.size()<=0)
	{
		Loggger<<"Rectangle is error!.\n";
		return;
	}

	outPutImg.copyTo((bg)(roi[0]));


	Mat tImg, tmaskImg;

	cvtColor(bg,tImg,CV_RGB2BGR);
	cvtColor(maskImg,tmaskImg,CV_RGB2BGR);

	// save the results
	char comName[1024];
	char maskName[1024];


	sprintf(comName,".\\resCombine\\1012-combine-%.2d.jpg",m_curPairIdx);

	//sprintf(maskName,".\\resCombine\\0913-mask-%.2d.jpg",m_curPairIdx);

	//imwrite(comName, tImg);
	imwrite(comName, tImg);

	//imwrite(maskName,tmaskImg);

	imshow("Combine-finalRes",tImg);

	imshow("maskImg",tmaskImg);


}

void StopMotion::imageGenerateAll(Mat& leftImg, Mat& rightImg, MatrixXXi& BgFgMark,
	                              Mat& outPutImg, Mat& outFinalImg)
{
	//based on the rightImg

	IndexType hight = leftImg.rows;
	IndexType width = leftImg.cols;

	outPutImg.release();
	outPutImg.create(hight, width, CV_8UC3);
	outPutImg.zeros(hight, width, CV_8UC3);
	//rightImg.copyTo(outPutImg);

	// original version
	Mat maskImg;
	maskImg.create(hight, width, CV_8UC3);

	for (int y = 0; y < hight; y++)
	{
		for (int x = 0; x < width; x++)
		{

			if (BgFgMark(y, x) == 1)
			{
				outPutImg.at<cv::Vec3b>(y, x) = leftImg.at<cv::Vec3b>(y, x); // large change
				maskImg.at<cv::Vec3b>(y, x) = fgMask;

			}
			else
			{
				outPutImg.at<cv::Vec3b>(y, x) = rightImg.at<cv::Vec3b>(y, x); //small change-keep itself.
				maskImg.at<cv::Vec3b>(y, x) = bgMask;
			}
		}
	}


	//using background pixels for hand

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1]].sltRect;


	if (roi.size() <= 0)
	{
		Loggger << "Rectangle is error!.\n";
		return;
	}

	outPutImg.copyTo((bg)(roi[0]));


	Mat tmaskImg;

	//cvtColor(bg, outFinalImg, CV_RGB2BGR);
	bg.copyTo(outFinalImg);
	cvtColor(maskImg, tmaskImg, CV_RGB2BGR);

	// save the results
	imshow("maskImg", tmaskImg);

}
void StopMotion::drawInitialMatching(Mat& bg)
{
// 	if (m_lMPixels.size() > 0)
// 	{
// 	}

}


struct MatchingInfo
{
	Point srPs;
	Point tgPs;
	MatchingInfo(Point _srP, Point _tgP)
	{
		srPs = _srP;
		tgPs = _tgP;
	}
};


void StopMotion::brushHandRegionAsBlack(Mat& oriImg, MatrixXXi& labels, Mat& outImg)
{

	oriImg.copyTo(outImg);

	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			if(labels(y,x) == 1)
			{
				outImg.at<Vec3b>(y,x) = bgMask;
			}

		}
	}
}

void StopMotion::localDeformation(Mat& leftImg, Mat& rightImg, 
								  MatrixXXi& BgFgMark, Mat& outPutImg)
{

    //1-find the seam 	
 	vector<Point2f> ptsList;
 	findSeamPosList(BgFgMark,ptsList); //by matlab's function

	vector<Point> lMatchPixels,rMatchPixels; 

 //	//2-1-show seam pixels
 //	//showSeamPixels(leftImg,rightImg,ptsList);// compare the pixels along the boundary
 //
	//// keep the ptslist along the objects of the only-substracting the background 
 //   vector<Point2f> seam;
	//downSamplePtslist(leftImg,ptsList,seam);

	////show down-sample boundary
	////showSeamPixels(leftImg,rightImg,seam);

 //	//3-1-matching feature points along the boundary

 //	//matchingFeaturePs(leftImg,rightImg,ptsList,lMatchPixels,rMatchPixels);
	//matchingFeaturePs(leftImg,rightImg,seam,lMatchPixels,rMatchPixels);


	// d-sample for two seam respectively
 	vector<Point2f> lSeam,rSeam;
// 	Mat ltp1,ltp2,ltp3;
// 	Mat rtp1,rtp2,rtp3;
// 
// 	leftImg.copyTo(ltp1);
// 	leftImg.copyTo(ltp2);
// 	leftImg.copyTo(ltp3);
// 
// 	rightImg.copyTo(rtp1);
// 	rightImg.copyTo(rtp2);
// 	rightImg.copyTo(rtp3);
//  	//2-2
// 	Scalar lcolor(255,0,0);
// 	Scalar rcolor(0,0,255);

 	//downsamplePtsList42(leftImg,rightImg,ptsList,lSeam,rSeam);

	//using hsv difference
	downsamplePtsUsingDifferenceHSV(leftImg,rightImg,ptsList,lSeam,rSeam);

	//for test consecutive situation
	//continueDownSample(leftImg,rightImg,ptsList,lSeam,rSeam);
// 	for (IndexType i = 0; i < lSeam.size(); ++ i)
// 	{
// 		Point2f curP = lSeam[i];
// 		circle(ltp1,curP,5,lcolor);
// 	}
// 	for (IndexType j = 0; j < rSeam.size(); ++ j)
// 	{
// 		Point2f curp = rSeam[j];
// 		circle(rtp1,curp,5,rcolor);
// 	}
// 	Mat ltp1t,rtp1t;
// 	cvtColor(ltp1,ltp1t,CV_RGB2BGR);
// 	cvtColor(rtp1,rtp1t,CV_RGB2BGR);
// 	imshow("L-1",ltp1t);
// 	imshow("R-1",rtp1t);
// 
// 	lSeam.clear();
// 	rSeam.clear();

    //downsamplePtsList42SIFT(leftImg,rightImg,ptsList,lSeam,rSeam);//compare the sift features descriptor

// 	for (IndexType i = 0; i < lSeam.size(); ++ i)
// 	{
// 		Point2f curP = lSeam[i];
// 		circle(ltp2,curP,5,lcolor);
// 	}
// 	for (IndexType j = 0; j < rSeam.size(); ++ j)
// 	{
// 		Point2f curp = rSeam[j];
// 		circle(rtp2,curp,5,rcolor);
// 	}
// 
// 	Mat ltp2t,rtp2t;
// 	cvtColor(ltp2,ltp2t,CV_RGB2BGR);
// 	cvtColor(rtp2,rtp2t,CV_RGB2BGR);
// 	imshow("L-2",ltp2t);
// 	imshow("R-2",rtp2t);
// 	lSeam.clear();
// 	rSeam.clear();


	//combine2DownSample(leftImg,rightImg,ptsList,lSeam,rSeam);

	//vector<Point2f> refLPtslist,refRPtslist;
	//refineDownedPtslist(leftImg,rightImg,ptsList,lSeam,rSeam,refLPtslist,refRPtslist);


// 	for (IndexType i = 0; i < lSeam.size(); ++ i)
// 	{
// 		Point2f curP = lSeam[i];
// 		circle(ltp3,curP,5,lcolor);
// 	}
// 	for (IndexType j = 0; j < rSeam.size(); ++ j)
// 	{
// 		Point2f curp = rSeam[j];
// 		circle(rtp3,curp,5,rcolor);
// 	}
// 
// 	Mat ltp3t,rtp3t;
// 	cvtColor(ltp3,ltp3t,CV_RGB2BGR);
// 	cvtColor(rtp3,rtp3t,CV_RGB2BGR);
// 	imshow("L-3",ltp3t);
// 	imshow("R-3",rtp3t);
// 	lSeam.clear();
// 	rSeam.clear();

	// only for pixels located on the intersection place?

 	//3-2
 	//matchingFeaturePs2(leftImg,rightImg,ptsList,lSeam,rSeam,lMatchPixels,rMatchPixels);

	// brush the region as bg
	Mat bRightImg;
	brushHandRegionAsBlack(rightImg,BgFgMark,bRightImg);
	imshow("bRIMG",bRightImg);
	matchingFeaturePs2(leftImg,bRightImg,ptsList,lSeam,rSeam,lMatchPixels,rMatchPixels);

	//3-3- using a overlap information
// 	Rect exRect;
// 	matchingFeatureOverLapLee(leftImg,rightImg,ptsList,lMatchPixels,rMatchPixels,exRect);


	Mat mTp,rtp;
	leftImg.copyTo(mTp);
	rightImg.copyTo(rtp);
	showPixMatching(mTp,rtp,lMatchPixels,rMatchPixels);
	//end for matching process of points!

	////4-using matlab codes for detecting the edges
 	Mat LEdges,REdges;
 	spClassify.detectEdgesName(leftImg,LEdges,false);
 	spClassify.detectEdgesName(rightImg,REdges,true);//should set the hand's region with blank, so no edges can cross the boun
 
 	//5-find the edge's component which the size of the points is large than a threshold.
 	vector<vector<Point> > leftContours, rightContours;
 	findEdgeComponent(LEdges,REdges,leftContours,rightContours);
 
 	//6-which edges cross the seam? return the idx of the component.
 	vector<IndexType> lcroedgeIdx, rcroedgeIdx;
 	detectEdgesCrossBoundary(leftImg,leftContours,rightContours,ptsList,
 		                     lcroedgeIdx,rcroedgeIdx);

	//detect the line segments which cross the seam!
	vector<lineSeg> leftLines;
	vector<lineSeg> rightLines;

	getLinesSegmentCorssSeam(leftImg,ptsList,
		leftContours,rightContours,
		lcroedgeIdx,rcroedgeIdx,
		leftLines,rightLines);

	//show the line segments
	showLines(leftImg,rightImg,ptsList,leftLines,rightLines);

//
//	//7-matching edges which cross the boundary.
  	//vector<Point> lMatchEdges,rMatchEdges;
 // //// 	matchEdgesCrossSeam(leftImg,rightImg,ptsList,leftContours,rightContours,
 // //// 		                lcroedgeIdx,rcroedgeIdx,lMatchEdges,rMatchEdges);
 // //
 // //
  //	//8-combine two matching points
  	//vector<CvPoint2D32f> srMatchingPs,tgMatchingPs;
  	//combineMatching(lMatchPixels,rMatchPixels,lMatchEdges,rMatchEdges,srMatchingPs,tgMatchingPs);
  
  
  	//the above method used the automatic way;
  
  	//can also add a few interaction method for matching
  
  
  	////9-deform the right image
  	 //deformRoiWithMathcing(leftImg,rightImg,ptsList,srMatchingPs,tgMatchingPs,outPutImg);

}

void StopMotion::showPixMatching(Mat& leftImg, Mat rightImg, vector<Point>& lPos, vector<Point>& rPos)
{
	assert(lPos.size() == rPos.size() );

	RNG rng(12345);

	for (int i = 0; i < lPos.size(); ++ i)
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		circle(leftImg,lPos[i],5,color);
		circle(rightImg,rPos[i],5,color);

	}

// 	Mat atp,btp;
// 	cvtColor(leftImg,atp,CV_RGB2BGR);
// 	cvtColor(rightImg,btp,CV_RGB2BGR);
// 	imshow("lPixMatching", atp);
// 	imshow("rPixMatching", btp);


	Mat drawIMg, drawTemp;

	vector<DMatch> matchPs;
	vector<KeyPoint> lKeyPs,rKeyPos;

	for (int i = 0; i< lPos.size(); ++ i)
	{
		DMatch tp;
		tp.queryIdx = i;
		tp.trainIdx = i;
		matchPs.push_back(tp);
        
		lKeyPs.push_back(KeyPoint(lPos[i],1.f));
		rKeyPos.push_back(KeyPoint(rPos[i],1.f));

	}

	drawMatches(leftImg, lKeyPs,rightImg,rKeyPos, matchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("MatchingImg",drawTemp);

}

void StopMotion::showPixMatching(Mat& leftImg, Mat rightImg,
								 vector<CvPoint2D32f>& lPos, 
								 vector<CvPoint2D32f>& rPos)
{
	assert(lPos.size() == rPos.size() );

	RNG rng(12345);

	for (int i = 0; i < lPos.size(); ++ i)
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		circle(leftImg,lPos[i],5,color);
		circle(rightImg,rPos[i],5,color);

	}

// 	Mat atp,btp;
// 	cvtColor(leftImg,atp,CV_RGB2BGR);
// 	cvtColor(rightImg,btp,CV_RGB2BGR);
// 	imshow("lPixMatching", atp);
// 	imshow("rPixMatching", btp);


	Mat drawIMg, drawTemp;

	vector<DMatch> matchPs;
	vector<KeyPoint> lKeyPs,rKeyPos;

	for (int i = 0; i< lPos.size(); ++ i)
	{
		DMatch tp;
		tp.queryIdx = i;
		tp.trainIdx = i;
		matchPs.push_back(tp);

		lKeyPs.push_back(KeyPoint(lPos[i],1.f));
		rKeyPos.push_back(KeyPoint(rPos[i],1.f));

	}

	drawMatches(leftImg, lKeyPs,rightImg,rKeyPos, matchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("Down-MatchingImg",drawTemp);
}


//automatic 
void StopMotion::findInitialMatchingAlongSeam(vector<Point2f>& ptslist, 
											  vector<CvPoint2D32f>& lmatchPs,
											  vector<CvPoint2D32f>& rmatchPs)
{

	//brush the right image.
// 	Mat rROI;
// 	brushHandRegionAsBlack(m_rROI,m_labels,rROI);

	vector<lineSeg> leftLines,rightLines;
	vector<IndexType> lineMatching;//invalid

	//obtainLinesMatching(m_lROI, rROI, ptslist, leftLines,rightLines,lineMatching);
	//obtainLinesMatching(m_lROI, m_rROI, ptslist, leftLines,rightLines,lineMatching);

	// detect the edges of the two images;

	vector<Point> lPoints, rPoints;
	obtainLinesMatchingPs(ptslist, leftLines,rightLines,lPoints,rPoints);


	//down sample the points! how?
	//eliminate the background pixels
	vector<Point2f> lObPs,rObPs;
	downsamplePtsUsingDifferenceHSV(m_lROI,m_rROI,ptslist,lObPs,rObPs);

	goDownsample(lObPs,lPoints);
	goDownsample(rObPs,rPoints);

	//showInitialInputPs(lPoints,rPoints);

	//
	MatchingLineInterSectionPs(ptslist,lPoints,rPoints,lmatchPs,rmatchPs);//dynamic pro



}

void StopMotion::MatchingLineInterSectionPs(vector<Point2f>& ptslist,
											vector<Point>& lPoints,vector<Point>& rPoints, 
											vector<CvPoint2D32f>& lmatchPs, vector<CvPoint2D32f>& rmatchPs)
{

	//using dynamic programming processing

   IndexType lSize = lPoints.size();
   IndexType rSize = rPoints.size();

   if(lSize == 0 || rSize == 0 )
   {
	   Loggger<<"not enough intersection points for matching.\n";
	   return;
   }

   bool isRevs = false;
   if (lSize > rSize)
   {
	   isRevs = true;
   }

   //right->left
   dynamicInterSPsMatching(ptslist,lPoints,rPoints,lmatchPs,rmatchPs,isRevs);

  //show matching information
   showPixMatching(m_lROI,m_rROI,lmatchPs,rmatchPs);

}


void StopMotion::dynamicInterSPsMatching(vector<Point2f>& ptslist,vector<Point>& lPoints,vector<Point>& rPoints, 
										 vector<CvPoint2D32f>& lmatchPs, vector<CvPoint2D32f>& rmatchPs, 
										 bool isReves)
{
	//calculate distance between two sets.
	MatrixXX disMat;
	disMat.setZero(lPoints.size(),rPoints.size());

	calculateDisTwoSets(lPoints,rPoints,disMat);

	//
	vector<vector<ScalarType> > disVec;

	if (isReves)
	{
		MatrixXX revDis  = disMat.transpose();

		for (IndexType i = 0; i < revDis.rows(); ++ i)
		{
			vector<ScalarType> curDis;

			for (IndexType j = 0; j < revDis.cols(); ++ j)
			{
				curDis.push_back(revDis(i,j));
			}

			disVec.push_back(curDis);
		}

	}else
	{
		for (IndexType i = 0; i < disMat.rows(); ++ i)
		{
			vector<ScalarType> curDis;

			for (IndexType j = 0; j < disMat.cols(); ++ j)
			{
				curDis.push_back(disMat(i,j));
			}

			disVec.push_back(curDis);
		}
	}

	//using dynamic processing 
	//start to matching

	vector<IndexType> lMIdx,rMIdx;
	dynamicProgramPsMatching(disVec,lPoints,rPoints,lMIdx,rMIdx);

	if (isReves)
	{
		for (IndexType i = 0; i < lMIdx.size(); ++ i)
		{
			lmatchPs.push_back(lPoints[rMIdx[i]]);
			rmatchPs.push_back(rPoints[lMIdx[i]]);
		}

	}else
	{
		for (IndexType i = 0; i < lMIdx.size(); ++ i)
		{
			lmatchPs.push_back(lPoints[lMIdx[i]]);
			rmatchPs.push_back(rPoints[rMIdx[i]]);
		}
	}



}

void StopMotion::dynamicProgramPsMatching(vector<vector<ScalarType>>& disM,
	                                      vector<Point>& lPoints,vector<Point>& rPoints, 
										  vector<IndexType>& lmatchPsIdx, vector<IndexType>& rmatchPsIdx)
{
	IndexType lrows = disM.size();
	IndexType rcols = disM[0].size();

	MatrixXX costMat;//record the cost for matching between pixels from the images
	costMat.resize(lrows + 1, rcols + 1);
	costMat.setConstant(INF);
	costMat.row(0).setConstant(0.);


	// 0   0  0   0   0  0 
	//INF  1  1   1   1 INF
	//INF INF 3   4   2 INF
	//INF INF INF 5   7  8
	//INF INF INF INF 9 INF

	for (int i = 1; i < lrows + 1; ++ i)
	{

		for (int j = i; j < rcols + 1; ++j)
		{
			vector<ScalarType> minE; 
			minE.clear();

			int stidx = i-1;
			int endidx = j;

			for (int k = stidx; k < endidx; ++ k)
			{
				ScalarType tp = 0.;

				tp = costMat(stidx,k) + *(min_element(disM[stidx].begin() + k, disM[stidx].begin() + endidx) );

				minE.push_back(tp);
			}

			costMat(i,j) = *(min_element(minE.begin(), minE.end() ) );

		}

	}

	//reverse


	lmatchPsIdx.clear();
	rmatchPsIdx.clear();

	int colsIdx = rcols;
	for (int i = lrows; i > 0; -- i/*, -- colsIdx*/)
	{	
		MatrixXX temp = costMat.block(i,1,1,colsIdx);

		IndexType pos = 1;
		temp.row(0).minCoeff(&pos);

		ScalarType maxV = temp.row(0).maxCoeff();
		if (maxV > INF)
		{
			printf("%d point no correspondence.\n",i);
			continue;
		}

		if ( pos < i - 1)//cross happened!
		{
			Loggger<<"Cross happened!\n";
			//continue;
		}
		lmatchPsIdx.insert(lmatchPsIdx.begin(),i - 1);
		rmatchPsIdx.insert(rmatchPsIdx.begin(),pos);
		colsIdx = pos;
	}


// 	for (int i = 0; i < lMatchingIdx.size(); ++ i)
// 	{
// 		printf("left %d matching %d right.\n",lMatchingIdx[i],rMatchingIdx[i]);
// 		lmatchPs.push_back(lPoints[lMatchingIdx[i]] );
// 		rmatchPs.push_back(rPoints[rMatchingIdx[i]] );
// 	}

}

void StopMotion::calculateDisTwoSets(vector<Point>& lPoints,vector<Point>& rPoints, 
									 MatrixXX& disMat)
{
	IndexType nL = lPoints.size();
	IndexType nR = rPoints.size();

   for (IndexType i = 0; i < nL; ++ i)
   {
	   for (IndexType j = 0; j < nR; ++ j)
	   {
		   disMat(i,j) = p2pDis(lPoints[i],rPoints[j]);//for another choices.
	   }
   }
}

ScalarType StopMotion::p2pDis(Point& lPs,Point& rPs)
{
	return sqrt( (lPs.x - rPs.x)*(lPs.x - rPs.x) + (lPs.y- rPs.y)*(lPs.y - rPs.y));
}

void StopMotion::goDownsample(vector<Point2f>& lObPs, vector<Point>& lLinePs)
{

	set<IndexType> allPs;

	for (IndexType i = 0; i < lObPs.size(); ++ i)
	{
		IndexType xC = lObPs[i].x;

		IndexType yC = lObPs[i].y;

		IndexType xyKey = frame_frame_to_key(xC,yC);

		allPs.insert(xyKey);
	}

	std::set<int>::iterator it;

	vector<Point> temp;

	for (IndexType j = 0; j < lLinePs.size(); ++ j)
	{
		IndexType curKey = frame_frame_to_key(lLinePs[j].x, lLinePs[j].y);

		it = allPs.find(curKey);

		if (it != allPs.end())
		{
			temp.push_back(lLinePs[j]);
		}
	}

	lLinePs = temp;
}


void StopMotion::showInitialInputPs(vector<Point>& lPoints,vector<Point>& rPoints)
{
	Mat ltemp,rtemp,ltt,rtt;
	m_lROI.copyTo(ltemp);
	m_rROI.copyTo(rtemp);

	cvtColor(ltemp,ltt,CV_RGB2BGR);
	cvtColor(rtemp,rtt,CV_RGB2BGR);

	for (IndexType i = 0; i < lPoints.size(); ++ i)
	{
		circle(ltt, lPoints[i],2,Scalar(255,0,0) );
	}

	for (IndexType i = 0; i < rPoints.size(); ++ i)
	{
		circle(rtt, rPoints[i],2,Scalar(255,0,0) );
	}

	imshow("LeftInputPs",ltt);
	imshow("RightInputPs",rtt);

}

void StopMotion::downSamplePtslist(Mat& lImg,vector<Point2f>& oriList,vector<Point2f>& downList)
{
   Mat roiBG,droiBG;
  
   Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
   Mat dbg;

   if (m_sltRect.size() > 0)
   {
       (*bg)(m_sltRect[0]).copyTo(roiBG);

       if (m_glbHomo.rows > 0)
       {
		   // deformation 
		   warpPerspective(roiBG,droiBG,m_glbHomo,roiBG.size());

       }else
	   {
		   roiBG.copyTo(droiBG);
	   }
	   
	   // smooth those two images for subtraction
	   Mat bgSImg,curSImg;

	   GaussianBlur(droiBG,bgSImg,Size(3,3), 0, 0, BORDER_DEFAULT);
	   GaussianBlur(lImg,curSImg,Size(3,3), 0, 0, BORDER_DEFAULT );

	   for (IndexType i = 0; i < oriList.size(); ++ i)
	   {
		   Vec3b bgColor = bgSImg.at<Vec3b>(oriList[i].y,oriList[i].x);
		   Vec3b curColor = curSImg.at<Vec3b>(oriList[i].y,oriList[i].x);

		   bool isfg = isDifferent(bgColor,curColor);
		   //another choice is to use shape descriptor 

		   bool isBack = false;
		   if (curColor[0] < 10 && curColor[1] < 10 && curColor[2] < 10)
		   {
			   isBack = true;
		   }

		   if (isfg && !isBack)
		   {
			   downList.push_back(oriList[i]);
		   }

	   }

   }else
   {
	   downList = oriList;
   }
}

void StopMotion::downsamplePtsList42(Mat& lImg,Mat& rImg, 
									 vector<Point2f>& oriList, 
									 vector<Point2f>& ldList,
									 vector<Point2f>& rdList)
{
	//down sample for two images respectively
	Mat roiBG,droiBG;

	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
	Mat dbg, bgSImg;

	if (m_sltRect.size() > 0)
	{
		(*bg)(m_sltRect[0]).copyTo(roiBG);

		if (m_glbHomo.rows > 0)
		{
			// deformation 
			warpPerspective(roiBG,droiBG,m_glbHomo,roiBG.size());

		}else
		{
			roiBG.copyTo(droiBG);
		}

		//smooth for bg image
	    GaussianBlur(droiBG,bgSImg,Size(3,3), 0, 0, BORDER_DEFAULT);

	}else
	{
		ldList = oriList;
		rdList = oriList;

		return;
	}

	Mat lSimg,rSimg;

	GaussianBlur(lImg,lSimg,Size(3,3), 0, 0, BORDER_DEFAULT );
	GaussianBlur(rImg,rSimg,Size(3,3), 0, 0, BORDER_DEFAULT );

	for (IndexType i = 0; i < oriList.size(); ++ i)
	{
		Vec3b bgColor = bgSImg.at<Vec3b>(oriList[i].y,oriList[i].x);
		Vec3b lcurColor = lSimg.at<Vec3b>(oriList[i].y,oriList[i].x);
		Vec3b rcurColor = rSimg.at<Vec3b>(oriList[i].y,oriList[i].x);

		//judge for left image
		bool lisfg = isDifferent(bgColor,lcurColor);
		bool lisBack = false;
		if (lcurColor[0] < 10 && lcurColor[1] < 10 && lcurColor[2] < 10)
		{
			lisBack = true;
		}

		if (lisfg && !lisBack)
		{
			ldList.push_back(oriList[i]);
		}


		//judge for right image
		bool risfg = isDifferent(bgColor,rcurColor);
		bool risHand = isHandPixel(rcurColor);

		bool risBack = false;

		if (rcurColor[0] < 10 && rcurColor[1] < 10 && rcurColor[2] < 10)
		{
			risBack = true;
		}

		if (risfg && !risBack && !risHand)
		{
			rdList.push_back(oriList[i]);
		}

	}

}

void StopMotion::continueDownSample(Mat& lImg,Mat& rImg,
									vector<Point2f>& oriList, 
									vector<Point2f>& ldList,vector<Point2f>& rdList)
{
	ScalarType dration = .3;

	IndexType nPs = (IndexType)(dration * oriList.size());

	ldList.clear();
	rdList.clear();

	for (IndexType i = 0; i < nPs; ++ i)
	{
		ldList.push_back(oriList[i]);
		rdList.push_back(oriList[i]);
	}

}

void StopMotion::downsamplePtsUsingDifferenceHSV(Mat& lImg,Mat& rImg, 
												 vector<Point2f>& oriList, 
												 vector<Point2f>& ldList,
												 vector<Point2f>& rdList)
{
	//down sample for two images respectively
	Mat roiBG,droiBG;

	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
	Mat dbg, bgSImg;

	if (m_sltRect.size() > 0)
	{
		(*bg)(m_sltRect[0]).copyTo(roiBG);

		if (m_glbHomo.rows > 0)
		{
			// deformation 
			warpPerspective(roiBG,droiBG,m_glbHomo,roiBG.size());

		}else
		{
			roiBG.copyTo(droiBG);
		}

		//smooth for bg image
		GaussianBlur(droiBG,bgSImg,Size(3,3), 0, 0, BORDER_DEFAULT);

	}else
	{
		ldList = oriList;
		rdList = oriList;

		return;
	}

	Mat lSimg,rSimg;

	GaussianBlur(lImg,lSimg,Size(3,3), 0, 0, BORDER_DEFAULT );
	GaussianBlur(rImg,rSimg,Size(3,3), 0, 0, BORDER_DEFAULT );

	Mat hsvbg,hsvleft,hsvright;

	cvtColor(bgSImg,hsvbg,CV_RGB2HSV);
	cvtColor(lSimg,hsvleft,CV_RGB2HSV);
	cvtColor(rSimg,hsvright,CV_RGB2HSV);


	//find the differences between two images

	Mat lDiff,rDiff;
	absdiff(hsvbg,hsvleft,lDiff);
	absdiff(hsvbg,hsvright,rDiff);

	ScalarType bgThreshold = 50;

	ldList.clear();
	rdList.clear();

	vector<Point2f> ltempList,rtempList;

	for (IndexType i = 0; i < oriList.size(); ++ i)
	{

		Vec3b lpixdiff = lDiff.at<Vec3b>(oriList[i].y,oriList[i].x);
		Vec3b rpixdiff = rDiff.at<Vec3b>(oriList[i].y,oriList[i].x);

		ScalarType ldist = sqrt(lpixdiff[0]*lpixdiff[0] + 
			                lpixdiff[1]*lpixdiff[1] + lpixdiff[2]*lpixdiff[2]);

		ScalarType rdist = sqrt(rpixdiff[0]*rpixdiff[0] + 
			rpixdiff[1]*rpixdiff[1] + rpixdiff[2]*rpixdiff[2]);

		if (ldist > bgThreshold)// judge for left seam
		{
			bool lisBack = false;
			Vec3b lcurColor = lSimg.at<Vec3b>(oriList[i].y,oriList[i].x);

			if (lcurColor[0] < 10 && lcurColor[1] < 10 && lcurColor[2] < 10)
			{
				lisBack = true;
			}

			if (!lisBack)
			{
				/*ldList.push_back(oriList[i]);*/
				ltempList.push_back(oriList[i]);
			}
		}

		if (rdist > bgThreshold) // judge for right seam
		{
			Vec3b rcurColor = rSimg.at<Vec3b>(oriList[i].y,oriList[i].x);
			////bool risHand = isHandPixel(rcurColor);

			bool risHand = isSkinPixel(rcurColor);
			if (!risHand)
			{
				/*rdList.push_back(oriList[i]);*/
			    rtempList.push_back(oriList[i]);
			}

		}

	}//end for all seam

	//if do not refine 
// 	ldList = ltempList;
// 	rdList = rtempList;

	//continue to refine the down sample points?

		vector<KeyPoint> lkeyP,rkeyP;
		IndexType lpsize = ltempList.size();
		IndexType rpsize = rtempList.size();

		if (lpsize < 2 || rpsize < 2)
		{
			return;
		}

		for (IndexType i = 0; i < lpsize; ++ i)
		{
			Point2f curp = ltempList[i];
			lkeyP.push_back(KeyPoint(curp,1.f));
		}

		for (IndexType j = 0; j < rpsize; ++ j)
		{
			Point2f curp = rtempList[j];
			rkeyP.push_back(KeyPoint(curp,1.f));
		}


		Ptr<DescriptorExtractor> descriptorExtractor;  
		string detectorType = "SIFT";  
		descriptorExtractor = DescriptorExtractor::create( detectorType ); 

		Mat lbgDesc,rbgDesc,lDesc,rDesc;
		lbgDesc.setTo(0);
		rbgDesc.setTo(0);
		lDesc.setTo(0);
		rDesc.setTo(0);

		descriptorExtractor->compute(bgSImg,lkeyP,lbgDesc);
		descriptorExtractor->compute(lSimg,lkeyP,lDesc);

		descriptorExtractor->compute(bgSImg,rkeyP,rbgDesc);
		descriptorExtractor->compute(rSimg,rkeyP,rDesc);

		vector<ScalarType> ldist,rdist;
		ldist.clear();
		rdist.clear();

		for (IndexType i = 0; i < lpsize; ++ i)
		{
			Mat lDiff = lDesc.row(i) - lbgDesc.row(i);
			ScalarType lvDiff = norm(lDiff,NORM_L2);
			ldist.push_back(lvDiff);
		}

		for (IndexType j = 0; j < rpsize; ++ j)
		{
			Mat rDiff = rDesc.row(j) - rbgDesc.row(j);
			ScalarType rvDiff = norm(rDiff,NORM_L2);
			rdist.push_back(rvDiff);
		}

		vector<ScalarType> tDist = ldist;

		vector<ScalarType> rDist = rdist;

		sort(tDist.begin(),tDist.end());

		sort(rDist.begin(),rDist.end());
		
		ScalarType thres = 0.1;// is a variable

		IndexType mPos = (int)(thres * tDist.size());

		IndexType rPos = (int)(thres * rDist.size());

		assert(mPos > 0 && mPos < tDist.size());

		ScalarType mThres = tDist[mPos];

		ScalarType rThres = rDist[rPos];

		ldList.clear();
		rdList.clear();

		for (IndexType i = 0; i < lpsize; ++ i)
		{
			ScalarType tDiff = ldist[i];
			if (tDiff > mThres)
			{
				ldList.push_back(ltempList[i]);
			}
		}


		for (IndexType j = 0; j < rpsize; ++ j)
		{
			ScalarType tDiff = rdist[j];
			if (tDiff > rThres)
			{
				rdList.push_back(rtempList[j]);
			}
		}

}

void StopMotion::downsamplePtsList42SIFT(Mat& lImg,Mat& rImg, 
										 vector<Point2f>& oriList,
										 vector<Point2f>& ldList,
										 vector<Point2f>& rdList)
{
	//down sample for two images respectively
	Mat roiBG,droiBG;

	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
	Mat dbg, bgSImg;

	if (m_sltRect.size() > 0)
	{
		(*bg)(m_sltRect[0]).copyTo(roiBG);

		if (m_glbHomo.rows > 0)
		{
			// deformation 
			warpPerspective(roiBG,droiBG,m_glbHomo,roiBG.size());

		}else
		{
			roiBG.copyTo(droiBG);
		}

		//smooth for bg image
		GaussianBlur(droiBG,bgSImg,Size(3,3), 0, 0, BORDER_DEFAULT);

	}else
	{
		ldList = oriList;
		rdList = oriList;

		return;
	}

	Mat lSimg,rSimg;

	GaussianBlur(lImg,lSimg,Size(3,3), 0, 0, BORDER_DEFAULT );
	GaussianBlur(rImg,rSimg,Size(3,3), 0, 0, BORDER_DEFAULT );

	//using SIFT descriptor to determine whether or not belongs to background
	IndexType psize = oriList.size();

	vector<KeyPoint> keyPs;

	for (IndexType i = 0; i < psize; ++ i)
	{
		Point2f tp;
		tp.x = oriList[i].y;
		tp.y = oriList[i].x;
		keyPs.push_back(KeyPoint(tp,1.f));		
	}

	Ptr<DescriptorExtractor> descriptorExtractor;  
	string detectorType = "SIFT";  
	descriptorExtractor = DescriptorExtractor::create( detectorType ); 


	Mat bgDesc,lDesc,rDesc;
	bgDesc.setTo(0);
	lDesc.setTo(0);
	rDesc.setTo(0);

	descriptorExtractor->compute(bgSImg,keyPs,bgDesc);
	descriptorExtractor->compute(lSimg,keyPs,lDesc);
	descriptorExtractor->compute(rSimg,keyPs,rDesc);

	vector<ScalarType> dist,rdist;
	dist.clear();
	rdist.clear();

	for (IndexType i = 0; i < psize; ++ i)
	{
		Mat lDiff = lDesc.row(i) - bgDesc.row(i);
		Mat rDiff = rDesc.row(i) - bgDesc.row(i);

		ScalarType lvDiff = norm(lDiff,NORM_L2);
		ScalarType rvDiff = norm(rDiff,NORM_L2);
		dist.push_back(lvDiff);
		rdist.push_back(rvDiff);

	}

	vector<ScalarType> tDist = dist;

	vector<ScalarType> rDist = rdist;

	sort(tDist.begin(),tDist.end());//existing a case that most of the element equal to 0!

	sort(rDist.begin(),rDist.end());

	ScalarType thres = 0.55;// is a variable

	IndexType mPos = (int)(thres * tDist.size());

	assert(mPos > 0 && mPos < tDist.size());

	ScalarType mThres = tDist[mPos];

	ScalarType rThres = rDist[mPos];

	ldList.clear();
	rdList.clear();

	for (IndexType i = 0; i < psize; ++ i)
	{
		ScalarType tDiff = dist[i];

		ScalarType rDiff = rdist[i];

		if (tDiff > mThres)
		{
			ldList.push_back(oriList[i]);
		}

		if (rDiff > rThres)
		{
			rdList.push_back(oriList[i]);
		}

	}


}



void  StopMotion::combine2DownSample(Mat& lImg,Mat& rImg, 
									 vector<Point2f>& oriList, 
									 vector<Point2f>& ldList,vector<Point2f>& rdList)
{
	//step1: using a threshold to separate the pixels from bg

	//stpe2: using SIFT descriptor 

	//down sample for two images respectively

	vector<Point2f> ltempList,rtempList;
	ltempList.clear();
	rtempList.clear();

	Mat roiBG,droiBG;

	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
	Mat dbg, bgSImg;

	if (m_sltRect.size() > 0)
	{
		(*bg)(m_sltRect[0]).copyTo(roiBG);

		if (m_glbHomo.rows > 0)
		{
			// deformation 
			warpPerspective(roiBG,droiBG,m_glbHomo,roiBG.size());

		}else
		{
			roiBG.copyTo(droiBG);
		}

		//smooth for bg image
		GaussianBlur(droiBG,bgSImg,Size(3,3), 0, 0, BORDER_DEFAULT);

	}else
	{
		ldList = oriList;
		rdList = oriList;

		return;
	}

	Mat lSimg,rSimg;

	GaussianBlur(lImg,lSimg,Size(3,3), 0, 0, BORDER_DEFAULT );
	GaussianBlur(rImg,rSimg,Size(3,3), 0, 0, BORDER_DEFAULT );

	for (IndexType i = 0; i < oriList.size(); ++ i)
	{
		Vec3b bgColor = bgSImg.at<Vec3b>(oriList[i].y,oriList[i].x);
		Vec3b lcurColor = lSimg.at<Vec3b>(oriList[i].y,oriList[i].x);
		Vec3b rcurColor = rSimg.at<Vec3b>(oriList[i].y,oriList[i].x);

		//judge for left image
		bool lisfg = isDifferent(bgColor,lcurColor);
		bool lisBack = false;

		ScalarType bgVal = 10.;

		if (lcurColor[0] <bgVal && lcurColor[1] < bgVal && lcurColor[2] < bgVal)
		{
			lisBack = true;
		}

		if (lisfg && !lisBack)
		{
			ltempList.push_back(oriList[i]);
		}

		//judge for right image
		bool risfg = isDifferent(bgColor,rcurColor);
		bool risHand = isHandPixel(rcurColor);

		bool risBack = false;

		if (rcurColor[0] < bgVal && rcurColor[1] < bgVal && rcurColor[2] < bgVal)
		{
			risBack = true;
		}

		if (risfg && !risBack && !risHand)
		{
			rtempList.push_back(oriList[i]);
		}

	}

	//end for step 1, only obtain the initial down-sample results
	// ldList and rdList for input data.

	vector<KeyPoint> lkeyP,rkeyP;
	IndexType lpsize = ltempList.size();
	IndexType rpsize = rtempList.size();

	if (lpsize < 2 || rpsize < 2)
	{
		return;
	}

	for (IndexType i = 0; i < lpsize; ++ i)
	{
		Point2f curp = ltempList[i];
		lkeyP.push_back(KeyPoint(curp,1.f));
	}

	for (IndexType j = 0; j < rpsize; ++ j)
	{
		Point2f curp = rtempList[j];
		rkeyP.push_back(KeyPoint(curp,1.f));
	}


	Ptr<DescriptorExtractor> descriptorExtractor;  
	string detectorType = "SIFT";  
	descriptorExtractor = DescriptorExtractor::create( detectorType ); 


	Mat lbgDesc,rbgDesc,lDesc,rDesc;
	lbgDesc.setTo(0);
	rbgDesc.setTo(0);
	lDesc.setTo(0);
	rDesc.setTo(0);

	descriptorExtractor->compute(bgSImg,lkeyP,lbgDesc);
	descriptorExtractor->compute(lSimg,lkeyP,lDesc);

	descriptorExtractor->compute(bgSImg,rkeyP,rbgDesc);
	descriptorExtractor->compute(rSimg,rkeyP,rDesc);

	vector<ScalarType> ldist,rdist;
	ldist.clear();
	rdist.clear();

	for (IndexType i = 0; i < lpsize; ++ i)
	{
		Mat lDiff = lDesc.row(i) - lbgDesc.row(i);
		ScalarType lvDiff = norm(lDiff,NORM_L2);
		ldist.push_back(lvDiff);
	}

	for (IndexType j = 0; j < rpsize; ++ j)
	{
		Mat rDiff = rDesc.row(j) - rbgDesc.row(j);
		ScalarType rvDiff = norm(rDiff,NORM_L2);
		rdist.push_back(rvDiff);
	}

	vector<ScalarType> tDist = ldist;

	vector<ScalarType> rDist = rdist;

	sort(tDist.begin(),tDist.end());

	sort(rDist.begin(),rDist.end());

	ScalarType thres = 0.55;// is a variable

	IndexType mPos = (int)(thres * tDist.size());

	IndexType rPos = (int)(thres * rDist.size());

	assert(mPos > 0 && mPos < tDist.size());

	ScalarType mThres = tDist[mPos];

	ScalarType rThres = rDist[rPos];

	ldList.clear();
	rdList.clear();

	for (IndexType i = 0; i < lpsize; ++ i)
	{
		ScalarType tDiff = ldist[i];
		if (tDiff > mThres)
		{
			ldList.push_back(ltempList[i]);
		}
	}


	for (IndexType j = 0; j < rpsize; ++ j)
	{
		ScalarType tDiff = rdist[j];
		if (tDiff > rThres)
		{
			rdList.push_back(rtempList[j]);
		}
	}


}

// the boundaries of the two image are different.

void StopMotion::matchingFeaturePs2(Mat& leftImg, Mat& rightImg,
									vector<Point2f>& oriBoundary,
									vector<Point2f>& lboundary, 
									vector<Point2f>& rboundary, 
									vector<Point>& lMPixels, 
									vector<Point>& rMPixels)
{
	//get the gradient value along the boundary

	vector<ScalarType> lGradBoundary,rGradBoundary;
	getBoundaryGradient2(leftImg,rightImg,lboundary,rboundary,lGradBoundary,rGradBoundary);

	//draw the curves
	m_autoSelect = new AutoSelect(0, 1e4);
	m_autoSelect->setBand(10);
	m_autoSelect->smoothCurves(lGradBoundary,5);
	m_autoSelect->smoothCurves(rGradBoundary,5);

	//downsample: have two choices ,threshold or local maximize value
	vector<ScalarType> ltrunG,rtrunG;
	vector<IndexType> ltrunIdx,rtrunIdx;

	//truncate with a threshold, using relatively value, down-sample with threshold.

	ScalarType graThres = 0.9;
	m_autoSelect->truncationCurves(lGradBoundary,graThres,ltrunG,ltrunIdx);
	m_autoSelect->truncationCurves(rGradBoundary,graThres,rtrunG,rtrunIdx);

	//draw the curves of gradient
	spClassify.drawVectorByMatlab(ltrunG,0);// draw truncate gradient value
	spClassify.drawVectorByMatlab(rtrunG,0);

	//another way! down-sample using local max value pixel,is not suitable,because 
    //each window must have a point

    drawMaxPs2(leftImg,rightImg,lboundary,rboundary,ltrunIdx,rtrunIdx);

 	//down-sample the pixel from the smoothed data,using the local maximal position! 
 	vector<ScalarType> lDSMarkGrad,rDSMarkGrad;
 	vector<IndexType> lDSIdx,rDSIdx;
 
 
 	//down-sample again for dynamic matching. small-large, make sure (n<=m)
 
 	//bool usLocalMax = true;
 	bool usLocalMax = false;
 
 	if (usLocalMax)
 	{
 		downSampleCandidates(ltrunG,ltrunIdx,rtrunG,rtrunIdx,
 			lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx);
 	}else
 	{
 		downSampleLargeCandidates(ltrunG,ltrunIdx,rtrunG,rtrunIdx,
 			lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx);
 	}
 
 	//draw original
 	//drawMaxPs(leftImg,rightImg,boundary,ltrunIdx,rtrunIdx);
 
 	//!!!calculate the distance matrix between two markable pixel sets
 	vector<vector<ScalarType> > pix_distM;
 
 	//with down-sample
  	//calculateDisMatrix(leftImg,rightImg,oriBoundary,
  	//	lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx, pix_distM);
 
  	calculateDisMatrix2(leftImg,rightImg,oriBoundary,lboundary,rboundary,
  		lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx, pix_distM);
  
  	//left size must larger than right size
 
  	dynamicProMatching2(pix_distM,lboundary,rboundary,lDSIdx,rDSIdx,
  		lMPixels,rMPixels);

}

void StopMotion::imageGenerWithBackg(Mat& leftImg, Mat& rightImg, MatrixXXi& BgFgMark, 
									 Mat& bgROI,Mat& outPutImg)
{
	//based on the rightImg

	IndexType hight = leftImg.rows;
	IndexType width = leftImg.cols;

	outPutImg.release();
	outPutImg.create(hight,width,CV_8UC3);
	outPutImg.zeros(hight,width,CV_8UC3);

	for (int y = 0; y < hight; y++) 
	{
		for (int x = 0; x < width; x++)
		{

			if (BgFgMark(y,x) == 1) // shouldn't appear the bg pixels in the final result.
			{
				Vec3b curC = leftImg.at<cv::Vec3b>(y,x);
				if (bgMask == curC)
				{
					outPutImg.at<cv::Vec3b>(y,x) = bgROI.at<cv::Vec3b>(y,x);//existing bg pixels in the final image.
				}else
				{
					outPutImg.at<cv::Vec3b>(y,x) = leftImg.at<cv::Vec3b>(y,x);
				}

			}else
			{
				outPutImg.at<cv::Vec3b>(y,x) = rightImg.at<cv::Vec3b>(y,x); //small change-keep itself.
			}
		}
	}

	Mat tImg;

	cvtColor(outPutImg,tImg,CV_RGB2BGR);

	// save the results
	char comName[1024];

	sprintf(comName,".\\resCombine\\combine-%.2d-1005.jpg",m_curPairIdx);

	imwrite(comName, tImg);

	imshow("Combine-finalRes",tImg);
}

void StopMotion::handRemoval(Mat& leftImg, Mat& rightImg,
							 MatrixXXi& intialLabels, MatrixXXi& outBgFgMark)
{

	// segment the right image 

 	trainGMM(rightImg,intialLabels); 

  	minCut(rightImg,intialLabels,outBgFgMark);

// 	MatrixXXi finalBGFGMark;
// 
// 	for (IndexType iterN = 0; iterN < 0; ++ iterN)
// 	{
// 		updateGMM(rightImg,outBgFgMark);
// 
//  	    minCut(rightImg,outBgFgMark,finalBGFGMark);
// 	}

}

void StopMotion::imageLabelAfterGMM(MatrixXXi& lastLabels)
{
	m_initFGLabel.resize(m_imgHight,m_imgWidth);
	m_initBGLabel.resize(m_imgHight,m_imgWidth);

	m_initBGLabel.setZero();
	m_initFGLabel.setZero();

	for (int y = 0; y < m_imgHight; y++) 
	{
		for (int x = 0; x < m_imgWidth; x++)
		{
			if (lastLabels(y,x) ==0 )
			{

			}else
			{

			}
		}
	}

}

IndexType StopMotion::getLabel(IndexType alpha, ScalarType rgb[3])
{
	vector<ScalarType> allProb;

	for (IndexType i = 0; i < N_GAUSS; ++i)
	{
		ScalarType temp = computerProb(alpha,i,rgb);
		allProb.push_back(temp);
	}


	return (min_element(allProb.begin(),allProb.end()) - allProb.begin() );

}

ScalarType StopMotion::computerProb(IndexType alpha, IndexType comIdx, 
									ScalarType reflectance[3])
{

	ScalarType D;

	if (alpha == 0)
	{
		D = -m_logwBG[comIdx] + 0.5*( log(m_varBG[comIdx][0]*m_varBG[comIdx][1]*m_varBG[comIdx][2]) +
		(reflectance[0]-m_meansBG[comIdx][0])*(reflectance[0]-m_meansBG[comIdx][0])/m_varBG[comIdx][0] +
		(reflectance[1]-m_meansBG[comIdx][1])*(reflectance[1]-m_meansBG[comIdx][1])/m_varBG[comIdx][1] +
		(reflectance[2]-m_meansBG[comIdx][2])*(reflectance[2]-m_meansBG[comIdx][2])/m_varBG[comIdx][2] );
	}
	else
	{
		D = -m_logwFG[comIdx] + 0.5*( log(m_varFG[comIdx][0]*m_varFG[comIdx][1]*m_varFG[comIdx][2]) +
		(reflectance[0]-m_meansFG[comIdx][0])*(reflectance[0]-m_meansFG[comIdx][0])/m_varFG[comIdx][0] +
		(reflectance[1]-m_meansFG[comIdx][1])*(reflectance[1]-m_meansFG[comIdx][1])/m_varFG[comIdx][1] +
		(reflectance[2]-m_meansFG[comIdx][2])*(reflectance[2]-m_meansFG[comIdx][2])/m_varFG[comIdx][2] );
	}

	return D;
}

ScalarType StopMotion::maxProb(IndexType alpha, ScalarType reflectance[3] )
{

    vector<ScalarType> allProb;

	for (IndexType i = 0; i < N_GAUSS; ++i)
	{
		ScalarType temp = computerProb(alpha,i,reflectance);
		allProb.push_back(temp);
	}

	//ScalarType sum_all = std::accumulate(allProb.begin(),allProb.end() ,0.);

	return (* max_element(allProb.begin(),allProb.end() ) );

}


ScalarType StopMotion::minProb(IndexType alpha, ScalarType reflectance[3] )
{
	vector<ScalarType> allProb;

	for (IndexType i = 0; i < N_GAUSS; ++i)
	{
		ScalarType temp = computerProb(alpha,i,reflectance);
		allProb.push_back(temp);
	}

	return (* min_element(allProb.begin(),allProb.end() ) );
}
ScalarType StopMotion::smoothItem(IndexType aLabel, IndexType bLabel, Matrix34& rgbColor)
{
	if (aLabel == bLabel)
	{
		return 0.;
	}

	VecX aDiff = rgbColor.col(0) - rgbColor.col(2);

	VecX bDiff = rgbColor.col(1) - rgbColor.col(3);
	

	ScalarType aDis = aDiff.col(0).norm();

	ScalarType bDis = bDiff.col(0).norm();

	return 0.5* (aDis + bDis);
}

ScalarType StopMotion::smoothItemOri(IndexType aLabel, IndexType bLabel, 
							   Vec3& curPixel, Vec3& neigPixel)
{
	if (aLabel == bLabel)
	{
		return 0.;
	}

	ScalarType beta = m_beta * 20;

	ScalarType diff = (curPixel - neigPixel).norm();

	return 50 * exp(-beta *diff);
}

ScalarType StopMotion::smoothItemOri(IndexType aLabel, IndexType bLabel, 
									 ScalarType curPixel[3], ScalarType neigPixel[3])
{

	if (aLabel == bLabel)
	{
		return 0.;
	}

	ScalarType dis = 0.;
	
	dis = (curPixel[0]- neigPixel[0])*(curPixel[0]- neigPixel[0]) + (curPixel[1] - neigPixel[1])*(curPixel[1] - neigPixel[1]) +(curPixel[2] - neigPixel[2])*(curPixel[2] - neigPixel[2]);

	ScalarType beta = m_beta * 20;

	return 50 * exp(-beta * dis);

}

void StopMotion::getColorUsingPtr(Mat& inputImg, IndexType ptrAdd, Vec3& curColor)
{

  curColor(0,0) = inputImg.data[ptrAdd];

  curColor(1,0) = inputImg.data[ptrAdd + 1]; 

  curColor(2,0) = inputImg.data[ptrAdd + 2];

}

void StopMotion::setGraphCutWeight(ScalarType weight_)
{
	assert(weight_ > 0.);
	m_gWeight = weight_;
// 	if (weight_>0. && weight_ <1.)
// 	{
// 		m_gWeight = weight_;
// 	}else
// 	{
// 		m_gWeight = 0.5;
// 	}
}

void StopMotion::setTresChange(IndexType diff)
{
	if (diff > 0 && diff < 51)
	{
	   m_thresDiff = diff;
	}

}

bool StopMotion::isHandPixel(const Vec3b& oriColor)
{
	bool cdt_1,cdt_2,cdt_3;

	cdt_1 = false;

	cdt_2 = false;

	cdt_3 = false;

	if ( oriColor[0] > 95 && oriColor[1] > 40 && oriColor[2] > 20)
	{
		cdt_1 = true;
	}


	uchar maxRGB = max(oriColor[0], max(oriColor[1], oriColor[2]) );
	uchar minRGB = min(oriColor[0], min(oriColor[1], oriColor[2]) );

	if ( (maxRGB - minRGB) > 15)
	{
		cdt_2 = true;
	}

	if ( (oriColor[0] - oriColor[1]) > 15 && oriColor[0] > oriColor[1] && oriColor[0] > oriColor[2] )
	{
		cdt_3 = true;
	}

	if ( cdt_3 && cdt_2 && cdt_1)
	{
		return true;
	}else
	{
		return false;
	}
}

void StopMotion::setInteractiveStatue(bool isInteractive)
{
	m_isWithInteractive = isInteractive;
}

bool StopMotion::initImageLabelWithInteractive(MatrixXXi& initLabel_)
{
	if (initLabel_.cols() > 0 && initLabel_.rows() > 0)
	{
          m_initLabel = initLabel_;

		  return true;
	}else
	{
		return false;
	}

}
ScalarType StopMotion::calculateProb(IndexType nDim,MatrixXX& mean_,MatrixXX& cov,MatrixXX& sValue)
{
	MatrixXX covInve;
	MatrixXX IdMat;
	IdMat.setIdentity(nDim,nDim);
	covInve.setZero(nDim,nDim);
	Eigen::HouseholderQR<MatrixXX> qr(cov);
	covInve = qr.solve(IdMat);
	MatrixXX temp = (sValue - mean_) * covInve * (sValue - mean_).transpose();
	ScalarType mo = temp(0,0);

	ScalarType scl = pow( (2*PI),1.5) * pow(cov.determinant(), 0.5 );

	ScalarType pro = exp(-mo *(0.5));

	//pro /= scl;
	
	return pro;
}

void StopMotion::motionToColor(Mat flow, Mat& color)
{
	if (color.empty())  
		color.create(flow.rows, flow.cols, CV_8UC3);  

	static vector<Scalar> colorwheel; //Scalar r,g,b  
	if (colorwheel.empty())  
		makecolorwheel(colorwheel);  

	// determine motion range:  
	float maxrad = -1;  

	// Find max flow to normalize fx and fy  
	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
				continue;  
			float rad = sqrt(fx * fx + fy * fy);  
			maxrad = maxrad > rad ? maxrad : rad;  
		}  
	}  

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  

			float fx = flow_at_point[0] / maxrad;  
			float fy = flow_at_point[1] / maxrad;  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
			{  
				data[0] = data[1] = data[2] = 0;  
				continue;  
			}  
			float rad = sqrt(fx * fx + fy * fy);  

			float angle = atan2(-fy, -fx) / CV_PI;  
			float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
			int k0 = (int)fk;  
			int k1 = (k0 + 1) % colorwheel.size();  
			float f = fk - k0;  
			//f = 0; // uncomment to see original color wheel  

			for (int b = 0; b < 3; b++)   
			{  
				float col0 = colorwheel[k0][b] / 255.0;  
				float col1 = colorwheel[k1][b] / 255.0;  
				float col = (1 - f) * col0 + f * col1;  
				if (rad <= 1)  
					col = 1 - rad * (1 - col); // increase saturation with radius  
				else  
					col *= .75; // out of range  
				data[2 - b] = (int)(255.0 * col);  
			}  
		}  
	}  
}

void StopMotion::makecolorwheel(vector<Scalar> &colorwheel)
{
	int RY = 15;  
	int YG = 6;  
	int GC = 4;  
	int CB = 11;  
	int BM = 13;  
	int MR = 6;  

	int i;  

	for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
	for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
	for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
	for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
	for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
	for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR)); 
}

void StopMotion::setShowPairId(IndexType* curIdx)
{
	for (IndexType i = 0; i < 2; ++ i)
	{
		m_showPairIdx[i] = curIdx[i];
	}

	//Loggger << m_showPairIdx[0] << " " << m_showPairIdx[1] << endl;

}

void StopMotion::rgb2YUV(Vec3b& rgb_, Vec3b& yuv_)
{
	Matrix33 trans;

	trans(0,0) = 0.299; trans(0,1) = 0.587; trans(0,2) = 0.114; 
	trans(1,0) = -0.147; trans(1,1) = -0.289; trans(1,2) = 0.436; 
	trans(2,0) = 0.615; trans(2,1) = -0.515; trans(2,2) = -0.100;  

	Vec3 ship;
	ship(0,0) = 0;
	ship(1,0) = 128;
	ship(2,0) = 128;

	Vec3 ori;
	ori(0,0) = rgb_(0);
	ori(1,0) = rgb_(1);
	ori(2,0) = rgb_(2);

	Vec3 res;

	res = trans * ori + ship;

	yuv_(0) = res(0,0);
	yuv_(1) = res(1,0);
	yuv_(2) = res(2,0);

}

bool StopMotion::isSkinPixel(Vec3b& pRGB)
{
	Vec3b yuv;

	rgb2YUV(pRGB,yuv);

	bool rgbColorS = false;
	bool yuvColorS = false;

	if (yuv(1) > 80 && yuv(1) < 130 && yuv(2) > 136 && yuv(2) < 200 && yuv(2) > yuv(1) )
	{
		yuvColorS = true;
	}

	if (pRGB(0) > 80 && pRGB(1) > 30 && pRGB(2) > 15)
	{
		IndexType rg = abs(pRGB(0) - pRGB(1) );

		if (rg> 15)
		{
			rgbColorS = true;
		}
	}

	if ( rgbColorS && yuvColorS)
	{
		return true;

	}else
	{
		return false;
	}

}

void StopMotion::handMarkerRGBYUV(const Mat& srImg, MatrixXXi& handMK)
{
	IndexType hight = srImg.rows;

	IndexType width = srImg.cols;

	handMK.setZero(hight, width);

	Mat initMark,temp;
	initMark.create(hight,width,CV_8UC1);

	for (int y = 0; y < hight; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			Vec3b clr = srImg.at<cv::Vec3b>(y,x);

			bool isH = isSkinPixel(clr);

			if (isH)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
			else
			{
				initMark.at<uchar>(y,x) = 255;//white
			}

		}
	}


	//imshow("YUV-Markers",initMark);

	//closing
	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(10,10));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(initMark, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image

// 	imshow("closing",filterMark);
// 	imshow("Edges",edgeTemp);

	MatrixXXi unLabels;

	findUnknowns(filterMark,edgeTemp,unLabels,25);

	Mat finalLabels;
	finalLabels.create(hight,width,CV_8UC3);

	for (IndexType hId = 0; hId < hight; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType gVal = filterMark.at<uchar>(hId,wId);

			IndexType isUnknown = unLabels(hId,wId);

			if (isUnknown)
			{
				handMK(hId,wId) = 2;
				finalLabels.at<Vec3b>(hId,wId) = ukMask;

			}else
			{
				if (gVal == 255)
				{
					handMK(hId,wId) = 0; 
					//handMK(hId,wId) = 2; //for automatic method
					finalLabels.at<Vec3b>(hId,wId) = bgMask;

				}else if(gVal == 0)
				{
					handMK(hId,wId) = 1;
					finalLabels.at<Vec3b>(hId,wId) = fgMask;
				}
			}


		}
	}


	Mat labelTp;
	cvtColor(finalLabels,labelTp,CV_RGB2BGR);
	imshow("inital-labels",labelTp);

}


void StopMotion::itaHandMarkers(const Mat& srImg, MatrixXXi& handMK)
{
	IndexType hight = srImg.rows;

	IndexType width = srImg.cols;

	handMK.resize(hight, width);

	for (IndexType hId = 0; hId < hight; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			handMK(hId,wId) = 2;
		}
	}

}
void StopMotion::getinitTrainData(const Mat& srImg, MatrixXXi& iniTrainLab, MatrixXXi& stillPixel)
{
	IndexType hight = srImg.rows;

	IndexType width = srImg.cols;

	iniTrainLab.setZero(hight, width);
	stillPixel.setZero(hight,width);

	Mat initMark,temp;
	initMark.create(hight,width,CV_8UC1);

	for (int y = 0; y < hight; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			Vec3b clr = srImg.at<cv::Vec3b>(y,x);

			bool isH = isSkinPixel(clr);
			//bool isH = isHandPixel(clr);

			if (isH)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
			else
			{
				initMark.at<uchar>(y,x) = 255;//white
			}

		}
	}


	imshow("YUV-Markers",initMark);

	//closing
	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(6,6));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(initMark, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image

	MatrixXXi unLabels, larUnlabels;

	findUnknowns(filterMark,edgeTemp,unLabels,30);

	findUnknowns(filterMark,edgeTemp,larUnlabels,60);

	MatrixXXi sureBackground;

	findSureBackground(filterMark,unLabels,larUnlabels,sureBackground); //XOR operation

	findStillPixel(filterMark,larUnlabels,stillPixel);

	Mat finalLabels;
	finalLabels.create(hight,width,CV_8UC3);

	for (IndexType hId = 0; hId < hight; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType gVal = filterMark.at<uchar>(hId,wId);

			IndexType isUnknown = unLabels(hId,wId); //small unknown labels

			IndexType isSureBackg = sureBackground(hId,wId);

			if (gVal == 0)
			{
				iniTrainLab(hId,wId) = 1;
				finalLabels.at<Vec3b>(hId,wId) = fgMask;
			}else
			{
				if (isSureBackg)
				{
					iniTrainLab(hId,wId) = 0;
					finalLabels.at<Vec3b>(hId,wId) = bgMask;
				}else
				{
					iniTrainLab(hId,wId) = 2;
					finalLabels.at<Vec3b>(hId,wId) = ukMask;
				}
			}

		}
	}

	Mat labelTp;
	cvtColor(finalLabels,labelTp,CV_RGB2BGR);
	imshow("inital-labels",labelTp);

}

void StopMotion::findUnknowns(Mat& initHandMarker, Mat& edgeMarker, MatrixXXi& unknownLabels, IndexType bandwidth)
{
	IndexType height = initHandMarker.rows;
	IndexType width = initHandMarker.cols;

	unknownLabels.setZero(height,width);

	assert(height > 0 && width > 0);

	//IndexType extSize = 25;

	Mat dilImage;

	Mat edgeExStruct = getStructuringElement(MORPH_RECT,Size(bandwidth,bandwidth));

	morphologyEx(edgeMarker, dilImage, CV_MOP_DILATE, edgeExStruct);


	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType isEdge = dilImage.at<uchar>(hId,wId);//0 or 255

			IndexType oriGray = initHandMarker.at<uchar>(hId,wId);

			if (isEdge && oriGray) // both of them with white color
			{
				unknownLabels(hId,wId) = 1;

			}else
			{
				continue;
			}
		}
	}


}


void StopMotion::findSureBackground(Mat& filterImg, MatrixXXi& smallUnknown,
									MatrixXXi& largeUnknown, MatrixXXi& sureBackg)
{
	IndexType height = filterImg.rows;
	IndexType width = filterImg.cols;

	sureBackg.setZero(height,width);

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType xorV = smallUnknown(hId,wId) ^ largeUnknown(hId,wId);
			IndexType gVal = filterImg.at<uchar>(hId,wId);

			if (xorV && gVal)
			{
				sureBackg(hId,wId) = 1;
			}
		}
	}
	
}

void StopMotion::findStillPixel(Mat& oriImg, MatrixXXi& largeUnknown, MatrixXXi& stillPixel)
{
	IndexType height = oriImg.rows;
	IndexType width = oriImg.cols;

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			IndexType gVal = oriImg.at<uchar>(hId,wId); //white
			IndexType isUk = largeUnknown(hId,wId);

			if (gVal && (!isUk))
			{
				stillPixel(hId,wId) = 1;
			}
		}
	}


}


void StopMotion::gcSegmentation(Mat& oriImg, MatrixXXi& initLabels, MatrixXXi& resLabels)
{
// 	IndexType iniLabelSize = 2;
// 	IndexType hight = oriImg.rows;
// 	IndexType width = oriImg.cols;
// 
// 	m_gcImg = new GCoptimizationGridGraph(width,hight,iniLabelSize);

	//m_gcImg->setDataCost()
}

void StopMotion::findLabelSeam(MatrixXXi& labels, Mat& seam)
{
	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	Mat initMark,temp;
	initMark.create(rows,cols,CV_8UC1);

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{

			IndexType isH = labels(y,x);

			if (isH)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
			else
			{
				initMark.at<uchar>(y,x) = 255;//white
			}

		}
	}

	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(10,10));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(initMark, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image                                                        

	imshow("Seam",edgeTemp);

}

void StopMotion::findSeamPosList(MatrixXXi& labels, vector<Point2f>& seam)
{
	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	//no expand
// 	Mat initMark,temp;
// 	initMark.create(rows,cols,CV_8UC1);
// 
// 	//continue to expand a large region?
// 
// 	for (int y = 0; y < rows; ++y)
// 	{
// 		for (int x = 0; x < cols; ++x)
// 		{
// 
// 			IndexType isH = labels(y,x);
// 
// 			if (isH)
// 			{
// 				initMark.at<uchar>(y,x) = 0;// black
// 			}
// 			else
// 			{
// 				initMark.at<uchar>(y,x) = 255;//white
// 			}
// 
// 		}
// 	}
// 
//     spClassify.findSeamPosition(initMark,seam);

	//do expand
	Mat expLabGray;

	IndexType band = 15;

	expandSeam(labels,band,expLabGray);
	//
	spClassify.findSeamPosition(expLabGray,seam);

// 	IndexType pSize = seam.size();
// 	//test detect pos
// 	//Mat seamShow = Mat::ones(2,pSize,CV_8UC3);
// 
// 	Mat seamShow;
// 	seamShow.create(rows,cols,CV_8UC1);
// 	seamShow.setTo(0);
// 
// 	for (IndexType i = 0; i < seam.size(); ++ i)
// 	{
// 		Point2f tp = seam[i];
// 		seamShow.at<uchar>(tp.y,tp.x) = 255;
// 	}
// 
// 	imshow("Test for position", seamShow);

}


void StopMotion::findSeamExpand(MatrixXXi& labels, vector<Point2f>& seam,MatrixXXi& resLabels)
{
	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	//do expand
	Mat expLabGray;

	IndexType band = 15;

	expandSeamExpand(labels,band,expLabGray,resLabels);

	spClassify.findSeamPosition(expLabGray,seam);

// 	IndexType pSize = seam.size();
// 
// 	Mat seamShow;
// 	seamShow.create(rows,cols,CV_8UC1);
// 	seamShow.setTo(0);
// 
// 	for (IndexType i = 0; i < seam.size(); ++ i)
// 	{
// 		Point2f tp = seam[i];
// 		seamShow.at<uchar>(tp.y,tp.x) = 255;
// 	}
// 
// 	imshow("Test for position", seamShow);

}

void StopMotion::matchingFeaturePs(Mat& leftImg, Mat& rightImg, 
								   vector<Point2f>& boundary, 
								   vector<Point>& lMPixels,
								   vector<Point>& rMPixels)
{

	//get the gradient value along the boundary

	vector<ScalarType> lGradBoundary,rGradBoundary;
	getBoundaryGradient(leftImg,rightImg,boundary,lGradBoundary,rGradBoundary);

	//draw the curves
	m_autoSelect = new AutoSelect(0, 1e4);
	m_autoSelect->setBand(10);
	m_autoSelect->smoothCurves(lGradBoundary,5);
	m_autoSelect->smoothCurves(rGradBoundary,5);

	//downsample: have two choices ,threshold or local maximize value
	vector<ScalarType> ltrunG,rtrunG;
	vector<IndexType> ltrunIdx,rtrunIdx;

	//truncate with a threshold, using relatively value, down-sample with threshold.
// 	ScalarType graThres = 0.7;
// 	m_autoSelect->truncationCurves(lGradBoundary,graThres,ltrunG,ltrunIdx);
// 	m_autoSelect->truncationCurves(rGradBoundary,graThres,rtrunG,rtrunIdx);
	//draw the curves of gradient
	//spClassify.drawVectorByMatlab(ltrunG,0);// draw truncate gradient value
	//spClassify.drawVectorByMatlab(rtrunG,0);

	//another way! downsample using local max value pixel

	vector<bool> isLocalL, isLocalR;
	vector<IndexType> recordL,recordR;
	m_autoSelect->detectMinMaxint(lGradBoundary,10,recordL);
	m_autoSelect->detectMinMaxint(rGradBoundary,10,recordR);

	for (int i = 0; i < recordL.size(); ++ i)
	{
		if (recordL[i] == 1 /*|| recordL[i] == 0*/)
		{
			isLocalL.push_back(true);
			ltrunG.push_back(lGradBoundary[i]);
			ltrunIdx.push_back(i);
		}
		else
		{
			isLocalL.push_back(false);
		}
	}

	for (int i = 0; i < recordR.size(); ++ i)
	{
		if (recordR[i] == 1 /*|| recordR[i] == 0*/)
		{
			isLocalR.push_back(true);
			rtrunG.push_back(rGradBoundary[i]);
			rtrunIdx.push_back(i);
		}
		else
		{
			isLocalR.push_back(false);
		}
	}

	spClassify.drawVectorMinMaxByMatlab(lGradBoundary,0,isLocalL);
	spClassify.drawVectorMinMaxByMatlab(rGradBoundary,0,isLocalR);

	//down-sample the pixel from the smoothed data,using the local maximal position!
	vector<ScalarType> lDSMarkGrad,rDSMarkGrad;
	vector<IndexType> lDSIdx,rDSIdx;


	//down-sample again for dynamic matching. small-large.

	//bool usLocalMax = true;
	bool usLocalMax = false;

	if (usLocalMax)
	{
		downSampleCandidates(ltrunG,ltrunIdx,rtrunG,rtrunIdx,
			lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx);
	}else
	{
		downSampleLargeCandidates(ltrunG,ltrunIdx,rtrunG,rtrunIdx,
			lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx);
	}

	//draw down-sample max pixel
	drawMaxPs(leftImg,rightImg,boundary,lDSIdx,rDSIdx);

	//draw original
	//drawMaxPs(leftImg,rightImg,boundary,ltrunIdx,rtrunIdx);

	//!!!calculate the distance matrix between two markable pixel sets
	vector<vector<ScalarType> > pix_distM;
	

	//with down-sample
	calculateDisMatrix(leftImg,rightImg,boundary,
		lDSMarkGrad,lDSIdx,rDSMarkGrad,rDSIdx, pix_distM);


	//left size must larger than right size

	dynamicProMatching(pix_distM,boundary,lDSIdx,rDSIdx,
		               lMPixels,rMPixels);

	//for visualization of curves
// 	Mat lSelectPix,rSelectPix;
// 	lSelectPix = Mat(leftImg.size(),leftImg.type(),1);
// 	rSelectPix = Mat(leftImg.size(),leftImg.type(),1);
// 
// 	for (IndexType i = 0; i < lDSIdx.size(); ++i)
// 	{
// 		Point feaPs = boundary[lDSIdx[i]];
// 		lSelectPix.at<Vec3b>(feaPs.y,feaPs.x) = leftImg.at<Vec3b>(feaPs.y,feaPs.x);
// 	}
// 
// 	for (IndexType i = 0; i < rtrunIdx.size(); ++i)
// 	{
// 		Point feaPs = boundary[rtrunIdx[i]];
// 		rSelectPix.at<Vec3b>(feaPs.y,feaPs.x) = rightImg.at<Vec3b>(feaPs.y,feaPs.x);
// 	}
// 
// 	Mat ltemp,rtemp;
// 	cvtColor(lSelectPix,ltemp,CV_RGB2BGR);
// 	cvtColor(rSelectPix,rtemp,CV_RGB2BGR);
// 
// 	imshow("LeftSelectBoundary",ltemp);
// 	imshow("RIghSelectBoundary",rtemp);


// 	spClassify.drawVectorByMatlab(lGradBoundary,0);
// 	spClassify.drawVectorByMatlab(rGradBoundary,0);
	//max -min
// 	vector<bool> isLocalL, isLocalR;
// 	vector<IndexType> recordL,recordR;
// 
// 	m_autoSelect->detectMinMaxint(ltrunG,10,recordL);
// 	m_autoSelect->detectMinMaxint(rtrunG,10,recordR);
// 
// 	for (int i = 0; i < recordL.size(); ++ i)
// 	{
// 		if (recordL[i] == 1 || recordL[i] == 0)
// 		{
// 			isLocalL.push_back(true);
// 		}
// 		else
// 		{
// 			isLocalL.push_back(false);
// 		}
// 	}
// 
// 	for (int i = 0; i < recordR.size(); ++ i)
// 	{
// 		if (recordR[i] == 1 || recordR[i] == 0)
// 		{
// 			isLocalR.push_back(true);
// 		}
// 		else
// 		{
// 			isLocalR.push_back(false);
// 		}
// 	}
// 
// 	spClassify.drawVectorMinMaxByMatlab(ltrunG,0,isLocalL);
// 	spClassify.drawVectorMinMaxByMatlab(rtrunG,0,isLocalR);


}


void StopMotion::matchingFeatureOverLapLee(Mat& leftImg, Mat& rightImg, 
										   vector<Point2f>& boundary, 
										   vector<Point>& lMPixels, 
										   vector<Point>& rMPixels,
										   cv::Rect& exRect)
{
	//matching feature points on overlap area out of hands regions, introduction by Prof. LI.

	//1- get the bbx of boundary

	Point minPs,maxPs;
	ScalarType diaLen = 0.;
	boundingboxSeam(boundary,minPs,maxPs,diaLen);


	//2-extend the boundary
	Point expMinPs,expMaxPs;

	bool isExpand = false;
	IndexType nSkip = 10;

	vector<bool> skipFlag;

	isExpand = expanROI(leftImg, minPs,maxPs,expMinPs,expMaxPs,nSkip,skipFlag);

	Rect dRoi(expMinPs,expMaxPs);

	Mat lExBbx,rExBbx;
	leftImg(dRoi).copyTo(lExBbx);
	rightImg(dRoi).copyTo(rExBbx);
	//3 detect the features from two images respectively

	//4 refine the features- delete the features which come from bbx

	//5-matching the features;

	//3-4-5 using only a function

	bool isMatching = false;

	Point updMinP,updMaxP;
	updMinP = minPs - expMinPs;
	updMaxP = maxPs - expMinPs;

	Rect oriRect(updMinP, updMaxP);	

	isMatching = mathcingExRoiBbx(lExBbx,rExBbx,oriRect,lMPixels,rMPixels);

	if (isMatching)
	{
		exRect = dRoi;
	}
}

bool StopMotion::mathcingExRoiBbx(Mat& lExBbx,Mat&rExBbx,cv::Rect& oriBbx,
								  vector<Point>& lMPixels,
								  vector<Point>& rMPixels)
{
	//detect features

	//refine
	//matching
	string detectorType = "SIFT";  
	string descriptorType = "SIFT";  
	string matcherType = "FlannBased";

	Ptr<FeatureDetector> featureDetector;  
	Ptr<DescriptorExtractor> descriptorExtractor;  
	Ptr<DescriptorMatcher> descriptorMatcher; 

	if( !createDetectorDescriptorMatcher( detectorType, descriptorType,
		matcherType, featureDetector, 
		descriptorExtractor,descriptorMatcher ) )  
	{  
		Loggger<<"Creat Detector Descriptor Matcher False!"<<endl;  
		return false;  
	} 

	//get the srcImg keypoints

	vector<KeyPoint> queryKeyPs, refQueryKP;
	vector<KeyPoint> trainKeyPs, refTrainKP;
	Mat queryDesc;
	Mat trainDesc;

	queryKeyPs.clear();
	trainKeyPs.clear();
	queryDesc.setTo(0);
	trainDesc.setTo(0);

	//left image
	featureDetector->detect(lExBbx, queryKeyPs);
	refineKeyPs2(oriBbx,queryKeyPs,refQueryKP);
    descriptorExtractor->compute(lExBbx,refQueryKP,queryDesc);

	//right image
	featureDetector->detect(rExBbx,trainKeyPs);
	refineKeyPs2(oriBbx,trainKeyPs,refTrainKP);
	descriptorExtractor->compute(rExBbx,refTrainKP,trainDesc);


	if (refQueryKP.size() <= 0 || refTrainKP.size() <= 0)
	{
		Loggger<<"No feature point located on overlap area!.";
		return false;
	}else
	{
		Mat lroi,rroi,ltemp,rtemp;
		lExBbx.copyTo(lroi);
		rExBbx.copyTo(rroi);

		Scalar color(125,120,120);
		for (IndexType i = 0; i < refQueryKP.size(); ++ i)
		{
			KeyPoint curkp = refQueryKP[i];
			Point ps = curkp.pt;
			circle(lroi,ps,5,color);
		}

		for (IndexType j = 0; j < refTrainKP.size(); ++ j)
		{
			KeyPoint curkp = refTrainKP[j];
			Point ps = curkp.pt;
			circle(rroi,ps,5,color);
		}

		cvtColor(lroi,ltemp,CV_RGB2BGR);
		cvtColor(rroi,rtemp,CV_RGB2BGR);

		imshow("lBBX_features",ltemp);
		imshow("rBBx-features",rtemp);
	}

	Mat homo;
	vector<DMatch> matchPs;
	bool isFound = matchingDescriptor(refQueryKP,refTrainKP,queryDesc,trainDesc,
		                              descriptorMatcher,homo, matchPs);

	vector<Point2f> srcMatch, desMatch;

	Loggger<<"Matches size = "<<matchPs.size()<<endl;

	for (IndexType i = 0; i < matchPs.size(); ++i)
	{
		Point2f pt1 = refQueryKP[matchPs[i].queryIdx].pt;
		Point2f pt2 = refTrainKP[matchPs[i].trainIdx].pt;

		lMPixels.push_back(pt1);
		rMPixels.push_back(pt2);
	}

	Mat drawIMg, drawTemp;

	drawMatches(lExBbx, refQueryKP,rExBbx,refTrainKP, matchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("matches", drawTemp);

	return true;
}

int StopMotion::getMatchingPoints(Mat& srImg, Mat& tgImg, 
								  vector<CvPoint2D32f>& srPoints, vector<CvPoint2D32f>& tgPoints)
{
	srPoints.clear();
	tgPoints.clear();

	string detectorType = "SIFT";  
	string descriptorType = "SIFT";  
	string matcherType = "FlannBased";

	Ptr<FeatureDetector> featureDetector;  
	Ptr<DescriptorExtractor> descriptorExtractor;  
	Ptr<DescriptorMatcher> descriptorMatcher; 

	if( !createDetectorDescriptorMatcher( detectorType, descriptorType,
		matcherType, featureDetector, 
		descriptorExtractor,descriptorMatcher ) )  
	{  
		Loggger<<"Creat Detector Descriptor Matcher False!"<<endl;  
		return 0;  
	} 

	//get the srcImg keypoints

	vector<KeyPoint> queryKeyPs, refQueryKP;
	vector<KeyPoint> trainKeyPs, refTrainKP;
	Mat queryDesc;
	Mat trainDesc;

	queryKeyPs.clear();
	trainKeyPs.clear();
	queryDesc.setTo(0);
	trainDesc.setTo(0);

	featureDetector->detect(srImg, queryKeyPs);

	refineKeyps(srImg,queryKeyPs,refQueryKP);

	descriptorExtractor->compute(srImg,refQueryKP,queryDesc);


	featureDetector->detect(tgImg,trainKeyPs);

	refineKeyps(tgImg,trainKeyPs,refTrainKP);

	descriptorExtractor->compute(tgImg,refTrainKP,trainDesc);

 	Mat homo;

	vector<DMatch> matchPs;

	bool isFound = matchingDescriptor(refQueryKP,refTrainKP,queryDesc,trainDesc,descriptorMatcher,homo, matchPs);

	vector<Point2f> srcMatch, desMatch;

	Loggger<<"Matches size = "<<matchPs.size()<<endl;

	for (IndexType i = 0; i < matchPs.size(); ++i)
	{

		CvPoint2D32f pt1 = refQueryKP[matchPs[i].queryIdx].pt;
		CvPoint2D32f pt2 = refTrainKP[matchPs[i].trainIdx].pt;


		ScalarType dis = sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x)  + (pt1.y - pt2.y)*(pt1.y - pt2.y)) ;

		if (dis > 15.)//exist a large disparity 
		{
			continue;
		}

		srPoints.push_back(pt1);
		tgPoints.push_back(pt2);

	}


	//initial matching points
	Mat oridrawIMg, oridrawTemp;

	drawMatches(srImg, refQueryKP,tgImg,refTrainKP, matchPs, oridrawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(oridrawIMg,oridrawTemp,CV_RGB2BGR);

	imshow("Ori GLobal Matches POints", oridrawTemp);


	vector<DMatch> agmatchPs;
	vector<KeyPoint> lKeyPs,rKeyPos;

	for (int i = 0; i< srPoints.size(); ++ i)
	{
		DMatch tp;
		tp.queryIdx = i;
		tp.trainIdx = i;
		agmatchPs.push_back(tp);

		lKeyPs.push_back(KeyPoint(srPoints[i],1.f));
		rKeyPos.push_back(KeyPoint(tgPoints[i],1.f));

	}

	Mat drawIMg, drawTemp;

	drawMatches(srImg, lKeyPs,tgImg,rKeyPos, agmatchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("GLobal Matches POints", drawTemp);

	return srPoints.size();
}

void StopMotion::refineKeyPs2(Rect& oriRect,vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs)
{
	outKeyPs.clear();

	for (IndexType i = 0; i < oriKeyPs.size(); ++ i)
	{
		KeyPoint curK = oriKeyPs[i];

		bool isInRect = isKPInBbx(curK,oriRect);

		if (!isInRect)
		{
			outKeyPs.push_back(curK);
		}
	}
}

bool StopMotion::isKPInBbx(KeyPoint& kp,Rect& oriRect)
{
	IndexType minx,miny,maxx,maxy;
	minx = oriRect.tl().x;
	miny = oriRect.tl().y;

	maxx = oriRect.br().x;
	maxy = oriRect.br().y;

	IndexType kpx,kpy;
	kpx = kp.pt.x;
	kpy = kp.pt.y;

	if (kpx >= minx && kpx <= maxx && kpy >= miny && kpy <= maxy)
	{
		return true;
	}else
	{
		return false;
	}
}

void StopMotion::downSampleCandidates(vector<ScalarType>& lOriGradVal,vector<IndexType>& lOriIdx,
									  vector<ScalarType>& rOriGradVal,vector<IndexType>& rOriIdx, 
									  vector<ScalarType>& ldownGradVal,vector<IndexType>& ldownIdx,
									  vector<ScalarType>& rdownGradVal,vector<IndexType>& rdownIdx)
{
	//how to down-sample the pixel?
	// sample the local maxmimal position first;

	vector<bool> isLocalL, isLocalR;
	vector<IndexType> recordL,recordR;

	m_autoSelect->detectMinMaxint(lOriGradVal,4,recordL);
	m_autoSelect->detectMinMaxint(rOriGradVal,4,recordR);


	vector<IndexType> coorIdx;
	IndexType nDownSize = 0;

	for (int i = 0; i < recordL.size(); ++ i)
	{

		if (recordL[i] == 1)
		{
			isLocalL.push_back(true);
			ldownGradVal.push_back(lOriGradVal[i]);
			ldownIdx.push_back(lOriIdx[i]);
		    coorIdx.push_back(nDownSize);
			++ nDownSize;
		}
		else
		{
			isLocalL.push_back(false);
		}
	}

	for (int i = 0; i < recordR.size(); ++ i)
	{
		if (recordR[i] == 1)
		{
			isLocalR.push_back(true);
			rdownGradVal.push_back(rOriGradVal[i]);
			rdownIdx.push_back(rOriIdx[i]);
		}
		else
		{
			isLocalR.push_back(false);
		}
	}



	if (ldownGradVal.size() > rdownGradVal.size() )
	{

		IndexType gap = ldownGradVal.size() - rdownGradVal.size();

		vector<ScalarType> temp = ldownGradVal;

		bubleSort(temp,coorIdx,ldownGradVal.size() );

		//delete the min element
		for (int i = 0; i < gap; ++ i)
		{
			ldownGradVal.erase(ldownGradVal.begin() + coorIdx[i]);
			ldownIdx.erase(ldownIdx.begin() + coorIdx[i]);

			for (int j = i; j < coorIdx.size(); ++ j)
			{
				if (coorIdx[j] > coorIdx[i])
				{
			    	coorIdx[j] = coorIdx[j] - 1;
				}

			}
		}

	}

	//draw the min-max position

	spClassify.drawVectorMinMaxByMatlab(lOriGradVal,0,isLocalL);
	spClassify.drawVectorMinMaxByMatlab(rOriGradVal,0,isLocalR);
	//end for drawing


}

void StopMotion::downSampleLargeCandidates(vector<ScalarType>& lOriGradVal,vector<IndexType>& lOriIdx,
										   vector<ScalarType>& rOriGradVal,vector<IndexType>& rOriIdx,
										   vector<ScalarType>& ldownGradVal,vector<IndexType>& ldownIdx,
										   vector<ScalarType>& rdownGradVal,vector<IndexType>& rdownIdx)
{
	ldownGradVal = lOriGradVal; ldownIdx = lOriIdx;
	rdownGradVal = rOriGradVal; rdownIdx = rOriIdx;

	if (ldownGradVal.size() > rdownGradVal.size() )
	{
		vector<IndexType> coorIdx;

		for (IndexType i = 0; i < ldownIdx.size(); ++ i)
		{
			coorIdx.push_back(i);
		}

		IndexType gap = ldownGradVal.size() - rdownGradVal.size();

		vector<ScalarType> temp = ldownGradVal;

		bubleSort(temp,coorIdx,ldownGradVal.size() );

		//delete the first gap min element successively.
		for (int i = 0; i < gap; ++ i)
		{
			ldownGradVal.erase(ldownGradVal.begin() + coorIdx[i]);
			ldownIdx.erase(ldownIdx.begin() + coorIdx[i]);

			for (int j = i; j < coorIdx.size(); ++ j)
			{
				if (coorIdx[j] > coorIdx[i])
				{
					coorIdx[j] = coorIdx[j] - 1;
				}

			}
		}

	}

	spClassify.drawVectorByMatlab(ldownGradVal,0);
	spClassify.drawVectorByMatlab(rdownGradVal,0);

}
void StopMotion::drawMaxPs(Mat& lImg,Mat& rImg, vector<Point2f>& boundnary, 
						   vector<IndexType>& lMaxIdx,vector<IndexType>& rMaxIdx)
{
	Mat ltpImg,rtpImg;
	lImg.copyTo(ltpImg);
	rImg.copyTo(rtpImg);

	Scalar color(125,120,120);
	for (int i = 0; i < lMaxIdx.size(); ++ i )
	{
		Point ps = boundnary[lMaxIdx[i]];
		circle(ltpImg,ps,5,color);
	}

	for (int i = 0; i < rMaxIdx.size(); ++ i)
	{
		Point ps = boundnary[rMaxIdx[i]];
		circle(rtpImg,ps,5,color);
	}

	Mat slImg,srimg;
	cvtColor(ltpImg,slImg,CV_RGB2BGR);
	cvtColor(rtpImg,srimg,CV_RGB2BGR);
	imshow("LMax Pos",slImg);
	imshow("Rmax Pos",srimg);

}


void StopMotion::drawMaxPs2(Mat& lImg,Mat& rImg, 
							vector<Point2f>& lboundnary, vector<Point2f>& rboundnary, 
							vector<IndexType>& lMaxIdx,vector<IndexType>& rMaxIdx)
{
	Mat ltpImg,rtpImg;
	lImg.copyTo(ltpImg);
	rImg.copyTo(rtpImg);

	Scalar color(125,120,120);
	for (int i = 0; i < lMaxIdx.size(); ++ i )
	{
		Point ps = lboundnary[lMaxIdx[i]];
		circle(ltpImg,ps,5,color);
	}

	for (int i = 0; i < rMaxIdx.size(); ++ i)
	{
		Point ps = rboundnary[rMaxIdx[i]];
		circle(rtpImg,ps,5,color);
	}

	Mat slImg,srimg;
	cvtColor(ltpImg,slImg,CV_RGB2BGR);
	cvtColor(rtpImg,srimg,CV_RGB2BGR);
	imshow("LMax Pos",slImg);
	imshow("Rmax Pos",srimg);

}


void StopMotion::bubleSort(vector<ScalarType>& oriData,vector<IndexType>& labels,IndexType lSize)
{
	ScalarType temp;
	IndexType labelTemp = 0;
	bool flag=false;
	for (int i=0;i<lSize;i++)
	{
		flag=true;
		for (int j=0;j<lSize-i-1;j++)
		{
			if(oriData[j]>oriData[j+1])
			{
				temp=oriData[j];
				labelTemp = labels[j];

				oriData[j]=oriData[j+1];
				labels[j] = labels[j+1];

				oriData[j+1]=temp;
				labels[j+1] = labelTemp;

				flag = false;
			}
		}
		if(flag) break;
	}
}

void StopMotion::getBoundaryGradient(Mat& leftImg, Mat rightImg,vector<Point2f>& boundary,
									 vector<ScalarType>& lGradBoundary, vector<ScalarType>& rGradBoundary)
{
	//along the boundary to find the remarkable features and matching.

	//calculate the gradient

	Mat leftTemp,rightTemp;

	GaussianBlur( leftImg, leftTemp, Size(3,3), 0, 0, BORDER_DEFAULT );

	GaussianBlur( rightImg, rightTemp, Size(3,3), 0, 0, BORDER_DEFAULT );

	Mat lGray,rGray;

	cvtColor(leftTemp, lGray,CV_RGB2GRAY);

	cvtColor(rightTemp, rGray,CV_RGB2GRAY);

	Mat lGradX,lGradY,rGradX,rGradY;

	Mat lGrad,rGrad,lDir,rDir,lNorm,rNorm;

	double lnormMax,rnormMax;

	Sobel(lGray,lGradX,CV_32F,1,0,3);
	Sobel(lGray,lGradY,CV_32F,0,1,3);
	cartToPolar(lGradX,lGradY,lNorm,lDir);

	minMaxLoc(lNorm,NULL,&lnormMax);
	lNorm.convertTo(lGrad,CV_8UC1,255.0/lnormMax,0);

	Sobel(rGray,rGradX,CV_32F,1,0,3);
	Sobel(rGray,rGradY,CV_32F,0,1,3);
	cartToPolar(rGradX,rGradY,rNorm,rDir);

	minMaxLoc(rNorm,NULL,&rnormMax);
	rNorm.convertTo(rGrad,CV_8UC1,255.0/rnormMax,0);

	for (IndexType i = 0; i < boundary.size(); ++ i)
	{
		Point curPos = boundary[i];

		ScalarType lVal  = lGrad.at<uchar>(curPos.y,curPos.x);
		lGrad.at<uchar>(curPos.y,curPos.x) = 255;
		ScalarType rVal = rGrad.at<uchar>(curPos.y,curPos.x);
		rGrad.at<uchar>(curPos.y,curPos.x) = 255;

		lGradBoundary.push_back(sqrt(lVal));
		rGradBoundary.push_back(sqrt(rVal));

	}

	imshow("seam-LeftGradient",lGrad);
	imshow("seam-RightGradient",rGrad);

}

void StopMotion::getBoundaryGradient2(Mat& leftImg, Mat rightImg, 
									  vector<Point2f>& lboundary,
									  vector<Point2f>& rboundary, 
									  vector<ScalarType>& lGradBoundary, 
									  vector<ScalarType>& rGradBoundary)
{
	//along the boundary to find the remarkable features and matching.

	//calculate the gradient

	Mat leftTemp,rightTemp;

	GaussianBlur( leftImg, leftTemp, Size(3,3), 0, 0, BORDER_DEFAULT );

	GaussianBlur( rightImg, rightTemp, Size(3,3), 0, 0, BORDER_DEFAULT );

	Mat lGray,rGray;

	cvtColor(leftTemp, lGray,CV_RGB2GRAY);

	cvtColor(rightTemp, rGray,CV_RGB2GRAY);

	Mat lGradX,lGradY,rGradX,rGradY;

	Mat lGrad,rGrad,lDir,rDir,lNorm,rNorm;

	double lnormMax,rnormMax;

	Sobel(lGray,lGradX,CV_32F,1,0,3);
	Sobel(lGray,lGradY,CV_32F,0,1,3);
	cartToPolar(lGradX,lGradY,lNorm,lDir);

	minMaxLoc(lNorm,NULL,&lnormMax);
	lNorm.convertTo(lGrad,CV_8UC1,255.0/lnormMax,0);

	Sobel(rGray,rGradX,CV_32F,1,0,3);
	Sobel(rGray,rGradY,CV_32F,0,1,3);
	cartToPolar(rGradX,rGradY,rNorm,rDir);

	minMaxLoc(rNorm,NULL,&rnormMax);
	rNorm.convertTo(rGrad,CV_8UC1,255.0/rnormMax,0);

	for (IndexType i = 0; i < lboundary.size(); ++ i)
	{
		Point curPos = lboundary[i];

		ScalarType lVal  = lGrad.at<uchar>(curPos.y,curPos.x);
		lGrad.at<uchar>(curPos.y,curPos.x) = 255;
		lGradBoundary.push_back(sqrt(lVal));

	}

	for (IndexType i = 0; i < rboundary.size(); ++ i)
	{
		Point curPos = rboundary[i];

		ScalarType rVal = rGrad.at<uchar>(curPos.y,curPos.x);
		rGrad.at<uchar>(curPos.y,curPos.x) = 255;
		rGradBoundary.push_back(sqrt(rVal));
	}

	imshow("seam-LeftGradient",lGrad);
	imshow("seam-RightGradient",rGrad);

}

void StopMotion::boundingboxSeam(vector<Point2f>& boundary, Point& minPos, Point& maxPos, ScalarType& diaLen)
{
	//bbx
	vector<int> xCoor,yCoor;
	xCoor.clear();yCoor.clear();

	for (IndexType i = 0; i < boundary.size(); ++ i )
	{
		Point2f tp = boundary[i];
		xCoor.push_back(tp.x);
		yCoor.push_back(tp.y); 
	}//the coordinate systems are different between opencv and matlab

	minPos.x = *(min_element(xCoor.begin(),xCoor.end()));
	minPos.y = *(min_element(yCoor.begin(),yCoor.end()));
	maxPos.x = *(max_element(xCoor.begin(),xCoor.end()));
	maxPos.y = *(max_element(yCoor.begin(),yCoor.end()));

	IndexType xdis = maxPos.x - minPos.x;
	IndexType ydis = maxPos.y - minPos.y;

	diaLen = sqrt(xdis*xdis + ydis*ydis);
}
void StopMotion::calculateDisMatrix(Mat& lImg, Mat& rImg, vector<Point2f>& boundary, 
									vector<ScalarType>& ldomainVal,vector<IndexType>& ldomainIdx, 
									vector<ScalarType>& rdomainVal,vector<IndexType>& rdomainIdx, 
									vector<vector<ScalarType> >& disM)
{

	Point minP,maxP;
	ScalarType diaLen;
	boundingboxSeam(boundary,minP,maxP,diaLen);// require the bbx of boundary!

    //calculate the distance between two pixels
	IndexType  lrows = ldomainIdx.size(); // the size of remarkable pixel in left image along boundary
	IndexType  rcols = rdomainIdx.size(); // the size of remarkable pixel in right image along boundary

	printf("The size of left = %d,right = %d.\n",lrows,rcols);

	vector<ScalarType> updateVal;
	vector<IndexType> updateIdx;

	//disM.resize(lrows);

	//using "SIFT" descriptor vector to calculate the distance between each pair of points;  
	MatrixXX descCostDis;

	descCostDistance(lImg,rImg,boundary,ldomainIdx,rdomainIdx,descCostDis);

	//

// 	char dist[1024];
// 	sprintf(dist,"distForDyna.txt");
// 	FILE* in_dist = fopen(dist,"w");

	for (IndexType i = 0; i < lrows; ++ i)
	{
		bool isCandidate = false;

		vector<ScalarType> rowDis;
		rowDis.clear();

		for (IndexType j = 0; j < rcols; ++ j)
		{
			//color distance and gradient distance && position constraint

			Point lPixPos = boundary[ldomainIdx[i]];
			Point rPixPos = boundary[rdomainIdx[j]];

			//Euler space
			IndexType xdis = lPixPos.x - rPixPos.x;
			IndexType ydis = lPixPos.y - rPixPos.y;
			ScalarType pixEDis = sqrt(xdis* xdis + ydis*ydis); // Euler distance
			ScalarType ratio = pixEDis/diaLen;

			// color space
// 			Vec3b lPsColor = lImg.at<Vec3b>(lPixPos.y,lPixPos.x);
// 			Vec3b rPsColor = rImg.at<Vec3b>(rPixPos.y,rPixPos.x);
// 
// 			Vec3b diffColor;
// 			diffColor[0] = abs(lPsColor[0]-rPsColor[0]);
// 			diffColor[1] = abs(lPsColor[1]-rPsColor[1]);
// 			diffColor[2] = abs(lPsColor[2]-rPsColor[2]);
// 			ScalarType colorDis = sqrt(diffColor[0]*diffColor[0] + diffColor[1]*diffColor[1]
// 			                           + diffColor[2]*diffColor[2]);

			// Gradient space
			ScalarType gradis = abs(ldomainVal[i] - rdomainVal[j]);// gradient distance

			//Index space : the idx of the vector
			IndexType absIdx = abs(i-j);

			//default value

			ScalarType pixsimi = INF;


			if (ratio < 0.5 /*&& gradis < 3.&& absIdx < 10  && colorDis < 50.*/)
			{
				/*pixsimi = pixEDis;*/
				//pixsimi = 10 * gradis;//gradient distance

				pixsimi = descCostDis(i,j);// desc--dis
				isCandidate = true;
			}
			rowDis.push_back(pixsimi);
			//disM[i].push_back(pixsimi);
			//fprintf(in_dist,"%f ",pixsimi);
		}

		//fprintf(in_dist,"\n");

		if (isCandidate)
		{
			disM.push_back(rowDis);
			updateVal.push_back(ldomainVal[i]);
			updateIdx.push_back(ldomainIdx[i]);

		}
		else
		{
			Loggger<<"No correspondence for this pixel,delete!.\n";
		}
	}

    //fclose(in_dist);
	//for updating the valid pixels

	ldomainVal = updateVal;
	ldomainIdx = updateIdx;

	updateVal.clear();
	updateIdx.clear();

}


void StopMotion::calculateDisMatrix2(Mat& lImg, Mat& rImg, 
									 vector<Point2f>& oriboundary,
									 vector<Point2f>& lboundary,vector<Point2f>& rboundary, 
									 vector<ScalarType>& ldomainVal,vector<IndexType>& ldomainIdx, 
									 vector<ScalarType>& rdomainVal,vector<IndexType>& rdomainIdx,
									 vector<vector<ScalarType> >& disM)
{

	Point minP,maxP;
	ScalarType diaLen;
	boundingboxSeam(oriboundary,minP,maxP,diaLen);// require the bbx of boundary!

	//calculate the distance between two pixels
	IndexType  lrows = ldomainIdx.size(); // the size of remarkable pixel in left image along boundary
	IndexType  rcols = rdomainIdx.size(); // the size of remarkable pixel in right image along boundary

	printf("The size of left = %d,right = %d.\n",lrows,rcols);

	vector<ScalarType> updateVal;
	vector<IndexType> updateIdx;


	//using "SIFT" descriptor vector to calculate the distance between each pair of points;  
	MatrixXX descCostDis;

	//descCostDistance(lImg,rImg,boundary,ldomainIdx,rdomainIdx,descCostDis);

	descCostDistance2(lImg,rImg,lboundary,rboundary,ldomainIdx,rdomainIdx,descCostDis);

	for (IndexType i = 0; i < lrows; ++ i)
	{
		bool isCandidate = false;

		vector<ScalarType> rowDis;
		rowDis.clear();

		//can only computer the distance between a range of the second contours 
		for (IndexType j = 0; j < rcols; ++ j)
		{
			//color distance and gradient distance && position constraint

			Point lPixPos = lboundary[ldomainIdx[i]];
			Point rPixPos = rboundary[rdomainIdx[j]];

			//Euler space
			IndexType xdis = lPixPos.x - rPixPos.x;
			IndexType ydis = lPixPos.y - rPixPos.y;
			ScalarType pixEDis = sqrt(xdis* xdis + ydis*ydis); // Euler distance
			ScalarType ratio = pixEDis/diaLen;

			// color space
			// 			Vec3b lPsColor = lImg.at<Vec3b>(lPixPos.y,lPixPos.x);
			// 			Vec3b rPsColor = rImg.at<Vec3b>(rPixPos.y,rPixPos.x);
			// 
			// 			Vec3b diffColor;
			// 			diffColor[0] = abs(lPsColor[0]-rPsColor[0]);
			// 			diffColor[1] = abs(lPsColor[1]-rPsColor[1]);
			// 			diffColor[2] = abs(lPsColor[2]-rPsColor[2]);
			// 			ScalarType colorDis = sqrt(diffColor[0]*diffColor[0] + diffColor[1]*diffColor[1]
			// 			                           + diffColor[2]*diffColor[2]);

			// Gradient space
			ScalarType gradis = abs(ldomainVal[i] - rdomainVal[j]);// gradient distance

			//Index space : the idx of the vector
			IndexType absIdx = abs(i-j);

			//default value

			ScalarType pixsimi = INF;


			if (ratio < 0.5 /*&& gradis < 3.&& absIdx < 10  && colorDis < 50.*/)
			{
				pixsimi = descCostDis(i,j);// desc--dis
				isCandidate = true;
			}
			rowDis.push_back(pixsimi);
		}


		if (isCandidate)
		{
			disM.push_back(rowDis);
			updateVal.push_back(ldomainVal[i]);
			updateIdx.push_back(ldomainIdx[i]);

		}
		else
		{
			Loggger<<"No correspondence for this pixel,delete!.\n";
		}
	}

	ldomainVal.clear();
	ldomainVal = updateVal;

	ldomainIdx.clear();
	ldomainIdx = updateIdx;

	updateVal.clear();
	updateIdx.clear();
}


void StopMotion::descCostDistance(Mat& lImg, Mat& rImg, 
								  vector<Point2f>& boundary, 
								  vector<IndexType>& ldomainIdx, vector<IndexType>& rdomainIdx,
								  MatrixXX& desCost)
{
	IndexType lsize = ldomainIdx.size();
	IndexType rsize = rdomainIdx.size();

	assert(lsize > 0 && rsize > 0);

	vector<KeyPoint> lkeyPs, rkeyPs;

	for (IndexType i = 0; i < lsize; ++ i)
	{
		Point2f tp;
		tp.x = boundary[ldomainIdx[i]].y;
		tp.y = boundary[ldomainIdx[i]].x;
		lkeyPs.push_back(KeyPoint(tp,1.f));		
	}

	for (IndexType j = 0; j < rsize; ++ j)
	{
		Point2f tp;
		tp.x = boundary[rdomainIdx[j]].y;
		tp.y = boundary[rdomainIdx[j]].x;
		rkeyPs.push_back(KeyPoint(tp,1.f));
	}

	Ptr<DescriptorExtractor> descriptorExtractor;  
	string detectorType = "SIFT";  
	descriptorExtractor = DescriptorExtractor::create( detectorType ); 


	Mat lDesc,rDesc;
	lDesc.setTo(0);
	rDesc.setTo(0);

	descriptorExtractor->compute(lImg,lkeyPs,lDesc);
	descriptorExtractor->compute(rImg,rkeyPs,rDesc);

	desCost.resize(lsize,rsize);

// 	char dist[1024];
// 	sprintf(dist,"shape_similarity.txt");
// 	FILE* in_dist = fopen(dist,"w");

	for (IndexType i = 0; i < lsize; ++ i)
	{
		for (IndexType j = 0; j < rsize; ++ j)
		{
			Mat curDesc = lDesc.row(i) - rDesc.row(j);
			desCost(i,j) = norm(curDesc,NORM_L2);
	        //fprintf(in_dist,"%f  ",desCost(i,j));
		}
		//fprintf(in_dist,"\n");
	}

	//fclose(in_dist);
}


void StopMotion::descCostDistance2(Mat& lImg, Mat& rImg, 
								   vector<Point2f>& lboundary, vector<Point2f>& rboundary,
								   vector<IndexType>& ldomainIdx, vector<IndexType>& rdomainIdx,
								   MatrixXX& desCost)
{
	IndexType lsize = ldomainIdx.size();
	IndexType rsize = rdomainIdx.size();

	assert(lsize > 0 && rsize > 0);

	vector<KeyPoint> lkeyPs, rkeyPs;

	for (IndexType i = 0; i < lsize; ++ i)
	{
		Point2f tp;
		tp.x = lboundary[ldomainIdx[i]].y;
		tp.y = lboundary[ldomainIdx[i]].x;
		lkeyPs.push_back(KeyPoint(tp,1.f));		
	}

	for (IndexType j = 0; j < rsize; ++ j)
	{
		Point2f tp;
		tp.x = rboundary[rdomainIdx[j]].y;
		tp.y = rboundary[rdomainIdx[j]].x;
		rkeyPs.push_back(KeyPoint(tp,1.f));
	}

	Ptr<DescriptorExtractor> descriptorExtractor;  
	string detectorType = "SIFT";  
	descriptorExtractor = DescriptorExtractor::create( detectorType ); 


	Mat lDesc,rDesc;
	lDesc.setTo(0);
	rDesc.setTo(0);

	descriptorExtractor->compute(lImg,lkeyPs,lDesc);
	descriptorExtractor->compute(rImg,rkeyPs,rDesc);

	desCost.resize(lsize,rsize);

	for (IndexType i = 0; i < lsize; ++ i)
	{
		for (IndexType j = 0; j < rsize; ++ j)
		{
			Mat curDesc = lDesc.row(i) - rDesc.row(j);
			desCost(i,j) = norm(curDesc,NORM_L2);
		}

	}

}

void StopMotion::dynamicProMatching(vector<vector<ScalarType> >& disM,vector<Point2f>& boundary, 
									vector<IndexType>& ldomainIdx,vector<IndexType>& rdomainIdx,
									vector<Point>& lMPixels,vector<Point>& rMPixels)
{

	//default left(row) --> right(cols)

	IndexType lrows = ldomainIdx.size();//disM.size();//
	IndexType rcols = rdomainIdx.size();//disM[0].size();//

	// as the same size of distance matrix
	assert( disM.size() == lrows && disM[0].size() == rcols);

// 	bool swp = false;
// 
	if (lrows > rcols)
	{
		Loggger<<"Should swap the direction.\n";
		return;
	}


	MatrixXX costMat;//record the cost for matching between pixels from the images
	costMat.resize(lrows + 1, rcols + 1);
	costMat.setConstant(INF);
	costMat.row(0).setConstant(0.);

	// 0   0  0   0   0  0 
	//INF  1  1   1   1 INF
	//INF INF 3   4   2 INF
	//INF INF INF 5   7  8
	//INF INF INF INF 9 INF

// 	char dist[1024];
// 	sprintf(dist,"costFunction.txt");
// 	FILE* costFunc = fopen(dist,"w");

	for (int i = 1; i < lrows + 1; ++ i)
	{
// 		for(int k = 0; k < i; ++k)
// 		{
//           fprintf(costFunc,"%f ", INF);
// 		}

		for (int j = i; j < rcols + 1; ++j)
		{
			vector<ScalarType> minE; 
			minE.clear();

			int stidx = i-1;
			int endidx = j;

			for (int k = stidx; k < endidx; ++ k)
			{
				ScalarType tp = 0.;

				tp = costMat(stidx,k) + *(min_element(disM[stidx].begin() + k, disM[stidx].begin() + endidx) );

				minE.push_back(tp);
			}

			costMat(i,j) = *(min_element(minE.begin(), minE.end() ) );
			//fprintf(costFunc,"%f ", costMat(i,j));
		}

		//fprintf(costFunc,"\n");
	}


	//fclose(costFunc);

// 	Loggger<<"Cost Matrix.\n";
// 	Loggger<<costMat<<endl;

	vector<IndexType> lMatchingIdx,rMatchingIdx;
	lMatchingIdx.clear();
	rMatchingIdx.clear();

	//reverse
 	int colsIdx = rcols;
 	for (int i = lrows; i > 0; -- i/*, -- colsIdx*/)
 	{	
 		MatrixXX temp = costMat.block(i,1,1,colsIdx);
 
 		IndexType pos = 1;
 		temp.row(0).minCoeff(&pos);
 		
		ScalarType maxV = temp.row(0).maxCoeff();
		if (maxV > INF)
		{
			printf("%d point no correspondence.\n",i);
			continue;
		}
 
 		if ( pos < i - 1)//cross happened!
 		{
 		    Loggger<<"Cross happened!\n";
 			//continue;
 		}
         lMatchingIdx.insert(lMatchingIdx.begin(),i - 1);
 		rMatchingIdx.insert(rMatchingIdx.begin(),pos);
		colsIdx = pos;
 	}

// 	for (IndexType i = 1; i <= lrows; ++ i)
// 	{
// 		MatrixXX temp = costMat.block(i,0,1,rcols + 1);
// 		IndexType pos = 1;
// 		temp.row(0).minCoeff(&pos);
// 
// 		printf("Min = %f.\n",temp(0,pos));
// 
// 		ScalarType maxV = temp.row(0).maxCoeff();
// 
// 		if (maxV > INF)
// 		{
// 			printf("%d point no correspondence.\n",i);
// 			continue;
// 		}
// 
// 		if ( pos < i )//cross happened!
// 		{
// 			Loggger<<"Cross happened!\n";
// 			//continue;
// 		}
// 		lMatchingIdx.push_back(i - 1);
// 		rMatchingIdx.push_back(pos - 1);
// 	}

	for (int i = 0; i < lMatchingIdx.size(); ++ i)
	{
		printf("left %d matching %d right.\n",lMatchingIdx[i],rMatchingIdx[i]);
		lMPixels.push_back(boundary[ldomainIdx[lMatchingIdx[i] ] ] );
		rMPixels.push_back(boundary[rdomainIdx[rMatchingIdx[i] ] ] );
	}

}

void StopMotion::dynamicProMatching2(vector<vector<ScalarType> >& disM, 
									 vector<Point2f>& lboundary,vector<Point2f>& rboundary, 
									 vector<IndexType>& ldomainIdx,vector<IndexType>& rdomainIdx, 
									 vector<Point>& lMPixels,vector<Point>& rMPixels)
{
	//default left(row) --> right(cols)

	IndexType lrows = ldomainIdx.size();//disM.size();//
	IndexType rcols = rdomainIdx.size();//disM[0].size();//

	// as the same size of distance matrix
	assert( disM.size() == lrows && disM[0].size() == rcols);

	if (lrows > rcols)
	{
		Loggger<<"Should swap the direction.\n";
		return;
	}


	MatrixXX costMat;//record the cost for matching between pixels from the images
	costMat.resize(lrows + 1, rcols + 1);
	costMat.setConstant(INF);
	costMat.row(0).setConstant(0.);

	// 0   0  0   0   0  0 
	//INF  1  1   1   1 INF
	//INF INF 3   4   2 INF
	//INF INF INF 5   7  8
	//INF INF INF INF 9 INF

	for (int i = 1; i < lrows + 1; ++ i)
	{
		for (int j = i; j < rcols + 1; ++j)
		{
			vector<ScalarType> minE; 
			minE.clear();

			int stidx = i-1;
			int endidx = j;

			for (int k = stidx; k < endidx; ++ k)
			{
				ScalarType tp = 0.;

				tp = costMat(stidx,k) + *(min_element(disM[stidx].begin() + k, disM[stidx].begin() + endidx) );

				minE.push_back(tp);
			}

			costMat(i,j) = *(min_element(minE.begin(), minE.end() ) );

		}

	}


	vector<IndexType> lMatchingIdx,rMatchingIdx;
	lMatchingIdx.clear();
	rMatchingIdx.clear();

	//reverse
	int colsIdx = rcols;
	for (int i = lrows; i > 0; -- i/*, -- colsIdx*/)
	{	
		MatrixXX temp = costMat.block(i,1,1,colsIdx);

		IndexType pos = 1;
		temp.row(0).minCoeff(&pos);

		ScalarType maxV = temp.row(0).maxCoeff();
		if (maxV > INF)
		{
			printf("%d point no correspondence.\n",i);
			continue;
		}

		if ( pos < i - 1)//cross happened!
		{
			Loggger<<"Cross happened!\n";
			//continue;
		}
		lMatchingIdx.insert(lMatchingIdx.begin(),i - 1);
		rMatchingIdx.insert(rMatchingIdx.begin(),pos);
		colsIdx = pos;
	}

	for (int i = 0; i < lMatchingIdx.size(); ++ i)
	{
		printf("left %d matching %d right.\n",lMatchingIdx[i],rMatchingIdx[i]);
		lMPixels.push_back(lboundary[ldomainIdx[lMatchingIdx[i] ] ] );
		rMPixels.push_back(rboundary[rdomainIdx[rMatchingIdx[i] ] ] );
	}

}

void StopMotion::calculateGradientSeam(Mat& leftImg, Mat& rightImg, vector<Point2f>& boundary, 
									   vector<ScalarType>& lGradSeam, vector<ScalarType>& rGradSeam)
{
	GaussianBlur( leftImg, leftImg, Size(3,3), 0, 0, BORDER_DEFAULT );

	GaussianBlur( rightImg, rightImg, Size(3,3), 0, 0, BORDER_DEFAULT );

	Mat lGray,rGray;

	cvtColor(leftImg, lGray,CV_RGB2GRAY);

	cvtColor(rightImg, rGray,CV_RGB2GRAY);

	Mat lGradX,lGradY,rGradX,rGradY;

	Mat absLGradX, absLGradY, absRGradX, absRGradY;

	Mat lGrad,rGrad;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	//for left image

	Sobel( lGray, lGradX, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );

	convertScaleAbs(lGradX,absLGradX);

	Sobel( lGray, lGradY, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

	convertScaleAbs(lGradY,absLGradY);

	addWeighted( absLGradX, 0.5, absLGradY, 0.5, 0, lGrad );

	// for right image
	Sobel( rGray, rGradX, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );

	convertScaleAbs(rGradX,absRGradX);

	Sobel( rGray, rGradY, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

	convertScaleAbs(rGradY,absRGradY);

	addWeighted( absRGradX, 0.5, absRGradY, 0.5, 0, rGrad );

	for (IndexType i = 0; i < boundary.size(); ++ i)
	{
		Point2f curPos = boundary[i];
		ScalarType lVal = lGrad.at<ushort>(curPos.x,curPos.y);
		ScalarType rVal = rGrad.at<ushort>(curPos.x,curPos.y);

		lGradSeam.push_back(lVal);
		rGradSeam.push_back(rVal);

	}

}

void StopMotion::showSeamPixels(Mat& leftImg, Mat& rightImg,vector<Point2f>& boundary)
{
	IndexType pSize = boundary.size();
	if (pSize <= 0)
	{
		return;
	}

	//IndexType pSize = seam.size();
	////test detect pos
	Mat seamShow = Mat::ones(20,pSize,CV_8UC3);

	for (IndexType i = 0; i < pSize; ++ i)
	{
		Point2f tp = boundary[i];
		seamShow.at<Vec3b>(10,i) = leftImg.at<Vec3b>(tp.y,tp.x);
		seamShow.at<Vec3b>(15,i) = rightImg.at<Vec3b>(tp.y,tp.x);
	}

	imshow("Boundary pixels", seamShow);

}

void StopMotion::findEdgeComponent(Mat& leftImg, Mat& rightImg, 
								   vector<vector<Point> >& leftcontours,
								   vector<vector<Point> >& rightcontours)
{

	Mat lBinary,rBinary;
	IndexType thres = 50;

	threshold(leftImg,lBinary,thres,255,THRESH_BINARY);
	threshold(rightImg,rBinary,thres,255,THRESH_BINARY);

	vector<vector<Point>> lconTemp,rconTemp;

	//vector<vector<Point> > leftcontours,rightcontours;
	vector<Vec4i> lhierarchy,rhierarchy;


	findContours(lBinary,lconTemp,lhierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(0,0) );
	findContours(rBinary,rconTemp,rhierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(0,0) );


// 	RNG rng(12345);
// 	/// Draw contours
// 	Mat drawing = Mat::zeros( leftImg.size(), CV_8UC3 );
// 	Mat drawingR = Mat::zeros( leftImg.size(), CV_8UC3 );

	for( int i = 0; i< lconTemp.size(); i++ )
	{
		//ScalarType ar = contourArea(lconTemp[i],false);

		if (lconTemp[i].size() > 10)
		{
			leftcontours.push_back(lconTemp[i]);
		}

// 		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
// 		drawContours( drawing, lconTemp, i, color, 2, 8, rhierarchy, 0, Point() );
	}

	for (int i = 0; i< rconTemp.size(); i++)
	{
		if (rconTemp[i].size() > 10)
		{
			rightcontours.push_back(rconTemp[i]);
		}
// 		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
// 		drawContours( drawingR, rconTemp, i, color, 2, 8, rhierarchy, 0, Point() );
	}

// 	imshow("ContoursLeft", drawing);
// 	imshow("ContoursRight", drawingR);
	//end for draw

	//Mat lxyedges,rxyedges;
	//lxyedges = Mat(leftImg.rows,leftImg.cols,CV_8UC1,10);
	//rxyedges = Mat(leftImg.rows,leftImg.cols,CV_8UC1,10);
	//vector<IndexType> xCoor,yCoor;
	//for (int i = 0; i < leftcontours.size(); ++ i)
	//{
	//	for (int j = 0; j < leftcontours[i].size(); ++j) 
	//	{
	//		Point tp = leftcontours[i][j];
	//		rxyedges.at<uchar>(tp.y,tp.x) = 100;
	//		xCoor.push_back(tp.x);
	//		yCoor.push_back(tp.y);
	//	}
	//}
	//IndexType xMin,xMax, yMin,yMax;
	//xMin = *(min_element(xCoor.begin(),xCoor.end()));
	//xMax = *(max_element(xCoor.begin(),xCoor.end()));
	//yMin = *(min_element(yCoor.begin(),yCoor.end()));
 //   yMax = *(max_element(yCoor.begin(),yCoor.end()));
	//	 
	//printf("xMin = %d; xMax = %d;yMin = %d;yMax = %d",xMin,xMax,yMin,yMax);
	//
	//imshow("LeftYX_255",lxyedges);
	//imshow("RightYX_255",rxyedges);
}

void StopMotion::detectEdgesCrossBoundary(Mat& leftImg,
										  vector<vector<Point> >& leftcontours,
										  vector<vector<Point> >& rightcontours, 
										  vector<Point2f>& seam, 
										  vector<IndexType>& lCroEIdx,
										  vector<IndexType>& rCorEIdx)
{
	// fit a box firstly, and then  eliminate the edges which far way from the seam.
	Point2f minPos, maxPos;
	vector<float> xCoor,yCoor;
	xCoor.clear();yCoor.clear();

	for (IndexType i = 0; i < seam.size(); ++ i )
	{
		Point2f tp = seam[i];
		xCoor.push_back(tp.x);
		yCoor.push_back(tp.y); 
	}//the coordinate systems are different between opencv and matlab

	minPos.x = *(min_element(xCoor.begin(),xCoor.end()));
	minPos.y = *(min_element(yCoor.begin(),yCoor.end()));
	maxPos.x = *(max_element(xCoor.begin(),xCoor.end()));
	maxPos.y = *(max_element(yCoor.begin(),yCoor.end()));

	//extend a band region for exactly to find the edges
	
	//draw this seam from graph cut process
//  	Mat visRect = Mat::zeros(maxPos.y + 100,maxPos.x + 100,CV_8UC3);
// 	for (IndexType i = 0; i < seam.size(); ++ i)
// 	{
// 
// 		visRect.at<Vec3b>(seam[i].y,seam[i].x) = fgMask;
// 	}
// 
//  	Scalar color = Scalar(50,50,50);
// 	rectangle(visRect,minPos,maxPos,color);
//  	circle(visRect,minPos,5,color);
//  	circle(visRect,maxPos,5,color);
// 	imshow("BBox",visRect);
	// end for draw bounding box

	Mat LEdgeIdx, REdgeIdx;

	LEdgeIdx = Mat(leftImg.size(),CV_8UC1,255 );
	REdgeIdx = Mat(leftImg.size(),CV_8UC1,255 );
	
	// if a pixel belongs to a edge, set this value as the idx of its edges.

	for(IndexType i = 0; i < leftcontours.size(); ++ i)
	{
		for (IndexType j = 0; j < leftcontours[i].size(); ++j)
		{
			Point tp = leftcontours[i][j];

			LEdgeIdx.at<uchar>(tp.y,tp.x) = i;// the pixel belongs to which component? Sure! 20160918
		}
	}

	for(IndexType i = 0; i < rightcontours.size(); ++ i)
	{
		for (IndexType j = 0; j < rightcontours[i].size(); ++j)
		{
			Point tp = rightcontours[i][j];

			REdgeIdx.at<uchar>(tp.y,tp.x) = i;
		}
	}


	// boundary index
	lCroEIdx.clear();
	rCorEIdx.clear();

	for (IndexType i = 0; i < seam.size(); ++ i )
	{
		Point2f tp = seam[i];

		if (LEdgeIdx.at<uchar>(tp.y,tp.x) <250)
		{
			lCroEIdx.push_back(LEdgeIdx.at<uchar>(tp.y,tp.x) );
		}

		if (REdgeIdx.at<uchar>(tp.y,tp.x) <250)
		{
			rCorEIdx.push_back(REdgeIdx.at<uchar>(tp.y,tp.x) );
		}
	}


	////draw the boundaries which cross the seam!
// 	Mat LcrosEdges,RcrosEdges;
// 	LcrosEdges = Mat::zeros(leftImg.size(),CV_8UC1);
// 	RcrosEdges = Mat::zeros(leftImg.size(),CV_8UC1); 
// 
// 	for (IndexType i = 0; i < lCroEIdx.size(); ++ i)
// 	{
// 		for (IndexType j = 0; j < leftcontours[lCroEIdx[i]].size(); ++j )
// 		{
// 			Point2f tp = leftcontours[lCroEIdx[i]][j];
// 
// 			LcrosEdges.at<uchar>(tp.y,tp.x) = 255;
// 		}
// 	}
// 
// 	for (IndexType i = 0; i < rCorEIdx.size(); ++ i)
// 	{
// 		for (IndexType j = 0; j < rightcontours[rCorEIdx[i]].size(); ++j )
// 		{
// 			Point2f tp = rightcontours[rCorEIdx[i]][j];
// 
// 			RcrosEdges.at<uchar>(tp.y,tp.x) = 255;
// 		}
// 	}
// 
// 	for (IndexType i = 0; i < seam.size(); ++ i )
// 	{
// 		Point2f tp = seam[i];
// 		LcrosEdges.at<uchar>(tp.y,tp.x) = 255;
// 		RcrosEdges.at<uchar>(tp.y,tp.x) = 255;
// 	}
// 
// 	imshow("LEdgeMask",LcrosEdges);
// 	imshow("REdgeMask",RcrosEdges);

}

void StopMotion::matchEdgesCrossSeam(Mat& leftImg, Mat& rightImg, 
									 vector<Point2f>& boundary, 
									 vector<vector<Point> >& leftcontours,
									 vector<vector<Point> >& rightcontours, 
									 vector<IndexType>& lCroEdgeIdx, 
									 vector<IndexType>& rCroEdgeIdx, 
									 vector<Point>& lMEdges, 
									 vector<Point>& rMEdges)
{

}

void StopMotion::combineMatchingInfo(vector<Point>& lPixM,vector<Point>& rPixM, 
									 vector<Point>& lEdgeM,vector<Point>& rEdgeM, 
									 vector<MatchingInfo>& finMatchInfo)
{
	finMatchInfo.clear();

	assert(lPixM.size() == rPixM.size() );
	for (IndexType i = 0; i < lPixM.size(); ++ i)
	{
		Point srP = lPixM[i];
		Point tgP = rPixM[i];
		MatchingInfo pM(srP,tgP);
		finMatchInfo.push_back(pM);
	}

	assert(lEdgeM.size() == rEdgeM.size() );
	for (IndexType i = 0; i < lEdgeM.size(); ++ i)
	{
		Point srP = lEdgeM[i];
		Point tgP = rEdgeM[i];
		MatchingInfo pM(srP,tgP);
		finMatchInfo.push_back(pM);
	}

}


// combine for piece homo

void StopMotion::combineMatching(vector<Point>& lPixM,vector<Point>& rPixM, 
									 vector<Point>& lEdgeM,vector<Point>& rEdgeM,
									 vector<CvPoint2D32f>& srfinMatchInfo,
									 vector<CvPoint2D32f>& tgfinMatchInfo)
{
	srfinMatchInfo.clear();
	tgfinMatchInfo.clear();

	assert(lPixM.size() == rPixM.size() );
	for (IndexType i = 0; i < lPixM.size(); ++ i)
	{
		Point srP = lPixM[i];
		Point tgP = rPixM[i];
		CvPoint2D32f srCor,tgCor;

		srCor.x = srP.x;srCor.y = srP.y;
		tgCor.x = tgP.x;tgCor.y = tgP.y;
		srfinMatchInfo.push_back(srCor);
		tgfinMatchInfo.push_back(tgCor);

	}

	assert(lEdgeM.size() == rEdgeM.size() );

	for (IndexType i = 0; i < lEdgeM.size(); ++ i)
	{
		Point srP = lEdgeM[i];
		Point tgP = rEdgeM[i];
		CvPoint2D32f srCor,tgCor;

		srCor.x = srP.x;srCor.y = srP.y;
		tgCor.x = tgP.x;tgCor.y = tgP.y;
		srfinMatchInfo.push_back(srCor);
		tgfinMatchInfo.push_back(tgCor);

	}
}

void StopMotion::deformRoiWithMathcingInfo(Mat& leftImg, Mat& rightImg, vector<Point2f>& seam, vector<MatchingInfo>& finM, Mat& outImg)
{

}

void StopMotion::deformRoiWithMathcing(Mat& leftImg, Mat& rightImg,
									   vector<Point2f>& seam, 
									   vector<CvPoint2D32f>& srfinMatchInfo, 
									   vector<CvPoint2D32f>& tgfinMatchInfo,
									   Mat& outImg)
{
	Point minPs,maxPs;
	ScalarType diaLen = 0.;

    //deformation only for the bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);
    //expand a few pixels for deformation
	Point expMinPs,expMaxPs;

	bool isExpand = false;
	IndexType nSkip = 20;

	vector<bool> skipFlag;

	isExpand = expanROI(leftImg, minPs,maxPs,expMinPs,expMaxPs,nSkip,skipFlag);
	// transfer the coordinates of the matching points
	if (isExpand)
	{
		updateCoorOfMatching(expMinPs,srfinMatchInfo,tgfinMatchInfo);
		m_roiMinPs = expMinPs;
	}else
	{
		updateCoorOfMatching(minPs,srfinMatchInfo,tgfinMatchInfo);
		m_roiMinPs = minPs;
	}

	Rect dRoi(expMinPs,expMaxPs);

	Mat lTpImg,rTpImg;
	leftImg(dRoi).copyTo(lTpImg);
	rightImg(dRoi).copyTo(rTpImg);

// 	//m_pieceHomo = new PieceHomo(leftImg,rightImg,50);
 	m_pieceHomo = new PieceHomo(lTpImg,rTpImg,10);

	connect(m_pieceHomo,SIGNAL(drawDeformedImg(Mat&,int,int,MatrixXX&,MatrixXX&)),
		    this,SLOT(drawEditImg(Mat&,int,int,MatrixXX&,MatrixXX&)));

 	m_pieceHomo->initFeatures(srfinMatchInfo,tgfinMatchInfo);//assign the matching information
 	m_pieceHomo->initHomo(m_glbHomo);

	//warping processing 
	Mat skipImg;
 	m_pieceHomo->deformMatchingImg(skipImg);

	//update the roi with matching processing

	Mat bbxImg;
	cvtColor(skipImg,bbxImg,CV_RGB2BGR);
	imshow("bbxIMg",bbxImg);

	leftImg.copyTo(outImg);

	if (isExpand)
	{
		Point ltp,rbp;

		if (skipFlag[0])
		{
	    	ltp.x = nSkip;
		}else
		{
			ltp.x = 0;
		}

		if (skipFlag[1])
		{
		  ltp.y = nSkip;
		}else
		{
			ltp.y = 0;
		}

		if (skipFlag[2])
		{
		   rbp.x = skipImg.cols - nSkip;
		}else
		{
		   rbp.x = skipImg.cols;
		}

		if (skipFlag[3])
		{
		   rbp.y = skipImg.rows - nSkip; 
		}else
		{
           rbp.y = skipImg.rows; 
		}


		Rect curR(ltp,rbp);

		Rect oriR(minPs,maxPs);

		assert(curR.width == oriR.width && curR.height == oriR.height);

		skipImg(curR).copyTo(outImg(oriR));


	}else
	{
		skipImg.copyTo(outImg(dRoi));
	}

}

void StopMotion::setInputImg(Mat& limg,Mat& rImg, MatrixXXi& labels)
{
// 	Mat ltemp, rtemp;
// 	cvtColor(limg,ltemp,CV_RGB2BGR);
// 	cvtColor(rImg,rtemp,CV_RGB2BGR);
// 
// 	ltemp.copyTo(m_lROI);
// 	rtemp.copyTo(m_rROI);


	limg.copyTo(m_lROI);

	rImg.copyTo(m_rROI);

	m_labels = labels;
}

//deformation while preserving lines

void StopMotion::deformLinesPoints(Mat& leftImg, Mat& rightImg, 
								   vector<Point2f>& seam, 
								   vector<CvPoint2D32f>& srfinMatchInfo, 
								   vector<CvPoint2D32f>& tgfinMatchInfo, 
								   Mat& outImg)
{

	//setInputImg(leftImg,rightImg);

	vector<lineSeg> leftLines,rightLines;
	vector<IndexType> lineMatching;

	leftLines.clear();
	rightLines.clear();
	lineMatching.clear();

	//obtainLinesMatching(leftImg, rightImg, seam, leftLines,rightLines, lineMatching);

	//using hough transformation method to obtain the line segments.
	obtainLinesHough(leftImg,rightImg,seam,leftLines,rightLines);

	deformROIMatchingLinesPoints(leftImg,rightImg,seam,
		srfinMatchInfo,tgfinMatchInfo,
		leftLines,rightLines,
		lineMatching,
		outImg);

}

void StopMotion::obtainLinesHough(Mat& leftImg, Mat& rightImg,vector<Point2f>& seam, 
								  vector<lineSeg>& leftLines, vector<lineSeg>& rightLines)
{
	Mat LEdges,REdges;

	if (m_rEdgesGray.empty() || m_lEdgesGray.empty())
	{
		spClassify.detectEdgesName(leftImg,LEdges,false);

		spClassify.detectEdgesName(rightImg,REdges,true);//should set the hand's region with blank, so no edges can cross the boun
	}else
	{
		m_lEdgesGray.copyTo(LEdges);
		m_rEdgesGray.copyTo(REdges);
	}


	vector<Vec4i> lLeftLine,rRightLines;

	detectLinesUsingHoughLines(LEdges,lLeftLine);

	detectLinesUsingHoughLines(REdges,rRightLines);

	sampleLines(seam,lLeftLine,leftLines);

	sampleLines(seam,rRightLines,rightLines);

	//showLines(leftImg,rightImg,seam,leftLines,rightLines);
}


void StopMotion::sampleLines(vector<Point2f>& seam,
							 vector<Vec4i>& oriLines, vector<lineSeg>& dSLines)
{
    Point minPs,maxPs;
	ScalarType diaLen = 0.;

	//bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);

	// only includes the inner lines, using clipping algorithm.

	for(IndexType i = 0; i < oriLines.size(); ++ i)
	{
		Point pt1, pt2;
		pt1.x = oriLines[i][0];
		pt1.y = oriLines[i][1];
		pt2.x = oriLines[i][2];
		pt2.y = oriLines[i][3];

		//bool isInBBX = isInnerLine(minPs,maxPs,pt1,pt2);
		double xst,xed,yst,yed;
		
		bool isInBBX = LiangBarsky(minPs.x,maxPs.x,minPs.y,maxPs.y,
			                      pt1.x,pt1.y,pt2.x,pt2.y,
								  xst,yst,xed,yed);

		if (isInBBX)
		{
			ScalarType lineLen = sqrt( (xed-xst)*(xed-xst) + (yed - yst)*(yed -yst) );

			if ( lineLen > 0.2 * diaLen)
			{
				lineSeg curLine;
				Point clipSP,clipEP;
				clipSP.x = xst;
				clipSP.y = yst;
				clipEP.x = xed;
				clipEP.y = yed;

				curLine.start_ = clipSP;
				curLine.end_ = clipEP;

				curLine.middle_ = 0.5 * (clipSP + clipEP);
				dSLines.push_back(curLine);

			}

		}

	}

}

bool StopMotion::LiangBarsky (double edgeLeft, double edgeRight, double edgeBottom, double edgeTop,   // Define the x/y clipping values for the border.
							  double x0src, double y0src, double x1src, double y1src,                 // Define the start and end points of the line.
							  double &x0clip, double &y0clip, double &x1clip, double &y1clip)  
{
	double t0 = 0.0;    double t1 = 1.0;
	double xdelta = x1src-x0src;
	double ydelta = y1src-y0src;
	double p,q,r;

	for(int edge=0; edge<4; edge++) {   // Traverse through left, right, bottom, top edges.
		if (edge==0) {  p = -xdelta;    q = -(edgeLeft-x0src);  }
		if (edge==1) {  p = xdelta;     q =  (edgeRight-x0src); }
		if (edge==2) {  p = -ydelta;    q = -(edgeBottom-y0src);}
		if (edge==3) {  p = ydelta;     q =  (edgeTop-y0src);   }   
		r = q/p;
		if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)

		if(p<0) {
			if(r>t1) return false;         // Don't draw line at all.
			else if(r>t0) t0=r;            // Line is clipped!
		} else if(p>0) {
			if(r<t0) return false;      // Don't draw line at all.
			else if(r<t1) t1=r;         // Line is clipped!
		}
	}

	x0clip = x0src + t0*xdelta;
	y0clip = y0src + t0*ydelta;
	x1clip = x0src + t1*xdelta;
	y1clip = y0src + t1*ydelta;

	return true;        // (clipped) line is drawn
}


void StopMotion::obtainLinesMatching(Mat& leftImg, Mat& rightImg, vector<Point2f>& seam,
									 vector<lineSeg>& leftLines, vector<lineSeg>& rightLines,
									 vector<IndexType>& lineMatching)
{

	////1-using matlab codes for detecting the edges
	Mat LEdges,REdges;
	spClassify.detectEdgesName(leftImg,LEdges,false);
	spClassify.detectEdgesName(rightImg,REdges,true);//should set the hand's region with blank, so no edges can cross the boun

	LEdges.copyTo(m_lEdgesGray);
	REdges.copyTo(m_rEdgesGray);

	//2-find the edge's component which the size of the points is large than a threshold.
	vector<vector<Point> > leftContours, rightContours;
	findEdgeComponent(LEdges,REdges,leftContours,rightContours);

	//3-which edges cross the seam? return the idx of the component.
	vector<IndexType> lcroedgeIdx, rcroedgeIdx;
	detectEdgesCrossBoundary(leftImg,leftContours,rightContours,seam,
							 lcroedgeIdx,rcroedgeIdx);
// 1
// 	getLinesSegments(seam,
// 		leftContours,rightContours,
// 		lcroedgeIdx,rcroedgeIdx,
// 		leftLines,rightLines);

	//2
// 	getLinesSegmentCorssSeam(leftImg,seam,
// 		leftContours,rightContours,
// 		lcroedgeIdx,rcroedgeIdx,
// 		leftLines,rightLines);

	//record the points which cross the seam

	vector<Point> lPoints,rPoints;
	getLinesSegmentCorssSeamPs(leftImg,seam,
		leftContours,rightContours,
		lcroedgeIdx,rcroedgeIdx,
		leftLines,rightLines,
		lPoints,rPoints);

	//
	//MatchingCrossPoints(seam,lPoints,rPoints);

	//for debug
	showLines(leftImg,rightImg,seam,leftLines,rightLines);

	//matching process

}


void StopMotion::obtainLinesMatchingPs(vector<Point2f>& seam, 
									   vector<lineSeg>& leftLines,vector<lineSeg>& rightLines, 
									   vector<Point>& lPoints,vector<Point>& rPoints)
{
	////1-using matlab codes for detecting the edges
	Mat LEdges,REdges;
	spClassify.detectEdgesName(m_lROI,LEdges,false);
	spClassify.detectEdgesName(m_rROI,REdges,true);//should set the hand's region with blank, so no edges can cross the boun

	//2-find the edge's component which the size of the points is large than a threshold.
	vector<vector<Point> > leftContours, rightContours;
	findEdgeComponent(LEdges,REdges,leftContours,rightContours);

	//3-which edges cross the seam? return the idx of the component.
	vector<IndexType> lcroedgeIdx, rcroedgeIdx;
	detectEdgesCrossBoundary(m_lROI,leftContours,rightContours,seam,
		lcroedgeIdx,rcroedgeIdx);

	//4
	getLinesSegmentCorssSeamPs(m_lROI,seam,
		leftContours,rightContours,
		lcroedgeIdx,rcroedgeIdx,
		leftLines,rightLines,
		lPoints,rPoints);

	//for debug
	showLines(m_lROI,m_rROI,seam,leftLines,rightLines);

}

void StopMotion::getLinesSegments(vector<Point2f>& seam, 
								  vector<vector<Point> >& lContours,vector<vector<Point> >& rContours, 
								  vector<IndexType>& lIdx,vector<IndexType>& rIdx, 
								  vector<lineSeg>& lLines, vector<lineSeg>& rLines)
{

	Point minPs,maxPs;
	ScalarType diaLen = 0.;

	//bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);

	IndexType lineLen = 10;

  //for left edges
	for (IndexType i = 0; i < lIdx.size(); ++ i)
	{
		vector<Point> curCont = lContours[lIdx[i]]; 
        if (curCont.size() < lineLen) continue;

		IndexType nLines = (IndexType)(curCont.size()/lineLen);

		for (IndexType j = 0; j < nLines; ++ j)
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
				lineSeg curLine;
				curLine.start_ = stP;
				curLine.end_ = edP;
				curLine.middle_ = curCont[(j+.5)*lineLen];

				lLines.push_back(curLine);
			}

		}// for a contour

	}//end for all contours which cross the seam

  //for right edges
	for (IndexType i = 0; i < rIdx.size(); ++ i)
	{
		vector<Point> curCont = rContours[rIdx[i]];
		if (curCont.size() < lineLen) continue;

		IndexType nLines = curCont.size()/lineLen;

		for (IndexType j = 0; j < nLines; ++ j)
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
				lineSeg curLine;
				curLine.start_ = stP;
				curLine.end_ = edP;
				curLine.middle_ = curCont[(j+.5)*lineLen];

				rLines.push_back(curLine);
			}

		}// for a contour

	}//end for all contours which cross the seam


}

//record the line segments which cross the seam
void StopMotion::getLinesSegmentCorssSeam(Mat& leftImg, vector<Point2f>& seam,
										  vector<vector<Point> >& lContours,
										  vector<vector<Point> >& rContours, 
										  vector<IndexType>& lIdx,
										  vector<IndexType>& rIdx,
										  vector<lineSeg>& lLines, vector<lineSeg>& rLines)
{
	Point minPs,maxPs;
	ScalarType diaLen = 0.;

	//bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);

	Mat seamMask;
	seamMask = Mat(leftImg.size(),CV_8UC1,255);//do not assign with 0!!

	for (IndexType i = 0; i < seam.size(); ++ i )
	{
		Point2f tp = seam[i];
		seamMask.at<uchar>(tp.y,tp.x) = 1;
	}

	IndexType lineLen = 10;

	//for left edges
	for (IndexType i = 0; i < lIdx.size(); ++ i) //Component
	{
		vector<Point> curCont = lContours[lIdx[i]]; 
		if (curCont.size() < lineLen) continue;

		IndexType nLines = (IndexType)(curCont.size()/lineLen);

		for (IndexType j = 0; j < nLines; ++ j) // line segments
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
 				vector<Point> linesP;
 
 				linesP.assign(curCont.begin() + (j*lineLen),curCont.begin() + ((j+1)*lineLen - 1));
 
 				for (IndexType k = 0; k < linesP.size(); ++ k) //points
 				{
 					if (seamMask.at<uchar>(linesP[k].y,linesP[k].x)  == 1)
 					{
 						lineSeg curLine;
 						curLine.start_ = stP;
 						curLine.end_ = edP;
 						curLine.middle_ = curCont[(j+.5)*lineLen];
 						lLines.push_back(curLine);
 
						//record the intersection point

						//circle(m_lROI,linesP[k],3,Scalar(255,255,255));
 						break;
 					}
 
 				}

			}

		}// for a contour

	}//end for all contours which cross the seam

	//for right edges
	for (IndexType i = 0; i < rIdx.size(); ++ i) //Component
	{
		vector<Point> curCont = rContours[rIdx[i]];
		if (curCont.size() < lineLen) continue;

		IndexType nLines = curCont.size()/lineLen;

		for (IndexType j = 0; j < nLines; ++ j) // line segments
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
 				vector<Point> linesP;
 
 				linesP.assign(curCont.begin() + (j*lineLen),curCont.begin() + ((j+1)*lineLen - 1));
 
 				for (IndexType k = 0; k < linesP.size(); ++ k) //points
 				{
 					if (seamMask.at<uchar>(linesP[k].y,linesP[k].x) == 1)
 					{
 						lineSeg curLine;
 						curLine.start_ = stP;
 						curLine.end_ = edP;
 						curLine.middle_ = curCont[(j+.5)*lineLen];
 
 						rLines.push_back(curLine);
 
						//circle(m_lROI,linesP[k],3,Scalar(255,0,0));

 						break;
 					}
 				}

			}

		}// for a contour

	}//end for all contours which cross the seam

	//imshow("LRITSP",m_lROI);
	//imshow("RITSP",m_rROI);

}


void StopMotion::getLinesSegmentCorssSeamPs(Mat& leftImg,vector<Point2f>& seam, 
											vector<vector<Point> >& lContours,
											vector<vector<Point> >& rContours,
											vector<IndexType>& lIdx,vector<IndexType>& rIdx, 
											vector<lineSeg>& lLines, vector<lineSeg>& rLines,
											vector<Point>& lInterSPs,vector<Point>& rInterSPs)
{
	Point minPs,maxPs;
	ScalarType diaLen = 0.;

	//bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);

	Mat seamMask;
	seamMask = Mat(leftImg.size(),CV_8UC1,255);//do not assign with 0!!

	MatrixXXi isIntersS;
	isIntersS.setZero(leftImg.rows,leftImg.cols);

	for (IndexType i = 0; i < seam.size(); ++ i )
	{
		Point2f tp = seam[i];
		seamMask.at<uchar>(tp.y,tp.x) = 1;
	}

	IndexType lineLen = 10;

	lInterSPs.clear();
	rInterSPs.clear();

	Mat ltemp,rtemp;

	m_lROI.copyTo(ltemp);
	m_rROI.copyTo(rtemp);

	//for left edges
	for (IndexType i = 0; i < lIdx.size(); ++ i) //Component
	{
		vector<Point> curCont = lContours[lIdx[i]]; 
		if (curCont.size() < lineLen) continue;

		IndexType nLines = (IndexType)(curCont.size()/lineLen);

		for (IndexType j = 0; j < nLines; ++ j) // line segments
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
				vector<Point> linesP;

				linesP.assign(curCont.begin() + (j*lineLen),curCont.begin() + ((j+1)*lineLen - 1));

				for (IndexType k = 0; k < linesP.size(); ++ k) //points
				{
					if (seamMask.at<uchar>(linesP[k].y,linesP[k].x)  == 1 && (!isIntersS(linesP[k].y,linesP[k].x)))
					{
						isIntersS(linesP[k].y,linesP[k].x) = 1;

						lInterSPs.push_back(linesP[k]);

						lineSeg curLine;
						curLine.start_ = stP;
						curLine.end_ = edP;
						curLine.middle_ = curCont[(j+.5)*lineLen];
						lLines.push_back(curLine);

						//record the intersection point

						circle(ltemp,linesP[k],3,Scalar(255,255,255));
						break;
					}

				}

			}

		}// for a contour

	}//end for all contours which cross the seam

	//for right edges
	isIntersS.setZero(leftImg.rows,leftImg.cols);
	for (IndexType i = 0; i < rIdx.size(); ++ i) //Component
	{
		vector<Point> curCont = rContours[rIdx[i]];
		if (curCont.size() < lineLen) continue;

		IndexType nLines = curCont.size()/lineLen;

		for (IndexType j = 0; j < nLines; ++ j) // line segments
		{
			Point stP = curCont[j*lineLen];
			Point edP = curCont[(j+1)*lineLen - 1];

			bool isInBBX = isLocatedBBX(minPs,maxPs,stP,edP);

			if (isInBBX)
			{
				vector<Point> linesP;

				linesP.assign(curCont.begin() + (j*lineLen),curCont.begin() + ((j+1)*lineLen - 1));

				for (IndexType k = 0; k < linesP.size(); ++ k) //points
				{
					if (seamMask.at<uchar>(linesP[k].y,linesP[k].x) == 1 && (!isIntersS(linesP[k].y,linesP[k].x)))
					{
						isIntersS(linesP[k].y,linesP[k].x) = 1;

						rInterSPs.push_back(linesP[k]);

						lineSeg curLine;
						curLine.start_ = stP;
						curLine.end_ = edP;
						curLine.middle_ = curCont[(j+.5)*lineLen];

						rLines.push_back(curLine);

						circle(ltemp,linesP[k],3,Scalar(255,0,0));

						break;
					}
				}

			}

		}// for a contour

	}//end for all contours which cross the seam


	imshow("LRITSP",ltemp);
	//imshow("RITSP",m_rROI);


}


bool StopMotion::isLocatedBBX(Point& minP, Point& maxP, Point& stP, Point& edP)
{
	Point middle;
	middle.x = 0.5*(stP.x + edP.x);
	middle.y = 0.5*(stP.y + edP.y);

	if ( middle.x >= minP.x && middle.x <= maxP.x 
		&& middle.y >= minP.y && middle.y <= maxP.y)
	{
		return true;
	}else
	{
		return false;
	}
}

bool StopMotion::isInnerLine(Point& minP, Point& maxP, Point& stP, Point& edP)
{
	bool isInF = false;
	bool isInS = false;

	if (stP.x >= minP.x && stP.x <= maxP.x 
		&& stP.y >= minP.y && stP.y <= maxP.y)
	{
		isInF = true;
	}

	if (edP.x >= minP.x && edP.x <= maxP.x 
		&& edP.y >= minP.y && edP.y <= maxP.y)
	{
		isInS = true;
	}

	if (isInF && isInS)
	{
		return true;
	}else
	{
		return false;
	}
}

void StopMotion::showLines(Mat& leftImg, Mat& rightImg, vector<Point2f>& seam, 
						   vector<lineSeg>& lLines, vector<lineSeg>& rLines)
{
	Mat lTempImg,rTempImg;

	leftImg.copyTo(lTempImg);
	rightImg.copyTo(rTempImg);

	Scalar color(255,0,0);
	//for left
	for (IndexType i = 0; i < lLines.size(); ++ i)
	{
		lineSeg curl = lLines[i]; 
		line(lTempImg,curl.start_,curl.end_,color);
	}

    //for right
	for (IndexType i = 0; i < rLines.size(); ++ i)
	{
		lineSeg curl = rLines[i]; 
		line(rTempImg,curl.start_,curl.end_,color);
	}

 	Scalar color2(255,255,0);
 	for (IndexType i = 0; i < seam.size(); i +=2)
 	{
 		circle(lTempImg,seam[i],1,color2);
 		circle(rTempImg,seam[i],1,color2);
 	}

	Mat lTT,rTT;
	cvtColor(lTempImg,lTT,CV_RGB2BGR);
	cvtColor(rTempImg,rTT,CV_RGB2BGR);

	imshow("leftLineSeg", lTT);
	imshow("rightLineSeg", rTT);

}

void StopMotion::deformROIMatchingLinesPoints(Mat& leftImg, Mat& rightImg,
											  vector<Point2f>& seam, 
											  vector<CvPoint2D32f>& srfinMatchInfo, 
											  vector<CvPoint2D32f>& tgfinMatchInfo, 
											  vector<lineSeg>& lLines, 
											  vector<lineSeg>& rLines, 
											  vector<IndexType>& linesMatchInfo, 
											  Mat& outImg)
{
	Point minPs,maxPs;
	ScalarType diaLen = 0.;

	//deformation only for the bounding box of the seam 
	boundingboxSeam(seam,minPs,maxPs,diaLen);
	//expand a few pixels for deformation
	Point expMinPs,expMaxPs;

	bool isExpand = false;
	IndexType nSkip = 20;

	vector<bool> skipFlag;

	isExpand = expanROI(leftImg, minPs,maxPs,expMinPs,expMaxPs,nSkip,skipFlag);

	// transfer the coordinates of the matching points
	if (isExpand)
	{
		updateCoorOfMatching(expMinPs,srfinMatchInfo,tgfinMatchInfo);
		updateCoorOfLines(expMinPs,lLines,rLines);

		m_roiMinPs = expMinPs;
	}else
	{
		updateCoorOfMatching(minPs,srfinMatchInfo,tgfinMatchInfo);
		updateCoorOfLines(minPs,lLines,rLines);

		m_roiMinPs = minPs;
	}

	Rect dRoi(expMinPs,expMaxPs);

	Mat lTpImg,rTpImg;
	leftImg(dRoi).copyTo(lTpImg);
	rightImg(dRoi).copyTo(rTpImg);


	//show the input for deformation
// 	Mat ltt,rtt;
// 	cvtColor(lTpImg,ltt,CV_RGB2BGR);
// 	cvtColor(rTpImg,rtt,CV_RGB2BGR);
// 	imshow("input-LdI",ltt);
// 	imshow("input-RdI",rtt);


	// deformation now
	m_pieceHomo = new PieceHomo(lTpImg,rTpImg,10);

	connect(m_pieceHomo,SIGNAL(drawDeformedImg(Mat&,int,int,MatrixXX&,MatrixXX&)),
		this,SLOT(drawEditImg(Mat&,int,int,MatrixXX&,MatrixXX&)));

	m_pieceHomo->initFeatures(srfinMatchInfo,tgfinMatchInfo);//assign the matching information
	m_pieceHomo->initHomo(m_glbHomo);

	m_pieceHomo->initLines(lLines,rLines,linesMatchInfo);

	//warping processing 
	Mat skipImg;
	m_pieceHomo->deformMatchingLinesPoints(skipImg);

	//complete
	Mat bbxImg;
	cvtColor(skipImg,bbxImg,CV_RGB2BGR);
	imshow("bbxIMg",bbxImg);

	leftImg.copyTo(outImg);

	if (isExpand)
	{
		Point ltp,rbp;

		if (skipFlag[0])
		{
			ltp.x = nSkip;
		}else
		{
			ltp.x = 0;
		}

		if (skipFlag[1])
		{
			ltp.y = nSkip;
		}else
		{
			ltp.y = 0;
		}

		if (skipFlag[2])
		{
			rbp.x = skipImg.cols - nSkip;
		}else
		{
			rbp.x = skipImg.cols;
		}

		if (skipFlag[3])
		{
			rbp.y = skipImg.rows - nSkip; 
		}else
		{
			rbp.y = skipImg.rows; 
		}

		Rect curR(ltp,rbp);

		Rect oriR(minPs,maxPs);

		assert(curR.width == oriR.width && curR.height == oriR.height);

		skipImg(curR).copyTo(outImg(oriR));

	}else
	{
		skipImg.copyTo(outImg(dRoi));
	}

}



void StopMotion::updateCoorOfMatching(Point& minPs, 
									  vector<CvPoint2D32f>& srfinMatchInfo,
									  vector<CvPoint2D32f>& tgfinMatchInfo)
{
	IndexType feaSize = srfinMatchInfo.size();
	for (IndexType i = 0; i < feaSize; ++ i)
	{
		srfinMatchInfo[i].x -= minPs.x;
		srfinMatchInfo[i].y -= minPs.y;
		assert(srfinMatchInfo[i].x >= 0 &&  srfinMatchInfo[i].y >=0); //appear once.

		tgfinMatchInfo[i].x -= minPs.x;
		tgfinMatchInfo[i].y -= minPs.y;
        assert(tgfinMatchInfo[i].x >= 0 && tgfinMatchInfo[i].y >=0);

	}
}


void StopMotion::updateCoorOfLines(Point& minPs, 
								   vector<lineSeg>& lLines, vector<lineSeg>& rLines)
{
	for (IndexType i = 0; i < lLines.size(); ++ i)
	{
		lineSeg curline = lLines[i];

		curline.start_.x -= minPs.x;
		curline.start_.y -= minPs.y;

		curline.middle_.x -= minPs.x;
		curline.middle_.y -= minPs.y;

		curline.end_.x -= minPs.x;
		curline.end_.y -= minPs.y;

		lLines[i] = curline;
	} 

	for (IndexType i = 0; i < rLines.size(); ++ i)
	{
		lineSeg curline = rLines[i];

		curline.start_.x -= minPs.x;
		curline.start_.y -= minPs.y;

		curline.middle_.x -= minPs.x;
		curline.middle_.y -= minPs.y;

		curline.end_.x -= minPs.x;
		curline.end_.y -= minPs.y;

		rLines[i] = curline;
	}

}

bool StopMotion::expanROI(Mat& inImg,Point& oriMin, Point& oriMax,
						  Point& newMin,Point& newMax, IndexType nSlide)
{
	//min point
	bool isexpand = false;

	if (oriMin.x - nSlide > 0)
	{
		newMin.x = oriMin.x - nSlide;
		isexpand = true;
	}else
	{
		newMin.x = oriMin.x;
	}

	if (oriMin.y - nSlide > 0)
	{
		newMin.y = oriMin.y - nSlide;
		isexpand = true;
	}else
	{
		newMin.y = oriMin.y;
	}

	//max point
	if (oriMax.x + nSlide <inImg.cols)
	{
		newMax.x = oriMax.x + nSlide;
		isexpand = true;
	}else
	{
		newMax.x = oriMax.x;
	}

	if (oriMax.y + nSlide < inImg.rows)
	{
		newMax.y = oriMax.y + nSlide;
		isexpand = true;
	}else
	{
		newMax.y = oriMax.y;
	}

	return isexpand;

}


bool StopMotion::expanROI(Mat& inImg,Point& oriMin, Point& oriMax,Point& newMin,Point& newMax, 
						  IndexType nSlide,vector<bool>& flagSkip)
{
	//min point
	flagSkip.clear();

	bool isexpand = false;

	if (oriMin.x - nSlide > 0)
	{
		newMin.x = oriMin.x - nSlide;
		isexpand = true;
		flagSkip.push_back(true);

	}else
	{
		newMin.x = oriMin.x;
	    flagSkip.push_back(false);
	}

	if (oriMin.y - nSlide > 0)
	{
		newMin.y = oriMin.y - nSlide;
		isexpand = true;
		flagSkip.push_back(true);
	}else
	{
		newMin.y = oriMin.y;
		flagSkip.push_back(false);
	}

	//max point
	if (oriMax.x + nSlide <inImg.cols)
	{
		newMax.x = oriMax.x + nSlide;
		isexpand = true;
		flagSkip.push_back(true);
	}else
	{
		newMax.x = oriMax.x;
		flagSkip.push_back(false);
	}

	if (oriMax.y + nSlide < inImg.rows)
	{
		newMax.y = oriMax.y + nSlide;
		isexpand = true;
		flagSkip.push_back(true);
	}else
	{
		newMax.y = oriMax.y;
		flagSkip.push_back(false);
	}

	return isexpand;
}

void StopMotion::drawEditImg(Mat& oriImg,int heigh,int width,
							 MatrixXX& quadCoor,MatrixXX& textCoor)
{
	emit transferImg(oriImg,m_roiMinPs,heigh,width,quadCoor,textCoor);
}

void StopMotion::expandSeam(MatrixXXi& labels, IndexType band, Mat& resGrayImg)
{
	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	//0-1 label to gray image

	Mat initMark,temp;
	initMark.create(rows,cols,CV_8UC1);

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{

			IndexType isH = labels(y,x);

			if (isH)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
			else
			{
				initMark.at<uchar>(y,x) = 255;//white
			}

		}
	}

	//using gray image to find the seam firstly

	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(6,6));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(initMark, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image

	Mat dilImage;
	//IndexType bandwidth = 10;

	Mat edgeExStruct = getStructuringElement(MORPH_RECT,Size(band,band));

	//do expand
	morphologyEx(edgeTemp, dilImage, CV_MOP_DILATE, edgeExStruct); //only expand the edge

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			IndexType isEdge = dilImage.at<uchar>(y,x);//0 or 255, 255 represents edge

			if (isEdge)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
		}
	}

	initMark.copyTo(resGrayImg);
}

void StopMotion::expandSeamExpand(MatrixXXi& labels, IndexType band,
								  Mat& resGrayImg, MatrixXXi& resLabels)
{
	IndexType rows = labels.rows();
	IndexType cols = labels.cols();

	//0-1 label to gray image
	resLabels.setZero(rows,cols);

	resLabels = labels;

	Mat initMark,temp;
	initMark.create(rows,cols,CV_8UC1);

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{

			IndexType isH = labels(y,x);

			if (isH)
			{
				initMark.at<uchar>(y,x) = 0;// black
			}
			else
			{
				initMark.at<uchar>(y,x) = 255;//white
			}

		}
	}

	//using gray image to find the seam firstly

	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(6,6));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(initMark, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image

	Mat dilImage;
	//IndexType bandwidth = 10;

	Mat edgeExStruct = getStructuringElement(MORPH_RECT,Size(band,band));

	//do expand
	morphologyEx(edgeTemp, dilImage, CV_MOP_DILATE, edgeExStruct); //only expand the edge

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			IndexType isEdge = dilImage.at<uchar>(y,x);//0 or 255, 255 represents edge

			if (isEdge)
			{
				initMark.at<uchar>(y,x) = 0;// black
				resLabels(y,x) = 1;
			}
		}
	}

	initMark.copyTo(resGrayImg);
}


void StopMotion::refineDownedPtslist(Mat& lImg,Mat& rImg, 
									 vector<Point2f>& oriList, 
									 vector<Point2f>& ldList,vector<Point2f>& rdList,
									 vector<Point2f>& ldListref,vector<Point2f>& rdListref)
{
	//how to refine the down sample points?  using sift descriptor, weight coordinates? angle?

}

void StopMotion::detectLinesUsingHoughLines(Mat& edges, vector<Vec4i>& lines)
{

	Mat temp;
	temp.create(edges.rows,edges.cols,CV_8UC1);

	for (IndexType i = 0; i < edges.rows; ++ i)
	{
		for (IndexType j = 0; j < edges.cols; ++ j)
		{
			IndexType gVal = edges.at<uchar>(i,j);
			if (gVal < 50)
			{
				temp.at<uchar>(i,j) = 0;
			}else
			{
				temp.at<uchar>(i,j) = 255;
			}
		}
	}

	//imshow("Before Lines",temp);

	HoughLinesP(temp, lines, 1, CV_PI/180, 30, 10, 10 );

	//show the lines

	Mat cdst;
	cdst.create(edges.rows,edges.cols,CV_8UC3);
	cdst.setTo(0);

	for( size_t i = 0; i < lines.size(); i++ )
	{
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];
		line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);

	}

	//imshow("oriLines",cdst);
}

void StopMotion::gloDeformRoi(Mat& roiL,Mat& roiR,
							  vector<CvPoint2D32f>& lMPs,vector<CvPoint2D32f>& rMPs, 
							  Mat& outImg)
{

	m_pieceHomo = new PieceHomo(roiL,roiR,10);

	connect(m_pieceHomo,SIGNAL(drawDeformedImg(Mat&,int,int,MatrixXX&,MatrixXX&)),
		this,SLOT(drawEditImg(Mat&,int,int,MatrixXX&,MatrixXX&)));

	m_pieceHomo->initFeatures(lMPs,rMPs);//assign the matching information
	m_pieceHomo->initHomo(m_glbHomo);

	//warping processing 
	//Mat skipImg;
	m_pieceHomo->deformMatchingImg(outImg);

}

void StopMotion::gloDeformRoiwithLines(Mat& roiL,Mat& roiR,
											vector<CvPoint2D32f>& lMPs,
											vector<CvPoint2D32f>& rMPs, 
											Mat& outImg)
{
	//obtain lines
	vector<lineSeg> leftLines,rightLines;

	leftLines.clear();

	rightLines.clear();

	obtainAllLines(roiL,roiR,leftLines,rightLines);

	deforAllLines(roiL,roiR,lMPs,rMPs,leftLines,rightLines,outImg);

}

void StopMotion::obtainAllLines(Mat& roiL, Mat& roiR,vector<lineSeg>& leftLines, vector<lineSeg>& rightLines)
{
	Mat LEdges,REdges;

	if (m_rEdgesGray.empty() || m_lEdgesGray.empty())
	{
		spClassify.detectEdgesName(roiL,LEdges,false);

		spClassify.detectEdgesName(roiR,REdges,true);//should set the hand's region with blank, so no edges can cross the boun
	}else
	{
		m_lEdgesGray.copyTo(LEdges);
		m_rEdgesGray.copyTo(REdges);
	}


	vector<Vec4i> lLeftLine,rRightLines;

	detectLinesUsingHoughLines(LEdges,lLeftLine);

	detectLinesUsingHoughLines(REdges,rRightLines);

	ScalarType diaLen = sqrt(roiL.rows * roiL.rows + roiL.cols * roiL.cols);

	transLines2Points(lLeftLine,diaLen,leftLines);

	transLines2Points(rRightLines,diaLen,leftLines);

}

void StopMotion::deforAllLines(Mat& roiL, Mat& roiR, 
							   vector<CvPoint2D32f>& lMPs,vector<CvPoint2D32f>& rMPs, 
							   vector<lineSeg>& leftLines, vector<lineSeg>& rightLines,
							   Mat& outImg)
{

	m_pieceHomo = new PieceHomo(roiL,roiR,10);

	connect(m_pieceHomo,SIGNAL(drawDeformedImg(Mat&,int,int,MatrixXX&,MatrixXX&)),
			this,SLOT(drawEditImg(Mat&,int,int,MatrixXX&,MatrixXX&)));

	m_pieceHomo->initFeatures(lMPs,rMPs);//assign the matching information
	m_pieceHomo->initHomo(m_glbHomo);

	vector<IndexType> linesMat;
	m_pieceHomo->initLines(leftLines,rightLines,linesMat);

	//warping processing 

	m_pieceHomo->deformMatchingLinesPoints(outImg);

}

void StopMotion::transLines2Points(vector<Vec4i>& lines, ScalarType disLen, vector<lineSeg>& linesSegs)
{
	for (IndexType i = 0; i < lines.size(); ++ i)
	{
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];

       ScalarType lineLen = sqrt( (pt1.x-pt2.x)*(pt1.x-pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) );

	   if ( lineLen > 0.3 * disLen)
	   {
		   lineSeg curLine;
		   curLine.start_ = pt1;
		   curLine.end_ = pt2;

		   curLine.middle_ = 0.5 * (pt1 + pt2);
		   linesSegs.push_back(curLine);

	   }
	}
}

