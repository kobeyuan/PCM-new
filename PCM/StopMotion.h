#ifndef _STOPMOTION_H_
#define _STOPMOTION_H_


#ifndef  	N_GAUSS
#define	N_GAUSS 5
#endif



#include <QThread>
#include "QImage"
#include "cv.h"
#include "SingleFrame.h"
#include "SingleFrameSet.h"

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/eigen.hpp"

//#include "opencv2/imgcodecs.hpp"

#include "Grabcut/gmm.h"

#include "basic_types.h"

//#include <gco-v3.0/GCoptimization.h>

#include <numeric>
#include <fstream>
#include <sstream>

#define frame_frame_to_key(f,i) ((f<<16)|i)
#define get_aframe_from_key(k) (k>>16)
#define get_bframe_from_key(k) (k&0xffff)

using namespace std;
using namespace cv;

//Vec3b bgMask(0,255,255);//blue
static Vec3b bgMask(0,255,57);//green - 0, the same with hand
static Vec3b fgMask(255,0,0); //red -1 
static Vec3b ukMask(145,145,145);//gray - 2
static Vec3b bkMask(0,0,0);


#include "spectral_clustering.h"

#include "AutoSelectionPairs.h"

#include "PiecewiseHomo.h"

struct MatchingInfo;

class StopMotion : public QThread
{
	Q_OBJECT
public:
	void run() Q_DECL_OVERRIDE;
signals:
	void finsh_compute();

public:
	StopMotion(std::vector< std::pair<IndexType,IndexType> >& keyPairs): m_Video(SingleFrameSet::get_instance().getTotVideo() ),
		       m_keyPairs(keyPairs)
	{
		//m_sltRect.
		m_thresDiff = 15;
		m_isWithInteractive = false;
		m_curPairIdx = 0;
		m_isOkDeformation = false;
// 		if (!!m_gcImg)
// 		{
// 			m_gcImg = nullptr;
// 		}

		//connect(m_pieceHomo,SIGNAL(drawDeformedImg(Mat&)),this,SLOT(drawEditImg(Mat&)));
	}

	~StopMotion();
	void setRect(std::vector<Rect>& slt_){ m_sltRect = slt_; }

public:
	void testChangeROI(IndexType pairId);
	void diffRoiBackG(IndexType fIdx,cv::Mat& resROI);
	void labelBGforRoi(Mat& bg, Mat& curF, Rect& rect_, Mat& resROI);
	void detectFeatures(Mat& oriImg, cv::vector<Point2f>& conners);

	void makeDifferenceWithBG(IndexType fIdx, MatrixXXi& labels, MatrixXXi& smoothLabels);
	void calDiffROIWithBG(Mat& bg, Mat& curF, Rect& rect_, MatrixXXi& labels, Mat& resROI);

	void setShowPairId(IndexType* curIdx);

	void matchAndDeform(Mat& srcImg, Mat& desImg, Mat& outImg);

	void matchFlowAndDeform(Mat& srcImg, Mat& desImg, Mat& outImg);
	//using the optical flow method
	void matchWithFlow(Mat& srcImg, Mat& desImg, Mat& outImg); 

	void matchUsingFlowAndForeLabels(Mat& srcImg, Mat& desImg, MatrixXXi& foreLabels, Mat& outImg);

	//registration-tranformation
	bool createDetectorDescriptorMatcher(const string& detectorType, 
		const string& descriptorType, const string& matcherType,  
		Ptr<FeatureDetector>& featureDetector,  
		Ptr<DescriptorExtractor>& descriptorExtractor,  
		Ptr<DescriptorMatcher>& descriptorMatcher );

	bool matchingDescriptor(const vector<KeyPoint>& queryKeyPoints,
		const vector<KeyPoint>& trainKeyPoints,  
		const Mat& queryDescriptors,
		const Mat& trainDescriptors,   
		Ptr<DescriptorMatcher>& descriptorMatcher,  
		Mat& homo,
		vector<DMatch>& matchPs,
		bool enableRatioTest = true);

    bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints,    
		const std::vector<cv::KeyPoint>& trainKeypoints,     
		float reprojectionThreshold,    
		std::vector<cv::DMatch>& matches,    
		cv::Mat& homography  );


	std::vector< std::pair<IndexType,IndexType> >& getKeypairs()
	{
		return m_keyPairs;
	}

	void refineKeyps(Mat& cufImg, vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs);
	void refineKeyPs2(Rect& oriRect,vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs);
	bool isKPInBbx(KeyPoint& kp,Rect& oriRect);
	bool isDifferent(Vec3b& bgPix, Vec3b& cufPix);

	bool trainGMM(Mat& inputImg, MatrixXXi& initalLabels);
	ScalarType calculateProb(IndexType nDim,MatrixXX& mean_,MatrixXX& cov,MatrixXX& sValue);
	void minCut(Mat& leftImg, MatrixXXi& lastLabels, MatrixXXi& outBgFgMark);
	void handRemoval(Mat& leftImg, Mat& rightImg, MatrixXXi& leftHandMark, MatrixXXi& outBgFgMark);
	ScalarType computerProb(IndexType alpha, IndexType comIdx, ScalarType reflectance[3]);
	ScalarType maxProb(IndexType alpha, ScalarType reflectance[3] );
	ScalarType minProb(IndexType alpha, ScalarType reflectance[3] );
	ScalarType smoothItem(IndexType aLabel, IndexType bLabel, Matrix34& rgbColor);
	ScalarType smoothItemOri(IndexType aLabel, IndexType bLabel, Vec3& curPixel, Vec3& neigPixel);
	ScalarType smoothItemOri(IndexType aLabel, IndexType bLabel, ScalarType curPixel[3], ScalarType neigPixel[3]);
	void getColorUsingPtr(Mat& inputImg, IndexType ptrAdd, Vec3& curColor);
	void imageGenerate(Mat& leftImg, Mat& rightImg, MatrixXXi& BgFgMark, Mat& outPutImg);

	//out the global image
	void imageGenerateAll(Mat& leftImg, Mat& rightImg, MatrixXXi& BgFgMark, Mat& outPutImg, Mat& outFinalImg);


	void imageGenerWithBackg(Mat& leftImg, Mat& rightImg, MatrixXXi& BgFgMark, Mat& bgROI,Mat& outPutImg);
	void setGraphCutWeight(ScalarType weight_);
	void setTresChange(IndexType diff);

	void initImageLabelWithHandDetect(Mat& inputImg);
	bool initImageLabelWithInteractive(MatrixXXi& initLabel_);
	void imageLabelAfterGMM(MatrixXXi& lastLabels/*only 0-1 value*/);//use the GMM information
	void updateGMM(Mat& inputImg, MatrixXXi& lastLabels);
	void copyROIFromKey(IndexType fIdx, Mat& desIMg);

	void motionToColor(Mat flow, Mat& vColor);
	void makecolorwheel(vector<Scalar> &colorwheel);

	void setCurPairIdx(IndexType idx_){m_curPairIdx = idx_;}

	void rgb2YUV(Vec3b& rgb_, Vec3b& yuv_);
	bool isSkinPixel(Vec3b& pRGB);
	void handMarkerRGBYUV(const Mat& srImg, MatrixXXi& handMK);
	void getinitTrainData(const Mat& srImg, MatrixXXi& iniTrainLab, MatrixXXi& stillPixel);

	void findUnknowns(Mat& initHandMarker, Mat& edgeMarker, MatrixXXi& unknownLabels, IndexType bandwidth);

	void itaHandMarkers(const Mat& srImg, MatrixXXi& handMK);

	void gcSegmentation(Mat& oriImg, MatrixXXi& initLabels, MatrixXXi& resLabels);

	void findSureBackground(Mat& filterImg, MatrixXXi& smallUnknown, MatrixXXi& largeUnknown, MatrixXXi& sureBackg);

	void findStillPixel(Mat& oriImg, MatrixXXi& largeUnknown, MatrixXXi& stillPixel);

	// only using the ROI region optical flow to estimate the affine matrix
	void matchByROIOnly(Mat& srcImg, Mat& desImg, Mat& outImg, MatrixXXi& markLabels);
	
	void setOkDeformation(bool isOk){m_isOkDeformation = isOk;}

	void checkBlackRegion(Mat& warpedImg);

	void localDeformation(Mat& leftImg, Mat& rightImg, 
		                  MatrixXXi& BgFgMark, Mat& outPutImg);

	void findLabelSeam(MatrixXXi& labels, Mat& seam);

	void findSeamPosList(MatrixXXi& labels, vector<Point2f>& seam);

	void findSeamExpand(MatrixXXi& labels, vector<Point2f>& seam,MatrixXXi& resLabels);

	void matchingFeaturePs(Mat& leftImg, Mat& rightImg, 
		                   vector<Point2f>& boundary, 
						   vector<Point>& lMPixels,
						   vector<Point>& rMPixels);

	void matchingFeaturePs2(Mat& leftImg, Mat& rightImg, 
		vector<Point2f>& oriBoundary,
		 vector<Point2f>& lboundary, 
		 vector<Point2f>& rboundary, 
		 vector<Point>& lMPixels,
		 vector<Point>& rMPixels);

	void matchingFeatureOverLapLee(Mat& leftImg, Mat& rightImg, 
		                           vector<Point2f>& boundary, 
		                           vector<Point>& lMPixels,
		                           vector<Point>& rMPixels,
								   cv::Rect& exRect);

	bool mathcingExRoiBbx(Mat& lExBbx,Mat&rExBbx,cv::Rect& oriBbx,
		                  vector<Point>& lMPixels,
		                  vector<Point>& rMPixels);

	int getMatchingPoints(Mat& srImg, Mat& tgImg,
		                  vector<CvPoint2D32f>& srPoints, vector<CvPoint2D32f>& tgPoints);

	void matchEdgesCrossSeam(Mat& leftImg, Mat& rightImg, 
		                     vector<Point2f>& boundary, 
							 vector<vector<Point> >& leftcontours,
							 vector<vector<Point> >& rightcontours,
							 vector<IndexType>& lCroEdgeIdx,
							 vector<IndexType>& rCroEdgeIdx,
							 vector<Point>& lMEdges,
							 vector<Point>& rMEdges);

	// obtain the gradient along the seam
	void calculateGradientSeam(Mat& leftImg, Mat& rightImg, vector<Point2f>& boundary,
		                       vector<ScalarType>& lGrad, vector<ScalarType>& rGrad);

	void showSeamPixels(Mat& leftImg, Mat& rightImg,vector<Point2f>& boundary);

	void findEdgeComponent(Mat& leftImg, Mat& rightImg,
		vector<vector<Point> >& leftcontours,
		vector<vector<Point> >& rightcontours);

	void detectEdgesCrossBoundary(Mat& leftImg,vector<vector<Point> >& leftcontours,vector<vector<Point> >& rightcontours,
		                          vector<Point2f>& seam, vector<IndexType>& lCroEIdx, vector<IndexType>& rCorEIdx);

	void combineMatchingInfo(vector<Point>& lPixM,vector<Point>& rPixM,
		                     vector<Point>& lEdgeM,vector<Point>& rEdgeM,
							 vector<MatchingInfo>& finMatchInfo);

	void combineMatching(vector<Point>& lPixM,vector<Point>& rPixM,
		vector<Point>& lEdgeM,vector<Point>& rEdgeM,
		vector<CvPoint2D32f>& srfinMatchInfo,
		vector<CvPoint2D32f>& tgfinMatchInfo);

	void deformRoiWithMathcingInfo(Mat& leftImg, Mat& rightImg,
		                           vector<Point2f>& seam,
								   vector<MatchingInfo>& finM,
								   Mat& outImg);
	void deformRoiWithMathcing(Mat& leftImg, Mat& rightImg,
		                       vector<Point2f>& seam,
		                       vector<CvPoint2D32f>& srfinMatchInfo,
		                       vector<CvPoint2D32f>& tgfinMatchInfo,
		                       Mat& outImg);

	//line preserving for deformation
	void  deformLinesPoints(Mat& leftImg, Mat& rightImg,
		vector<Point2f>& seam,
		vector<CvPoint2D32f>& srfinMatchInfo,
		vector<CvPoint2D32f>& tgfinMatchInfo,
		Mat& outImg);

	void deformROIMatchingLinesPoints(Mat& leftImg, Mat& rightImg,
		vector<Point2f>& seam,
		vector<CvPoint2D32f>& srfinMatchInfo,
		vector<CvPoint2D32f>& tgfinMatchInfo,
		vector<lineSeg>& lLines,
	    vector<lineSeg>& rLines,
		vector<IndexType>& linesMatchInfo,
		Mat& outImg);


	void getLinesSegments(vector<Point2f>& seam,
		                  vector<vector<Point> >& lContours,vector<vector<Point> >& rContours,
						  vector<IndexType>& lIdx,vector<IndexType>& rIdx,
						  vector<lineSeg>& lLines, vector<lineSeg>& rLines);

	void getLinesSegmentCorssSeam(Mat& leftImg,vector<Point2f>& seam,
		vector<vector<Point> >& lContours,vector<vector<Point> >& rContours,
		vector<IndexType>& lIdx,vector<IndexType>& rIdx,
		vector<lineSeg>& lLines, vector<lineSeg>& rLines);

	void getLinesSegmentCorssSeamPs(Mat& leftImg,vector<Point2f>& seam,
		vector<vector<Point> >& lContours,vector<vector<Point> >& rContours,
		vector<IndexType>& lIdx,vector<IndexType>& rIdx,
		vector<lineSeg>& lLines, vector<lineSeg>& rLines,
		vector<Point>& lInterSPs,vector<Point>& rInterSPs);

	void obtainLinesMatching(Mat& leftImg, Mat& rightImg,vector<Point2f>& seam,
		                     vector<lineSeg>& leftLines, vector<lineSeg>& rightLines,
							 vector<IndexType>& lineMatching);

	void obtainLinesHough(Mat& leftImg, Mat& rightImg,vector<Point2f>& seam,
		vector<lineSeg>& leftLines, vector<lineSeg>& rightLines);

	//also obtain the intersection
	void obtainLinesMatchingPs(vector<Point2f>& seam,
		vector<lineSeg>& leftLines, vector<lineSeg>& rightLines,
		vector<Point>& lPoints,vector<Point>& rPoints);

	void sampleLines(vector<Point2f>& seam,vector<Vec4i>& oriLines, vector<lineSeg>& dSLines);

	bool isLocatedBBX(Point& minP, Point& maxP, Point& stP, Point& edP);

	bool isInnerLine(Point& minP, Point& maxP, Point& stP, Point& edP);

	void showLines(Mat& leftImg, Mat& rightImg, vector<Point2f>& seam, vector<lineSeg>& lLines, vector<lineSeg>& rLines);

	void calculateDisMatrix(Mat& lImg, Mat& rImg, vector<Point2f>& boundary,
		                    vector<ScalarType>& ldomainVal,vector<IndexType>& ldomainIdx,
							vector<ScalarType>& rdomainVal,vector<IndexType>& rdomainIdx,
							vector<vector<ScalarType> >& disM);
	void calculateDisMatrix2(Mat& lImg, Mat& rImg, 
		vector<Point2f>& oriboundary,
		vector<Point2f>& lboundary,vector<Point2f>& rboundary,
		vector<ScalarType>& ldomainVal,vector<IndexType>& ldomainIdx,
		vector<ScalarType>& rdomainVal,vector<IndexType>& rdomainIdx,
		vector<vector<ScalarType> >& disM);

	void dynamicProMatching(vector<vector<ScalarType> >& disM,vector<Point2f>& boundary,
		                    vector<IndexType>& ldomainIdx,vector<IndexType>& rdomainIdx,
							vector<Point>& lMPixels,vector<Point>& rMPixels);

	void dynamicProMatching2(vector<vector<ScalarType> >& disM,
		vector<Point2f>& lboundary,vector<Point2f>& rboundary,
		vector<IndexType>& ldomainIdx,vector<IndexType>& rdomainIdx,
		vector<Point>& lMPixels,vector<Point>& rMPixels);

	void boundingboxSeam(vector<Point2f>& boundary, Point& minPos, Point& maxPos, ScalarType& diaLen);

	void showPixMatching(Mat& leftImg, Mat rightImg,vector<Point>& lPos, vector<Point>& rPos);

	void showPixMatching(Mat& leftImg, Mat rightImg,vector<CvPoint2D32f>& lPos, vector<CvPoint2D32f>& rPos);

	void getBoundaryGradient(Mat& leftImg, Mat rightImg,vector<Point2f>& boundary,
		                     vector<ScalarType>& lGradBoundary, vector<ScalarType>& rGradBoundary);

	void getBoundaryGradient2(Mat& leftImg, Mat rightImg,
		vector<Point2f>& lboundary, vector<Point2f>& rboundary,
		vector<ScalarType>& lGradBoundary, vector<ScalarType>& rGradBoundary);	

	void downSampleCandidates(vector<ScalarType>& lOriGradVal,vector<IndexType>& lOriIdx,vector<ScalarType>& rOriGradVal,vector<IndexType>& rOriIdx,
		                      vector<ScalarType>& ldownGradVal,vector<IndexType>& ldownIdx,vector<ScalarType>& rdownGradVal,vector<IndexType>& rdownIdx);

	void downSampleLargeCandidates(vector<ScalarType>& lOriGradVal,vector<IndexType>& lOriIdx,vector<ScalarType>& rOriGradVal,vector<IndexType>& rOriIdx,
		vector<ScalarType>& ldownGradVal,vector<IndexType>& ldownIdx,vector<ScalarType>& rdownGradVal,vector<IndexType>& rdownIdx);

	void drawMaxPs(Mat& lImg,Mat& rImg, vector<Point2f>& boundnary, vector<IndexType>& lMaxIdx,vector<IndexType>& rMaxIdx);

	void drawMaxPs2(Mat& lImg,Mat& rImg, vector<Point2f>& lboundnary, vector<Point2f>& rboundnary, vector<IndexType>& lMaxIdx,vector<IndexType>& rMaxIdx);


	void downSamplePtslist(Mat& lImg,vector<Point2f>& oriList,vector<Point2f>& downList);

	void downsamplePtsList42(Mat& lImg,Mat& rImg,
		                     vector<Point2f>& oriList,
		                     vector<Point2f>& ldList,vector<Point2f>& rdList);

	void downsamplePtsUsingDifferenceHSV(Mat& lImg,Mat& rImg,
		vector<Point2f>& oriList,
		vector<Point2f>& ldList,vector<Point2f>& rdList);

	void continueDownSample(Mat& lImg,Mat& rImg,
		vector<Point2f>& oriList,
		vector<Point2f>& ldList,vector<Point2f>& rdList);

	void downsamplePtsList42SIFT(Mat& lImg,Mat& rImg,
		vector<Point2f>& oriList,
		vector<Point2f>& ldList,vector<Point2f>& rdList);

	void combine2DownSample(Mat& lImg,Mat& rImg,
		vector<Point2f>& oriList,
		vector<Point2f>& ldList,vector<Point2f>& rdList);

	void descCostDistance(Mat& lImg, Mat& rImg,
                          vector<Point2f>& boundary,
		                  vector<IndexType>& ldomainIdx, 
						  vector<IndexType>& rdomainIdx, 
		                  MatrixXX& desCost);

	void descCostDistance2(Mat& lImg, Mat& rImg,
		vector<Point2f>& lboundary,
		vector<Point2f>& rboundary,
		vector<IndexType>& ldomainIdx, 
		vector<IndexType>& rdomainIdx, 
		MatrixXX& desCost);

	bool expanROI(Mat& inImg,Point& oriMin, Point& oriMax,Point& newMin,Point& newMax, IndexType nSlide);

	bool expanROI(Mat& inImg,Point& oriMin, Point& oriMax,Point& newMin,Point& newMax, 
		          IndexType nSlide,vector<bool>& flagSkip);

	void updateCoorOfMatching(Point& minPs,
		                      vector<CvPoint2D32f>& srfinMatchInfo,
		                      vector<CvPoint2D32f>& tgfinMatchInfo);

	void updateCoorOfLines(Point& minPs, vector<lineSeg>& lLines, vector<lineSeg>& rLines);

	void expandSeam(MatrixXXi& labels, IndexType band, Mat& resGrayImg);

	void expandSeamExpand(MatrixXXi& labels, IndexType band, Mat& resGrayImg,MatrixXXi& resLabels);

	void refineDownedPtslist(Mat& lImg,Mat& rImg,
		vector<Point2f>& oriList,
		vector<Point2f>& ldList,vector<Point2f>& rdList,
		vector<Point2f>& ldListref,vector<Point2f>& rdListref);


	void brushHandRegionAsBlack(Mat& oriImg, MatrixXXi& labels, Mat& outImg);

	// 11-08
	//find a initial matching results along the seam.	
	//automatic to detect the matching pixel.

	void findInitialMatchingAlongSeam(vector<Point2f>& ptslist, vector<CvPoint2D32f>& lmatchPs,vector<CvPoint2D32f>& rmatchPs);

	void MatchingLineInterSectionPs(vector<Point2f>& ptslist,
	    vector<Point>& lPoints,vector<Point>& rPoints,
		vector<CvPoint2D32f>& lmatchPs,
		vector<CvPoint2D32f>& rmatchPs);

	void goDownsample(vector<Point2f>& lObPs,
		              vector<Point>& lLinePs);

	void showInitialInputPs(vector<Point>& lPoints,vector<Point>& rPoints);

	void dynamicInterSPsMatching(vector<Point2f>& ptslist,vector<Point>& lPoints,vector<Point>& rPoints,
		vector<CvPoint2D32f>& lmatchPs, vector<CvPoint2D32f>& rmatchPs, bool isReves);

	void calculateDisTwoSets(vector<Point>& lPoints,vector<Point>& rPoints, 
		                     MatrixXX& disMat);

	ScalarType p2pDis(Point& lPs,Point& rPs);

	void dynamicProgramPsMatching(vector<vector<ScalarType>>& disVec,vector<Point>& lPoints,vector<Point>& rPoints,
		vector<IndexType>& lmatchPsIdx, vector<IndexType>& rmatchPsIdx);

	void drawInitialMatching(Mat& bg);

	void detectLinesUsingHoughLines(Mat& edges, vector<cv::Vec4i>& lines);

	bool LiangBarsky (double edgeLeft, double edgeRight, double edgeBottom, double edgeTop,   // Define the x/y clipping values for the border.
		double x0src, double y0src, double x1src, double y1src,                 // Define the start and end points of the line.
		double &x0clip, double &y0clip, double &x1clip, double &y1clip);

	void gloDeformRoi(Mat& roiL,Mat& roiR,vector<CvPoint2D32f>& lMPs,vector<CvPoint2D32f>& rMPs, Mat& outImg);

	void gloDeformRoiwithLines(Mat& roiL,Mat& roiR,vector<CvPoint2D32f>& lMPs,vector<CvPoint2D32f>& rMPs, Mat& outImg);

	void obtainAllLines(Mat& roiL, Mat& roiR,vector<lineSeg>& leftLines, vector<lineSeg>& rightLines);

	void deforAllLines(Mat& roiL, Mat& roiR, vector<CvPoint2D32f>& lMPs,vector<CvPoint2D32f>& rMPs,
		               vector<lineSeg>& leftLines, vector<lineSeg>& rightLines,Mat& outImg);
	
	void transLines2Points(vector<Vec4i>& lines, ScalarType disLen, vector<lineSeg>& lineseg);

signals:
	//void outPutAnImage(QImage* showImg);
	void outPutAnImage(bool isRight, cv::Mat& showImg);
	void outPutPureImage(bool isRIght, QImage& showImg);
	void transferImg(Mat& oriImg,Point& minPs,int heigh,int width,
		             MatrixXX& quadCoor, MatrixXX& textCoor);

	//void test();

public slots:
	void drawEditImg(Mat& oriImg,int height,int width,MatrixXX& quadCoor,MatrixXX& textCoor);

public:
	void initBaseInfo();
	bool isHandPixel(const Vec3b& oriColor);
	IndexType getLabel(IndexType alpha, ScalarType rgb[3]);
	void setInteractiveStatue(bool isInteractive);
	void bubleSort(vector<ScalarType>& oriData,vector<IndexType>& labels,IndexType lSize);
	void setInputImg(Mat& limg,Mat& rImg,MatrixXXi& labels);
public:
	float m_meansFG[N_GAUSS][3];
	float m_varFG[N_GAUSS][3];
	float m_logwFG[N_GAUSS];

	MatrixXXi m_initFGLabel;// the class id after GMM

	float m_meansBG[N_GAUSS][3];
	float m_varBG[N_GAUSS][3];
	float m_logwBG[N_GAUSS];
	MatrixXXi m_initBGLabel;

	float m_beta;
	float m_gWeight;
	
	IndexType m_imgHight;
	IndexType m_imgWidth;

public:
	IndexType m_thresDiff;
	MatrixXXi m_initLabel; //assign the label value/ interactive method && middle results in iteration process  !!!
	bool m_isWithInteractive;
	bool m_isOkDeformation;

private:
	 QImage              m_curShowImg;
	 LongType            m_curFrameIdx;
	 LongType            m_totFrames;
	 cv::Mat             m_curImgData;  
	 cv::VideoCapture&   m_Video;  

private:
	std::vector< std::pair<IndexType,IndexType> >& m_keyPairs; 
private:
	std::vector<cv::Rect> m_sltRect;

private:
	IndexType m_showPairIdx[2];
	IndexType m_curPairIdx;

private:
	//GCoptimizationGridGraph* m_gcImg;
	
private:
	SpectralClusteringThread spClassify;

private:
		AutoSelect* m_autoSelect;
private:
		PieceHomo* m_pieceHomo;
private:
        Mat m_glbHomo;    
		Point m_roiMinPs;

private:
	Mat m_lROI;
	Mat m_rROI;
	MatrixXXi m_labels;

private:
	Mat m_lEdgesGray;
	Mat m_rEdgesGray;

};

#endif