#ifndef AUTOSELECTIONPAIRS_H_
#define AUTOSELECTIONPAIRS_H_

#include <QThread>
#include "QImage"
#include "cv.h"
#include "SingleFrame.h"
#include "SingleFrameSet.h"

#include "basic_types.h"

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/eigen.hpp"


#include "spectral_clustering.h"

#include <vector>

using namespace std;
using namespace cv;

struct flowPoint;
struct Shot;

class AutoSelect : public QThread
{
	Q_OBJECT
public:
	void run() Q_DECL_OVERRIDE;
signals:
	void finsh_compute();

public:

	AutoSelect(IndexType _curFrame, IndexType _totFrame): m_video(SingleFrameSet::get_instance().getTotVideo())
	{
		m_curFrameIdx = _curFrame;

		m_totFrames = _totFrame;

		if (m_video.get(CV_CAP_PROP_FRAME_HEIGHT) && m_video.get(CV_CAP_PROP_FRAME_WIDTH))
		{
			m_rows = m_video.get(CV_CAP_PROP_FRAME_HEIGHT);
			m_cols = m_video.get(CV_CAP_PROP_FRAME_WIDTH);
		}

		m_band = 2;
	}

	~AutoSelect(){};

public:
	void visOptialFlow(IndexType _start, IndexType _end);
	void calculateFlow(Mat& _curF, Mat& _nFrame, IndexType curIdx);
	void calculateFlow(Mat& _curF, Mat& _nFrame, Mat& flow, IndexType curIdx);
	void motionToColor(Mat flow, Mat& vColor);
	void makecolorwheel(vector<Scalar> &colorwheel);
	void denoiseFlow(Mat& flow);
	//using ransac method for denoise
	void ransacFlow(Mat& _flow);
	void computerRansac(vector<flowPoint>& _pixPoints);
	void samplePoint(vector<flowPoint>& _pixPoints, IndexType spSize, Vec2f& ctr);
	void estimateCenter(vector<flowPoint>& _pixPoints, Vec2f& oldCtr, Vec2f& newCtr, ScalarType thres);
	void updateFlowWDNoise(Mat& flow, vector<flowPoint>& _pixels);

	void normaliFlow(Mat& flow, IndexType fId);
	void findEdgeOfNormFlow(Mat& flow, IndexType fId);
	
	void setBand(int _band){m_band = _band;}

	//classify
	void labelingOneProcess(IndexType _start, IndexType _end, vector<IndexType>& _labels);
	void calculateFeatures(vector<Mat>& inputData, MatrixXX& smtMat);

	ScalarType calcuFlowDis(Mat& srFlow, Mat& tgFlow);
	ScalarType calcuUnDirFlowDis(Mat& srFlow, Mat& tgFlow);
	ScalarType dis2Similar(ScalarType dis);
	void dis2Similar(MatrixXX& _dis, MatrixXX& _simi);

	//draw the area of the foreground at each frame

	void areaForeground(IndexType backgIdx,IndexType stId, IndexType Len);

	void areaForegWithRect(IndexType backgIdx,IndexType stId, 
		                   IndexType Len,cv::Rect& _roi);

	void calculateForeG(IndexType backgIdx,IndexType stId, IndexType fLen, std::vector<ScalarType>& pixels);
	
	ScalarType foreGPixels(Mat& backImg, Mat& curImg);

	ScalarType foreGPixelsRect(Mat& backImg, Mat& curImg,cv::Rect& _roi);

	void calculateForeGWithRect(IndexType backgIdx,IndexType stId, IndexType fLen, 
		                        cv::Rect& _roi, std::vector<ScalarType>& pixels);

/*<<<<<<< HEAD*/
	void smoothCurve(std::vector<ScalarType>& pixels);

	void truncationCurves(std::vector<ScalarType>& ptslist, ScalarType threshold);

	void truncationCurves(std::vector<ScalarType>& ptslist, ScalarType threshold,
		                  std::vector<ScalarType>& aftlist,vector<IndexType>& recIdx);
/*=======*/
	// statistic the optical flow information
	void areaFlowVis(IndexType backgIdx,IndexType stId, IndexType Len);
	void calculateFlowForeG(IndexType backgIdx,IndexType stId, IndexType Len,vector<ScalarType>& nPixels);
	ScalarType calFlow2Stat(Mat& fFrame, Mat&sFrame);
	ScalarType calFlow2Stat(Mat& fFrame, Mat&sFrame, Mat& outputFlow);
	ScalarType calRationFlow(Mat& flow, ScalarType thres);

	//with rectangle information
	void areaFlowVisRect(IndexType backgIdx,IndexType stId,
		                 IndexType Len,cv::Rect& _roi);
	void calculateFlowForeGRect(IndexType backgIdx,IndexType stId, IndexType Len,
		                        vector<ScalarType>& nPixels,cv::Rect& _roi);


	//  deal with the curves
	void smoothCurves(vector<ScalarType>& sgls, IndexType band);
//<<<<<<< HEAD
	void piecewiseCurve(vector<ScalarType>& sgls, IndexType band);

	bool isLocalMin(IndexType pos, vector<ScalarType>& sgls, IndexType band);
	bool isLocalMinMax(IndexType pos, vector<ScalarType>& sgls, IndexType band);
// =======
// >>>>>>> origin/master
// >>>>>>> origin/master

	void detectMinValue(vector<ScalarType>& sgls, IndexType band, vector<bool>& localLab);
	void detectMinMax(vector<ScalarType>& sgls, IndexType band, vector<bool>& localLab);

	//automatic selections
	void splitVideo2Shots(IndexType startFrame, vector<IndexType>& recordMixMax, vector<Shot>& shots);
	void autoGetPairs(vector<Shot>& shots, vector<std::pair<IndexType,IndexType>>& pairs);
	void selectWithRegular(Shot& oneShot, std::pair<IndexType,IndexType>& keyPair);
	void selectWithAlinment(Shot& oneShot, std::pair<IndexType,IndexType>& keyPair);
	void calDisMatrix(Shot& oneShot, IndexType fRange, MatrixXX& disMat,std::pair<IndexType,IndexType>& keyPair);

	void findPairFormMat( MatrixXX& disMat, std::pair<IndexType,IndexType>& keyPair);
	ScalarType calAlignError(Mat& srImg, Mat& tgImg);
	ScalarType backProjectionError(vector<Point2f>& srCoor, vector<Point2f>& tgCoor, Mat&homo);

	void savePairsafterSelection(vector<std::pair<IndexType,IndexType> >& keyPairs);
	
	void detectMinMaxint(vector<ScalarType>& sgls, IndexType band, vector<IndexType>& localLab);

	IndexType islocalMinMaxint(IndexType pos, vector<ScalarType>& sgls, IndexType band);

	IndexType areaForegWithRectSta(IndexType backgIdx,IndexType stId, 
		                           IndexType Len,cv::Rect& _roi);
public:
	void saveOpticalFlow(IndexType _startFrame,vector<Mat>& _flow);

	void readOpticalFlow(IndexType _startFrame,vector<Mat>& _flow);

	bool isDifferent(Vec3b& bgPix, Vec3b& cufPix);

private:

	cv::VideoCapture&   m_video; 
	LongType            m_curFrameIdx;
	LongType            m_totFrames;
	cv::Mat             m_curFrameData;

	IndexType m_rows;
	IndexType m_cols;

	IndexType m_nbiter;
	SpectralClusteringThread spClassify;
private:
	IndexType m_band;
	cv::Rect  m_roi;
};

#endif