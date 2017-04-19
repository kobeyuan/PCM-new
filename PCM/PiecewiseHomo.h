#ifndef PIECEWISEHOMO_H_
#define PIECEWISEHOMO_H_

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
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "myMesh.h"

#include <Eigen/IterativeLinearSolvers>

#include <Eigen/src/IterativeLinearSolvers/LeastSquareConjugateGradient.h>
#include <Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>

#include "DrawTexture.h"

using namespace std;
using namespace cv;


inline double bilinear_interp(double x, double y,
							  double v11, double v12,
							  double v21, double v22) 
{
	return (v11 * (1 - y) + v12 * y) * (1 - x) + (v21 * (1 - y) + v22 * y) * x;
}

class PieceHomo : public QThread
{
	Q_OBJECT
public:
	void run() Q_DECL_OVERRIDE;
signals:
	void finsh_compute();

public:
	PieceHomo(Mat& _srimg, Mat& _tgimg,IndexType _res);

// 	PieceHomo(Mat& _srimg, Mat& _tgimg, 
// 		      vector<CvPoint2D32f>& _srPs,
// 			  vector<CvPoint2D32f>& _tgPs,
// 		      IndexType _res);

	~PieceHomo(){}

public:
	void alignmentPiece(Mat& outImg);//using features from itself

	void deformMatchingImg(Mat& outImg);

	void deformMatchingLinesPoints(Mat& outImg);

	void featureBasedAlignment(Mat& interImg);

	void calFeaPoUsingFlow();

	void constructGridMesh();

	void initFeatures(vector<CvPoint2D32f>& srPs, vector<CvPoint2D32f>& tgPs);

	void initWarp();

	void initFeatureCoordinate();

	void initHomo(Mat& _homo);

	void warpingGrid();

	void warpingGridwithLines();

	void warpingPiecewiseHomo();

	void estimateHomo();

	void homography(vector<CvPoint2D32f>& srPoints, 
		           vector<CvPoint2D32f>& tgPoints, Mat& _homo);
	void transHomo(Point2f& oriPs, Point2f& transfPs, Mat& homo);

	void renderWarpedImg();

	void bilinearInterpo(Mat& resItp);
	
	void combineHomos2Img(Mat& _outImg);

	void initLines(vector<lineSeg>& lLines, vector<lineSeg>& rLines, vector<IndexType>& matchingInfo);

	void drawLines();

signals:
    void drawDeformedImg(Mat& oriImg,int height,int width, MatrixXX& quads,MatrixXX& texCoors); 

public:

	CvPoint2D32f transform(Mat* R, ScalarType x, ScalarType y);
	void refineKeyps(Mat& cufImg, vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs);
	bool matchingDescriptor(const vector<KeyPoint>& queryKeyPoints, 
		                    const vector<KeyPoint>& trainKeyPoints, 
							const Mat& queryDescriptors,
							const Mat& trainDescriptors, 
							Ptr<DescriptorMatcher>& descriptorMatcher, 
							Mat& homo, 
							vector<DMatch>& matchPs,
							bool enableRatioTest /* = true */);

	bool createDetectorDescriptorMatcher(const string& detectorType,
		                                 const string& descriptorType,
										 const string& matcherType, 
										 Ptr<FeatureDetector>& featureDetector, 
										 Ptr<DescriptorExtractor>& descriptorExtractor, 
										 Ptr<DescriptorMatcher>& descriptorMatcher );
	void mat2Qimage(const Mat& srcMat, QImage& desQImage);
private:
   Mat& m_srImg;
   Mat& m_tgImg;
   IndexType m_res;
   IndexType m_width;
   IndexType m_heigh;
   Cquadmesh* m_grid;

   vector<CvPoint2D32f> m_srFeaturePs;
   vector<CvPoint2D32f> m_tgFeaturePs;

   // for lines
private:
	vector<lineSeg> m_srLines;
	vector<lineSeg> m_tgLines;
	vector<IndexType> m_linesMathingInfo;

	vector<vector<vector<IndexType> >> m_linesNodeIdx;
	vector<vector<vector<ScalarType> >> m_lineNodeCoor;

private:
	vector< vector<int>> _np;        // neighbor points of feature points
	vector< vector<float>> _cd;   // coordinate of feature points in quad mesh  
	Mat _H;
	//vector<vector<CvPoint2D32f>> _corners;  // 4 corners
	vector<CvPoint2D32f> _corners;  // 4 corners
	vector<CvPoint2D32f> _warpv; 
	vector<CvPoint2D32f> _texCoord; 
	CvPoint bL, tR, vbL, vtR;   

private:
	vector<Mat> m_homo;//homo matrix for all quad
	vector<Mat> m_oldHomo;
	Mat m_glbHomo;

	MatrixXX m_oriCtlCoor;
	MatrixXX m_oldCtlCoor;
	MatrixXX m_newCtlCoor;

private:
	dTexture* m_rendWid;
	Mat m_quadIdxPix;
};

#endif
