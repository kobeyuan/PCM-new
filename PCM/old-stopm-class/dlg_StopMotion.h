#ifndef _DLG_STOPMOTION_H_
#define _DLG_STOPMOTION_H_

#include <QtWidgets/QDialog>
#include "ui_Stop-Motion.h"
#include "StopMotion.h"
#include "cv.h"

#include "DialCut.h"
#include <stack>
#include <vector>

#include "AutoSelectionPairs.h"

#include "PiecewiseHomo.h"

#include "Blender/blend.h"

using namespace cv;

#define frame_frame_to_key(f,i) ((f<<16)|i)
#define get_aframe_from_key(k) (k>>16)
#define get_bframe_from_key(k) (k&0xffff)
class CanvasUI;

class StopMotionUI : public QDialog
{
	Q_OBJECT

public:
	StopMotionUI();
	~StopMotionUI();

public:
	void initConnect();
	void initBaseInfo();
	void addPairs();
	void addMatching();

	void drawKeyFrame(IndexType fIdx);
	void drawKeyFrame(IndexType fIdx, bool isRight);
	void progressBar();
	bool initPairFromImageSeq();
	void simpleHandClassify(const Mat* srImg, Mat* tgImg);
	void handMarkerGenerate(const Mat& srImg, MatrixXXi& handMK);//fg-1-hand;bg-0-non-hand;
	//void handMarkerDetect(const Mat& srImg, MatrixXXi& handMK);
	void markersGenDiff(const Mat& srImg, const Mat& tgImg/*, MatrixXXi& diffMK*/);//1-L-hand,2-R-hand;0-non-hand
	bool isHandPixel(const Vec3b& oriColor);
	bool isHandPixelNormal(const Vec3b& oriColor);
	bool isHandPixelNormal2(const Vec3b& oriColor);
	void mat2Qimage(const Mat& srcMat, QImage& desQImage);
	void qImage2Mat(const QImage& srcQImage,Mat& desMat);
	void writeImages();
	void wirteBackground();
	void initStopMotionClass();
	void assignInitialLabels();

	void translucentImages(Mat& srIMg, Mat& tgIMg, 
		                   ScalarType alpha, Mat& resImg);

	void testGraphCut(Mat& inputIMg);

	void checkStillPixels(MatrixXXi& outPutMark, MatrixXXi& stillPixels);

	void initInteracrionLabels();

	bool checkTrainData(MatrixXXi& initLabels);

	void updateLeftWindowImage(Mat& warpedROI);

	void updateLeft(Mat& gcutImg);

	void updateRight(Mat& freImg);

	void updateLeftAsBG();
	void updateRightAsBG();

	void possionCompute();

	void changeLabelsType();

	//with the background image
	void changeLabelsTypeFinal();

	void possionImages(Mat& leftImg, Mat& rightImg);

	void finPossionImages(Mat& bgImg, Mat& leftImg, Mat& rightImg);

	void saveSmoothImage(unsigned char* res);

	void findClosestPointOnSeam(QPoint& curP, Point& cloPs);

	PieceHomo*& getPieceHomo()
	{
		return m_pieceHomo;
	}	
	void bubleSort(vector<ScalarType>& oriData,vector<IndexType>& labels,IndexType lSize);

	void completionWithBG();

	void exRect(Rect& oriR, Rect& exR, IndexType nSkip);

	void displayMiddleResults(Mat& mRes);

	void showMiddleCombine(Mat& left,Mat& right);


	bool isMaskFromRight();

	void gcAfterDeformation(Mat& leftImg, Mat& rightImg, Mat& outImg, bool isRightMask);

	void handRemoveByGCut();

	void updateCurrentImage(vector<Point2f>& ptsList, Mat& outPutImg,vector<CvPoint2D32f>& lmatchPs,vector<CvPoint2D32f>& rmatchPs);

	void findNearestLine();

	ScalarType p2LineSegDis(QPoint& oP, CvPoint2D32f& lsp, CvPoint2D32f& lep);

	//new global transform
	void showROIs(Mat& lROI,Mat& rROI);

	void showOriROIs();

	bool globalDeformation(Mat& roiL, Mat& roiR, Mat& outImg);


	void initalROIsShow();

public slots:
		void  run(bool b)
		{
			m_sMotion = new StopMotion(m_keyPairs);

			connect(m_sMotion, SIGNAL(finished()), m_sMotion, SLOT(deleteLater() ) );

			m_sMotion->start();
		}

	void playPause();
	void playStop();
	void preFrame();
	void nextFrame();
	void makeSlow();
	void makeFast();
	void selectKeyFrameA();
	void selectKeyFrameB();

	//interaction matching
	void selectLeftFeatures();
	void selectRighFeatures();
	void feaIdxChangesLeft(int);
	void feaIdxChangesRight(int);


	void changeCurFrameIdx(int);
	void showPairIdx(int);
	void combinePairImages();
	void setROIs();
	void getROIsFromSign();

	void displayAnImage(bool isRight, cv::Mat& showImg);
	void disPlayPureImage(bool isRight, QImage& showImg);
	void handDectect();
	void selectBackground();
	void showBackGround();
	void savePairs();

	void showCoorRGB(const QPoint& pos, const QRgb& rgb_);
	void showLabels(const QPoint& pos, const QRgb& rgb_);

	void obtainCoor(const QPoint& pos);

	void obtainCoorLeft(const QPoint& pos);

	void obtainCoorRight(const QPoint& pos);

	void delCurLine();

	void updateGraphweight(int);
	void updateTresDifference(int);
	void afterCombine();
	void initTwoRegion(Mat& inputImg, MatrixXXi& handMark);

	void getStatusTool();
	void getMaskFromImages();

	/* do not care about bg or fg*/
	void getMaskFromImage(QImage& _maskIMg, MatrixXXi& _labels);
	void getMaskFromLeftImages();
	void showTransForSelection(int curFrame);

	void undoStrokeDraw();

	void okDeformation();

	void tryDeformROIOnlyMethod();

	void setDeletePress(){m_isPressDel = true;}

	void fuseTwoROIs();

	void newFuseTwoROIs();

	//void combineImages(MatrixXXi& cutLabels);
	void autoSelectionPair();

	void assignFrameIdx(int);

	// auto selection
	void setBackgId(int);
	void setstartId(int);
	void setLen(int);
	void setBand(int);
	void pSmooth();
	void stopPSmooth();

	void isOpticalflow();

	void drawEditImg(Mat& oriImg,Point& minPs, int heigh,int width,MatrixXX& quadCoor,MatrixXX& textCoor);

	void deformImgInteraction();

	void setCanva(CanvasUI* _in)
	{
		m_canvas = _in;
	}

	void getMatchingPs(vector<QPoint>&);

	void setLeftWindowasBG();
	void setRightWindowasBG();

	void fullScreen();

	void detectFGObjects();

	void deleteGloMatchingPsLeft(const QPoint& pos);

	void deleteGloMatchingPsRight(const QPoint& pos);

signals:  
	void theSecondCombinationStep(Mat& roiR, Mat& outImg);

public:
	StopMotion* m_sMotion;
	bool   m_isVideo;

public:
    AutoSelect* m_autoSelect;

private:
	Ui::StopMotion ui;

private:
	QTimer*             m_timeCtl;
	QImage              m_curShowImg;
	QImage              m_curShowLeftImg;
	QImage              m_curMiddleImg;

	QImage              m_lroi;
	QImage              m_rroi;//for global registration 

	std::stack<QImage>  m_rightmaskImgStack;
	std::stack<QImage>  m_leftmaskImgStack;

	QImage              m_rmaskImage;
	QImage              m_lmaskImage;

	IndexType           m_delay;
	IndexType           m_rate;   
	LongType          m_curFrameIdx;
	LongType          m_totFrames;
	cv::Mat             m_curImgData;
	cv::VideoCapture&   m_video;

	bool                m_isPlay;
	bool                m_isInitImSeq;
	bool                m_isDealLeftImg;
	bool                m_isOKDeformation;
	bool                m_isPressDel;
private:
	vector< std::pair<IndexType,IndexType> > m_keyPairs;
	IndexType    m_keyFrame[2]; // input from video
	IndexType    m_showPairIdx[2]; //output, the idx in the total sequence display, in sub-window

	IndexType    m_curPairIdx;

private:
	vector<cv::Rect>  m_sltRect;
	IndexType m_backGroundIdx;
	ScalarType m_gWeight;
	IndexType m_thresDiff;

	bool m_isOpticalFlow;

private:
	MatrixXXi m_initLabels;// in order to graph cut process--only for ROI
	MatrixXXi m_finalLabels;
	MatrixXXi m_markLabels;// markers from interactive method- entire image
	MatrixXXi m_markLeftLabels;
	MatrixXXi m_diffLabels; // the size of diffLabels equals to the size of ROI.

private:
    IndexType m_bgId;
	IndexType m_srtId;
	IndexType m_lenSeq;
	IndexType m_band;

public:
	Mat m_leftROI;
	Mat m_rightROI;
	Mat m_dfLeftROI; //with the same size of leftROI, registration
	Mat m_gcLeftROI;//temp image for after graph cut processing.
	Mat m_gMatchingImg;
	DialCut*  m_dialCut;

	PieceHomo* m_pieceHomo;

	Mat m_initCurFrame;

private:
	Adrien::GlobalBlender* _blender;
	ImageAbs _blendedImage;
	ImageAbs* _blendTemp;
	vector<ImageAbs*> _inputImage;
	ushort* _labels;

	vector<MatrixXXi> _gcutLabels;
	//for interactive matching processing
public:
	IndexType m_lfeaIdx;
	IndexType m_rfeaIdx;
	IndexType m_totSeamPsLeft,m_totSeamPsright;
	IndexType m_totEdgeLeft,m_totEdgeRight;

	vector<IndexType> m_lMatchingPs;
	vector<IndexType> m_rMatchingPs;

	vector<CvPoint2D32f> m_lMPixels;
	vector<CvPoint2D32f> m_rMPixels;
	vector<Point2f> m_seamPts;//roi frame
	MatrixXXi m_cutLabels;
	Mat m_gcutImg;//for draw circles--global image!

	CanvasUI* m_canvas;

private:
	bool m_isRightMask; //using which mask image for graph cut? default as right.
	bool m_isLeftBg;//set the left window as the background image.
	bool m_isRightBg;

private:
	bool m_isFullS;
private:
    QPoint m_delPCoor;//for find a closest line.
	QPoint m_selectPointLeft;
	QPoint m_selectPointRight;

	IndexType m_delLineIdx;

	vector<QPoint> m_gloMatPsLeft,m_gloMatPsRight;
    Mat m_lROIshow;
	Mat m_rROIshow;//bgr
};

#endif