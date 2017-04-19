#ifndef _DLG_STOPMOTIONASIG_H_
#define _DLG_STOPMOTIONASIG_H_

#include <QtWidgets/QDialog>
#include "ui_Stop-Motion-completion.h"
//#include "ui_dlg_autoselectkeyframes.h"
#include "ui_dlg_autoKeyPas.h"
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

class StopMotionASIG : public QDialog
{
	Q_OBJECT

public:
	StopMotionASIG();
	~StopMotionASIG();

public:
	enum OperStep{Step_ROIS = 0,  Step_HandR, Step_GLOBALM, Step_LocalD,Step_BGGCut,Step_Smooth};
	enum VideoStep{Step_BG = 0, Step_Pair};

public:
	void initConnect();
	void initBaseInfo();
	bool initPairFromImageSeq();

	void setCanva(CanvasUI* _in)
	{
		m_canvas = _in;
	}

	//draw functions
  void drawKeyPair(IndexType pIdx);
  void drawKeyFrame(IndexType fIdx, bool isRight);

  //draw major window using mat data

  void drawLeftWin(QImage& mask, QImage& input);

  void drawRightWin(QImage& mask, QImage& input);

  void initalROIsShow();

  void displayLeftSubWin(Mat& leftWin);
  void displayRightSubWin(Mat& rightWin);
  void displayMiddleResults(Mat& mRes);



  //others
  void mat2Qimage(const Mat& srcMat, QImage& desQImage);


  //show or hidden buttons
  void showButtonsOne();
  void showButtonAfterROISelected();



  //general functions
  void translucentImages(Mat& srIMg, Mat& tgImg, 
	  ScalarType alpha, Mat& resImg);

  void combineTwoRois();//using graph cut method directly
  bool isMaskFromRight();

  void handRemoveByGCut(Mat& leftImg, Mat& rightImg);
  void getSeamFromGraphcut();

  bool getLabesFromMaskROI(QImage& _maskIMg, MatrixXXi& _labels);


  bool globalDeformation(Mat& roiL, Mat& roiR, Mat& outImg);
  bool globalDeformationSym(Mat& roiL, Mat& roiR, Mat& outImg);

  void combineImages(Mat& leftImg, Mat& rightImg);

  bool findClosestPointOnSeam(QPoint& curP, Point& cloPs);

  void bubleSort(vector<ScalarType>& oriData, vector<IndexType>& labels, IndexType lSize);

  void localDeformation();

  void completedwithBG();

  void setting4back2localMatching();

  void showSeqinList();
  void initImaLab();
  void displayListImg(Mat& img, int listId);
public slots:
	void getROIsFromRect();
	void showPairIdx(int idx);

	//above UI functions
	void getToolStatus();

	//about next operation
	void preHandRemovalAction();

	//obtain mask from rois
	void getMaskFromLeftROI();
	void getMaskFromRightROI();

	//undo operation for graph cut

	void undoHandRemoval();
	void undoGloMatching();
	void undoLocalMatching();
	void undoBackgroundReplace();

	void preGlobalMatching();
	void preLocalMatching();
	void preBackgroundReplace();
	void preNextKeyFrame();

	//back operation
	void back2Select();
	void back2HandRemoval();
	void back2GlobalMatching();
	void back2LocalMatching();

	//obtain the global matching points
	void obtainCoorLeft(const QPoint& pos);
	void obtainCoorRight(const QPoint& pos);

	//obtain the local matching points
	void getMatchingPs(vector<QPoint>& mPs);

	//draw warped image
	void drawEditImg(Mat& oriImg, Point& minPs, int height, int width, MatrixXX& quadCoor, MatrixXX& textCoor);

	//draw sub window for local matching points
	void drawSubWin4LocalMatching();

	//zoom
	void zoomSameTime(float& ration, ImageLabel& _panter);

	void transImagesSameTime(float& dx, float& dy, ImageLabel& _panter);

	void showCurPairs();

public:
  cv::VideoCapture&   m_video;
  cv::Mat             m_curImgData; //from video sequences

  bool   m_isVideo;
  CanvasUI* m_canvas;
  bool      m_isInitImSeq;//input sequences, then initialize.

public:
	Ui::StopMotionCom ui;
	vector<ImageLabel*> m_seqInList;
//for automatic selection ui
	Ui::autoSelectKey atui;
public:

private:
	StopMotion* m_sMotion;

	vector< std::pair<IndexType,IndexType> > m_keyPairs;
	IndexType    m_showPairIdx[2]; //current output, the idx in the total sequence display
	IndexType    m_curPairIdx;
    LongType     m_totFrames;

	//inputs for QLabel drawing, only for current results.
	QImage  m_curShowLeftImg;
	QImage  m_maskLeftImg;

	QImage  m_curShowRightImg;
	QImage  m_maskRightImg;

	QImage  m_curShowSubLImg;
	QImage  m_maskSubLImg;

	QImage  m_curShowSubRImg;
	QImage  m_maskSubRImg;

	QImage  m_curShowSubMImg;
	QImage  m_maskSubMImg;

	//stack for mask information
	std::stack<QImage> m_leftmaskImgStack, m_rightmaskImgStack;

	std::stack<QImage> m_leftmaskImgStackBG, m_rightmaskImgStackBG;//for background replacement

	MatrixXXi m_markLeftLabels, m_markRightLabels;

	MatrixXXi m_markLeftLabelsBG, m_markRightLabelsBG;//for background replacement
	//
	vector<cv::Rect>  m_sltRect;//record selected rois

	//image data for each steps
private:
	Mat m_leftROI;
	Mat m_rightROI;
	Mat m_combineROI;
	vector<Mat> m_combineImgHis;//4 local deformation step
	QImage m_iniglo4LocComImg;
	Mat m_comImgSelect;//record the two rois togeter.
	bool m_isGloMatching;
	bool m_isLocMatching;

	Mat m_gloDeformROI;//record the global deformed image
	Mat m_locDeformROI;//record the local deformation image
	Mat m_bgROI;

	QImage m_gloCombineImg;//after global matching,record the global combined image 
	//for display
	QImage  m_leftSubWinQImg;
	QImage  m_rightSubWinQImg;
	QImage  m_combineSubQImg;

	//which roi?
	bool m_isRightMask;
	MatrixXXi m_cutLabels;//graph cut labels--record for the next operation processing
	MatrixXXi m_cutExLabels;
	MatrixXXi m_cutLabelsBG;//record the background replacement labels
	vector<MatrixXXi> _gcutLabels;//record the segmentation labels
	DialCut*  m_dialCut;

	//seams
	vector<Point2f> m_seamPts;//roi frame
	IndexType m_totSeamPsLeft, m_totSeamPsright;

private:
	OperStep m_stepCtrl;

private:
	vector<QPoint> m_gloMatPsLeft, m_gloMatPsRight;
	vector<CvPoint2D32f> m_locMatPsLeft, m_locMatPsRight;
	QPoint m_selectPointLeft;
	QPoint m_selectPointRight;

};

#endif