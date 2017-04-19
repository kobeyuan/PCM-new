#pragma once

#include "ui_dlg_imagesScanner.h"

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/eigen.hpp"

using namespace cv;

class KeyFrameCapture : public QDialog
{
	Q_OBJECT
// public:
// 	void run() Q_DECL_OVERRIDE;

public:
	KeyFrameCapture();
	~KeyFrameCapture();

public:
	void initConnect();

public:
	void readCameraData();
	void openLeftCamera();
	void openRightCamera();

	void showCameraWin(Mat& curImg, bool isRigWin);
	void mat2Qimage(const Mat& srcMat, QImage& desQImage);
	void translucentImages(Mat& srIMg, Mat& tgImg,
		ScalarType alpha, Mat& resImg);

	void previewKeyframes();
	void drawPreLeftImage(Mat& imgData, int winIdx);

public slots:

    void playVideo();
	void pauseCameraLeft();
	void pauseCameraRight();


public:
	Ui::KeyFrameCapUI ui;
	vector<ImageLabel*> m_ImgLabel;

public:
	cv::VideoCapture m_videoDataLeft;
	cv::VideoCapture m_videoDataRight;
	VideoWriter writer;
	int m_leftDelay;
	int m_rightDelay;
	int m_allDelay;
	bool m_leftbutton;
	bool m_rightbutton;

	Mat m_curLeftImg;
	Mat m_curRightImg;
	vector<pair<Mat, Mat>> m_keyFrames;
};