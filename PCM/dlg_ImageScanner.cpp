#include "dlg_ImageScanner.h"

KeyFrameCapture::KeyFrameCapture()
{
	ui.setupUi(this);

	m_leftDelay = m_rightDelay = m_allDelay = 1;
	m_leftbutton =  false;
	m_rightbutton = false;

	ui.ROK->setVisible(false);

	initConnect();

	m_ImgLabel.push_back(ui.SKey1);
	m_ImgLabel.push_back(ui.SKey2);
	m_ImgLabel.push_back(ui.SKey3);
	m_ImgLabel.push_back(ui.SKey4);
	m_ImgLabel.push_back(ui.SKey5);

}

KeyFrameCapture::~KeyFrameCapture()
{

}

void KeyFrameCapture::initConnect()
{
	connect(ui.LOK, SIGNAL(clicked()), this, SLOT(pauseCameraLeft()));
	connect(ui.ROK, SIGNAL(clicked()), this, SLOT(pauseCameraRight()));
}

void KeyFrameCapture::playVideo()
{
	readCameraData();
}

void KeyFrameCapture::pauseCameraLeft()
{
	m_leftbutton = true;
	readCameraData();

// 	if (m_leftbutton)
// 	{
//     	//m_leftDelay = -1;
// 		m_leftbutton = false;
// 		m_rightbutton = true;
// 		//openLeftCamera();
// 
// 		//m_allDelay = -1;
// 		readCameraData();
// 
// 	}else
// 	{
// 		//m_leftDelay = 1;
// 		m_leftbutton = true;
// 		m_rightbutton = false;
// 		//openLeftCamera();
// 		//m_allDelay = 1;
// 		readCameraData();
// 	}

}

void KeyFrameCapture::pauseCameraRight()
{
	m_rightbutton = true;
	readCameraData();

// 	if (m_rightbutton)
// 	{
// 		//m_rightDelay = -1;
// 		//openRightCamera();
// 		//m_allDelay = -1;
// 
// 		m_rightbutton = false;
// 		m_leftbutton = true;
// 		readCameraData();
// 	}
// 	else
// 	{
// 		//m_rightDelay = 1;
// 		//openRightCamera();
// 		//m_allDelay = 1;
// 
// 		m_rightbutton = true;
// 		m_leftbutton = false;
// 		readCameraData();
// 
// 	}
}

void KeyFrameCapture::readCameraData()
{
	m_videoDataLeft.release();
	m_videoDataLeft = cv::VideoCapture(1);

	Mat tempImg,comImg;

	Mat tempLeft, tempRight;

	 while (1)
	 {	 
		 m_videoDataLeft >> tempImg;

		 if (!m_leftbutton && !m_rightbutton)
		 {
			 showCameraWin(tempImg, 0);
			 //right is empty
		 }

		 if (m_leftbutton)//left is sure, keep this left image
		 {
		    //showCameraWin(tempImg, 0);
			 if (m_curLeftImg.empty())
			 {
				 tempImg.copyTo(m_curLeftImg);
				 showCameraWin(m_curLeftImg, 0);

			 }else
			 {
				 translucentImages(m_curLeftImg, tempImg, 0.5, comImg);
				 showCameraWin(comImg, 1);
			 }

			ui.LOK->setVisible(false);
			ui.ROK->setVisible(true);

		 }

		 if (m_rightbutton)//the right image is sure save this pair
		 {
		     showCameraWin(tempImg, 0);
			 tempImg.copyTo(m_curRightImg);
			 //tempImg.copyTo(tempRight);

			 m_keyFrames.push_back(std::make_pair(m_curLeftImg, m_curRightImg));
			 //m_keyFrames.push_back(std::make_pair(tempLeft, tempRight));

			 m_curLeftImg = Mat();
			 m_curRightImg = Mat();


			 previewKeyframes();

			 m_rightbutton = false;
			 m_leftbutton = false;

			 ui.LOK->setVisible(true);
			 ui.ROK->setVisible(false);

		 }

		 imshow("Camera", tempImg);
		 waitKey(1);
	 }

}


void KeyFrameCapture::openLeftCamera()
{
	m_videoDataLeft.release();
	m_videoDataLeft = cv::VideoCapture(0);

	Mat curImgLeft, curImgRight, tempImg;

	while (1)
	{
		m_videoDataLeft >> curImgLeft;
		showCameraWin(curImgLeft, 0);
		imshow("CameraLeft", curImgLeft);
		waitKey(m_leftDelay);

	}
}

void KeyFrameCapture::openRightCamera()
{
	m_videoDataRight.release();
	m_videoDataRight = cv::VideoCapture(0);

	Mat curImgLeft, curImgRight;

	while (1)
	{
		m_videoDataRight >> curImgRight;
		showCameraWin(curImgRight, 1);
		imshow("CameraRight", curImgRight);
		waitKey(m_rightDelay);
	}
}

void KeyFrameCapture::showCameraWin(Mat& curImg, bool isRigWin)
{
	QImage curQimg, maskImg;

	//mat2Qimage(curImg, curQimg);
	Mat tempImg;
	cvtColor(curImg, tempImg, CV_BGR2RGB);
	curQimg = QImage((const unsigned char*)(tempImg.data),
		tempImg.cols, tempImg.rows, QImage::Format_RGB888);

	maskImg = QImage(curQimg.width(), curQimg.height(),
		QImage::Format_ARGB32);

	if (isRigWin)
	{
		ui.SRigWin->setImage(&curQimg);
		ui.SRigWin->setMaskImage(maskImg);
		ui.SRigWin->updateDisplayImage();

	}else
	{
		ui.SLeftWin->setImage(&curQimg);
		ui.SLeftWin->setMaskImage(maskImg);
		ui.SLeftWin->updateDisplayImage();
	}

}

void KeyFrameCapture::mat2Qimage(const Mat& srcMat, QImage& desQImage)
{
	IndexType nChannel = srcMat.channels();

	if (nChannel == 3)
	{
		Mat srcTemp = Mat(srcMat.rows, srcMat.cols, srcMat.type());
		srcMat.copyTo(srcTemp);
		desQImage = QImage(srcTemp.cols, srcTemp.rows, QImage::Format_RGB888);
		memcpy(desQImage.bits(), srcTemp.data, srcTemp.cols*srcTemp.rows * 3 * sizeof(unsigned char));
	}
	else if (nChannel == 4 || nChannel == 1)
	{
		desQImage = QImage((const unsigned char*)srcMat.data, srcMat.cols, srcMat.rows, srcMat.step, QImage::Format_ARGB32);
	}
}

void KeyFrameCapture::translucentImages(Mat& srIMg, Mat& tgImg,
	ScalarType alpha, Mat& resImg)
{
	//assert(alpha >= 0. && alpha <= 1.);

	if (alpha < 0. || alpha > 1.0)
	{
		alpha = 0.5;
	}

	ScalarType beta = 1. - alpha;

	addWeighted(srIMg, alpha, tgImg, beta, 0.0, resImg);

}

void KeyFrameCapture::previewKeyframes()
{
	int kfSize = m_keyFrames.size();

	if (kfSize<= 5)
	{
		for (int i = 0; i < kfSize; ++ i)
		{
			Mat curImg = m_keyFrames[i].first;

			drawPreLeftImage(curImg, i);
		}

	}else
	{
		int preLong = kfSize - 5;
		for (int i = 0; i < 5; ++i)
		{
			Mat curImg = m_keyFrames[i + (preLong)].first;

			drawPreLeftImage(curImg, i);
		}
	}
}

void KeyFrameCapture::drawPreLeftImage(Mat& imgData, int winIdx)
{
	if (winIdx <0 || winIdx >=5)
	{
		return;
	}

	QImage curQimg, maskImg;

	//mat2Qimage(curImg, curQimg);
	Mat tempImg;
	cvtColor(imgData, tempImg, CV_BGR2RGB);
	curQimg = QImage((const unsigned char*)(tempImg.data),
		tempImg.cols, tempImg.rows, QImage::Format_RGB888);

	maskImg = QImage(curQimg.width(), curQimg.height(),
		QImage::Format_ARGB32);

	m_ImgLabel[winIdx]->setImage(&curQimg);
	m_ImgLabel[winIdx]->setMaskImage(maskImg);
	m_ImgLabel[winIdx]->updateDisplayImage();

}

//read a video
//m_videoData = cv::VideoCapture("F://Projects//STOP MOTION//Videos-sample//book-lifting.avi");
//long totalFrameSize = m_videoData.get(CV_CAP_PROP_FRAME_COUNT);
//m_videoData.set(CV_CAP_PROP_POS_FRAMES, i);
// 	QString filepath = "G:\\ScannerVideo\\";
// 	QString videoFilepath = filepath;
// 	string vedioFilename = videoFilepath.toUtf8().constData();
// 	vedioFilename.append("aaa.avi");
// 
// 	writer = VideoWriter(vedioFilename.c_str(), CV_FOURCC('X', 'V', 'I', 'D'), 25, Size(640, 480), true);