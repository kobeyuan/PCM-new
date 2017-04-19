#pragma once

/* proceesing a video for keyframe automatic selection*/

#include "GeneratedFiles/ui_dlg_autoKeyPas.h"
#include "StopMotion.h"
#include "newSelectionAlgo.h"
#include "cv.h"

using namespace cv;

#define frame_frame_to_key(f,i) ((f<<16)|i)
#define get_aframe_from_key(k) (k>>16)
#define get_bframe_from_key(k) (k&0xffff)


class DLGAutoSelection : public QDialog
{
	Q_OBJECT
public:
	DLGAutoSelection();
	~DLGAutoSelection(){}

public:
	void initConnect();

	void drawLeftWin(QImage& mask, QImage& input);
	void drawRightWin(QImage& mask, QImage& input);

	void initBaseInfo();

public slots:
   
//play a video
	void playPause();
	void playStop();
	void preFrame();
	void nextFrame();
	void changeCurFrameIdx(int);

private:
	Ui::autoSelectKey ui;

private:
	cv::VideoCapture&   m_video;
	QTimer*             m_timeCtl;
	IndexType           m_delay;
	IndexType           m_rate;
	LongType            m_curFrameIdx;
	LongType            m_totFrames;
	cv::Mat             m_curImgData;
	QImage              m_curShowImg;
	QImage              m_curShowLeftImg;

	bool m_isPlay;
	AutoSelectNew* m_autoSelectAlgo;//using dynamic programming method for selection

};