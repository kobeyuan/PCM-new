#include "dlg_autoselection.h"

DLGAutoSelection::DLGAutoSelection():m_video(SingleFrameSet::get_instance().getTotVideo())
{
	ui.setupUi(this);
	m_timeCtl = new QTimer(this);
	m_curFrameIdx = 0;

}

void DLGAutoSelection::initConnect()
{
	connect(m_timeCtl, SIGNAL(timeout()), this, SLOT(nextFrame()));

	connect(ui.playAndPause, SIGNAL(clicked()), this, SLOT(playPause()));

	connect(ui.StopPlay, SIGNAL(clicked()), this, SLOT(playStop()));

	connect(ui.previousFrame, SIGNAL(clicked()), this, SLOT(preFrame()));

	connect(ui.inVideoView, SIGNAL(valueChanged(int)), this, SLOT(changeCurFrameIdx(int)));

	//connect(ui.inVideoView, SIGNAL(valueChanged(int)), this, SLOT(showTransForSelection(int))); (ui.NextFrame, SIGNAL(clicked()), this, SLOT(nextFrame()));


}

void DLGAutoSelection::nextFrame()
{
	//assert(m_curFrameIdx <= m_totFrames);

	if (m_curFrameIdx >= m_totFrames)
	{
		Loggger << "End position.\n";
		return;
	}

	m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx);

	m_video >> m_curImgData;

	if (!m_curImgData.empty())
	{
		Mat tempImg;

		cvtColor(m_curImgData, tempImg, CV_BGR2RGB);

		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			tempImg.cols, tempImg.rows, QImage::Format_RGB888);

		QImage mask;
		drawLeftWin(mask, m_curShowImg);

		++m_curFrameIdx;

// 		ui.inVideoView->setValue(m_curFrameIdx);
// 
// 		ui.isBackground->setValue(m_curFrameIdx);
// 
// 		ui.LeftCandIdx->setValue(m_curFrameIdx);
// 
// 		ui.RightCandId->setValue(m_curFrameIdx);
// 
// 		ui.frameIdx->setValue(m_curFrameIdx);
		//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );

	}
	else
	{
		Loggger << "The Final!.\n";
		m_timeCtl->stop();
	}
}

void DLGAutoSelection::drawLeftWin(QImage& mask, QImage& input)
{
	if (mask.width() <= 0)
	{
		mask = QImage(input.width(), input.height(),
			QImage::Format_ARGB32);

		mask.fill(1);
	}

	ui.SinputVideo->setImage(&input);

	ui.SinputVideo->setMaskImage(mask);

	ui.SinputVideo->updateDisplayImage();
}

void DLGAutoSelection::drawRightWin(QImage& mask, QImage& input)
{
	if (mask.width() <= 0)
	{
		mask = QImage(input.width(), input.height(),
			QImage::Format_ARGB32);

		mask.fill(1);
	}

	ui.SLabelsVideo->setImage(&input);

	ui.SLabelsVideo->setMaskImage(mask);

	ui.SLabelsVideo->updateDisplayImage();
}

void DLGAutoSelection::playPause()
{
	m_timeCtl->start();

	ui.previousFrame->setEnabled(false);

	ui.NextFrame->setEnabled(false);

	//ui.selectPairStatue->setEnabled(false);

	m_isPlay = true;
}

void DLGAutoSelection::playStop()
{
	m_timeCtl->stop();

	m_isPlay = false;

	ui.previousFrame->setEnabled(true);

	ui.NextFrame->setEnabled(true);

	//ui.selectPairStatue->setEnabled(true);//show the number of the pairs

	ui.playAndPause->setEnabled(true);

// 	ui.isSelectKeyFrameA->setEnabled(true);
// 
// 	ui.isSelectKeyFrameB->setEnabled(true);
}

void DLGAutoSelection::preFrame()
{
	m_curFrameIdx--;

	if (m_curFrameIdx >= 0)
	{
		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx);

		m_video >> m_curImgData;

		// 		ui.isBackground->setValue(m_curFrameIdx);
		// 
		// 		ui.LeftCandIdx->setValue(m_curFrameIdx);
		// 
		// 		ui.RightCandId->setValue(m_curFrameIdx);
		// 
		// 		ui.frameIdx->setValue(m_curFrameIdx);

		Mat tempImg;
		cvtColor(m_curImgData, tempImg, CV_BGR2RGB);
		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			tempImg.cols, tempImg.rows, QImage::Format_RGB888);
		QImage mask;
		drawLeftWin(mask, m_curShowImg);;

	}
	else
	{
		Loggger << "Bound to beginning!.\n";
	}
}

void DLGAutoSelection::initBaseInfo()
{
	//if (!m_isVideo) return;

	m_rate = m_video.get(CV_CAP_PROP_FPS);

	assert(m_rate > 1);

	m_delay = 1000 / m_rate;

	m_totFrames = m_video.get(CV_CAP_PROP_FRAME_COUNT);
    ui.inVideoView->setRange(0, m_totFrames - 1);



}

void DLGAutoSelection::changeCurFrameIdx(int curFrmIdx)
{

	if (curFrmIdx >= 0 && curFrmIdx < m_totFrames)
	{
		m_curFrameIdx = curFrmIdx;

		//set the progress bar
		ui.inVideoView->setValue(m_curFrameIdx);


// 		if (!ui.RightCandId->hasFocus())
// 		{
// 			ui.LeftCandIdx->setValue(curFrmIdx);
// 		}

// 		ui.RightCandId->setValue(curFrmIdx);
		ui.frameIdx->setValue(curFrmIdx);

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx);

		m_video >> m_curImgData;

		Mat tempImg;
		cvtColor(m_curImgData, tempImg, CV_BGR2RGB);
		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			tempImg.cols, tempImg.rows, QImage::Format_RGB888);

		QImage mask;
		drawLeftWin(mask, m_curShowImg);

	}


}