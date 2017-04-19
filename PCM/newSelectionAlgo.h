#pragma once

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


#include "BoostGraph.h"
#include "pool_allocator.h"

#include <vector>

using namespace std;
using namespace cv;


struct Shot;
struct FrameData;

class AutoSelectNew : public QThread
{
	Q_OBJECT
public:
	void run() Q_DECL_OVERRIDE;
signals:
	void finsh_compute();


public:
	AutoSelectNew(IndexType _curFrame, IndexType _totFrame) : m_video(SingleFrameSet::get_instance().getTotVideo())
	{
		m_curFrameIdx = _curFrame;

		m_totFrames = _totFrame;

		if (m_video.get(CV_CAP_PROP_FRAME_HEIGHT) && m_video.get(CV_CAP_PROP_FRAME_WIDTH))
		{
			m_rows = m_video.get(CV_CAP_PROP_FRAME_HEIGHT);
			m_cols = m_video.get(CV_CAP_PROP_FRAME_WIDTH);
		}

		//m_band = 2;
	}

	~AutoSelectNew() {};

public:

	void initDataFromSegVideo();
	//splitvideo2shots;
	void constructVideoGraph();
	void calculateFrameSta();

private:
	//input
	cv::VideoCapture&   m_video; 
	LongType            m_curFrameIdx;
	LongType            m_totFrames;
	FrameData           m_curFrameData;

	vector<FrameData>   m_staData;//

	IndexType m_rows;
	IndexType m_cols;

	//statistics from the segmented videos
	//vector<MatrixXXi> m_videoLabels;
	

	//output
	vector<std::pair<IndexType, IndexType> > m_selectPairs;//save the images

};