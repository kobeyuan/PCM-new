#ifndef _SINGLEFRAME_H_
#define _SINGLEFRAME_H_


#include<cv.h>
#include "pool_allocator.h"
#include "Eigen\Dense"
#include "basic_types.h"
#include <QImage>
#include <QMutex>



class SingleFrame
{
public:
	SingleFrame()
	{
		frameIdx = 0;
		isSelect = false;
		isShow = false;
		body = nullptr;
		sigImg = nullptr;
		roiData = nullptr;
		sltRect.clear();
	}

	SingleFrame(QImage* img)
	{
		frameIdx = 0;
		isSelect = false;
		isShow = false;
		body = nullptr;
		roiData = nullptr;
		sltRect.clear();

		if (!!img)
		{
			sigImg = new QImage;
			*sigImg = img->copy(); //should deep copy.

		}else
		{
			sigImg = nullptr;
		}
	}

	SingleFrame(cv::Mat* mat_)
	{
		frameIdx = 0;
		isSelect = false;
		isShow = false;
		sigImg = nullptr;
		roiData = nullptr;
		body = new cv::Mat;
		sltRect.clear();

		mat_->copyTo(*body);
	}

	SingleFrame(SingleFrame* sFrame_)
	{
		frameIdx = sFrame_->frameIdx;
		isSelect = sFrame_->isSelect;
		isShow   = sFrame_->isShow;
		setBoby(sFrame_->body);

		if (!!sFrame_->sigImg)
		{
		  sigImg = new QImage(*sFrame_->sigImg);
		}

		if(!!sFrame_->roiData)
		{
			roiData = new cv::Mat;
			sFrame_->roiData->copyTo(*roiData);
		}

		sltRect = sFrame_->sltRect;
	}

	void setBoby(cv::Mat* mat_);

	void setShow(bool showYes);

	void setSecRect(std::vector<cv::Rect>& rect_);

	~SingleFrame(){}


public:

	LongType frameIdx; //index among the video
	bool       isSelect; // is key frame?
	bool       isShow;  //display in the window.
	cv::Mat*       body;
	QImage*        sigImg;
	std::vector<cv::Rect>       sltRect;
	cv::Mat*       roiData; 

};

#endif

