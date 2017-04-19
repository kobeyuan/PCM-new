#include "SingleFrame.h"

void SingleFrame::setBoby(cv::Mat* mat_)
{
	if (!!body)
	{
		//delete body;
		body = nullptr;
	}

	body = new cv::Mat;

	mat_->copyTo(*body);
}

void SingleFrame::setShow(bool showYes)
{
	isShow = showYes;
}

void SingleFrame::setSecRect(std::vector<cv::Rect>& rect_)
{

	IndexType rSize = rect_.size();

	sltRect.clear();

	for (IndexType i= 0; i < rSize; ++i )
	{
		sltRect.push_back(rect_[i] );
	}

// 	if (rect_.width ==0 || rect_.height == 0)
// 	{
// 		sltRect.x = 0;
// 		sltRect.y = 0;
// 
// 		sltRect.width = 0;
// 		sltRect.height = 0;
// 
// 		return;
// 	}
// 
// 	sltRect = rect_;
}
