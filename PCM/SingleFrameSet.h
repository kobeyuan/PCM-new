#ifndef _SINGLEFRAMESET_H_
#define _SINGLEFRAMESET_H_

#include "SingleFrame.h"
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include <unordered_map>
/* 
	A wrapper class of a set of single frames, a video actually.
*/

class SingleFrameSet
{
public:

	static SingleFrameSet& get_instance()
	{
		static SingleFrameSet  instance;
		return instance;
	}

	void push_back(SingleFrame* );
	bool empty(){ return set_.empty(); }
	void clear();
	size_t size(){ return set_.size();}

	SingleFrame& operator[](size_t idx)
	{
		if (idx >= set_.size())
		{
			Loggger<<"Out of frame idx range.\n";
		}

		return *set_[idx];
	}

	cv::VideoCapture& getTotVideo(){return globalVideo;}

	std::vector<SingleFrame*>& getImSeq(){return set_;}

	bool setBackGround(SingleFrame* back_);

	SingleFrame& getBackGround();

private:
	SingleFrameSet(){}
	~SingleFrameSet(){}

	SingleFrameSet(const SingleFrameSet& );
	cv::VideoCapture globalVideo;

private:
	std::vector<SingleFrame*> set_;
	SingleFrame* m_backGround;
	bool m_haveBackGround;
	//std::map<IndexType, SingleFrame>* set_;

};

#endif