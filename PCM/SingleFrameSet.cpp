#include "SingleFrameSet.h"

void SingleFrameSet::push_back(SingleFrame* new_single)
{
	if ( new_single != nullptr)
	{
		set_.push_back(new_single);
	}
}

void SingleFrameSet::clear()
{
	while( set_.empty() == false )
	{
		delete set_.back();
		set_.pop_back();
	}

	if (!!m_backGround)
	{
		m_backGround = nullptr;
	}
}

bool SingleFrameSet::setBackGround(SingleFrame* back_)
{
	if (!!back_)
	{
		m_backGround = new SingleFrame(back_);
		m_haveBackGround = true;

	}else
	{
		m_backGround = nullptr;
		return false;
	}
}

SingleFrame& SingleFrameSet::getBackGround()
{
	if (!!m_backGround)
	{
		return *m_backGround;
	}
}
