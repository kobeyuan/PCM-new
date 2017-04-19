#pragma once

#include <iostream>
#include <vector>
#include <cv.h>
#include <highgui.h>

using namespace std;

class CBezier
{
public:
	CBezier(void);
	CBezier(float thr){ _thr = thr;  }
	~CBezier(void);
	void QuadFitting(vector<CvPoint2D32f> p, vector< vector<int>>& segs); 
	void setthr(float th){ _thr = th; }

private:
	void QuadFitting(int s, int e);  

private:
	vector<CvPoint2D32f> _p; 
	vector<vector<int>> _segs;
	float  _thr;                             // threshold for partition
};

