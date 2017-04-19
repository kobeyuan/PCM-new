#include "newSelectionAlgo.h"

struct Shot
{
	IndexType startFrame;
	IndexType middleFrame;
	IndexType endFrame;

	Shot()
	{
		startFrame = 0;
		middleFrame = 0;
		endFrame = 0;
	}
	bool isValid()
	{
		if (startFrame > 0 && middleFrame > 0 && endFrame > 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

//default using opencv's data structure

struct FrameData
{
	IndexType frameIdx;
	Mat segLabels;
	Mat fFlow; //Mat::zeros(length, lenght, CV_8UC3);
	Mat bFlow;
	vector<Point2i> objContour;
	Point2f objWeight;
	vector<Point2i> handContour;//connect the hands region, it may difficult for
	Rect objBox;
	Rect exObjBox;
};

void AutoSelectNew::initDataFromSegVideo()
{
	//load the statistics data to select the key frames
	//initialize the FrameData structure
}

void AutoSelectNew::constructVideoGraph()
{
// 	LabelsGraph* new_labelGraph_space = allocator_.allocate<LabelsGraph>();
// 
// 	LabelsGraph* new_labelGraph = new (new_labelGraph_space)LabelsGraph;
// 
// 	add_vertex(gvp, *new_labelGraph);
// 
// 	add_edge(j, i, gep, *new_labelGraph);
	//the codes from the multi-way propagation Line 316

	//shortest path algorithm: LIne 1895

}

void AutoSelectNew::calculateFrameSta()
{
	//contour
	//weight
	//
}

