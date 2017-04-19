#ifndef DIALCUT_H_
#define DIALCUT_H_

#define A_INFINITY 100000.		/* infinite capacity */

#include "basic_types.h"

#include <QThread>
#include "QImage"
#include "cv.h"

#include <vector>
#include <set>

#include <Grabcut/MinCut/graph.h>

#include <graphcuts/image.h>
#include <graphcuts/coord.h>
#include <graphcuts/distanceMap.h>

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// C_GRAD is gradmag divide, C_GRAD2 is comparing actual grads
enum CutType {C_NORMAL, C_GRAD, C_GRAD2, C_BOTH};

#define _ITER_MAX_ 100

#define INDEX_NONPRESENT ((void*)1)
#define INDEX_ACTIVE     ((void*)0)
#define IS_NODE_ID(index) ((unsigned)index>1)
#define IMREF(im, p) (imRef((im), (p).x, (p).y))

const struct Coord NEIGHBORS[] = { Coord(1, 0), Coord(0, -1) };
#define NEIGHBOR_NUM (sizeof(NEIGHBORS) / sizeof(Coord))


using namespace cv;

class DialCut : public QThread
{
	Q_OBJECT
public:
	void run() Q_DECL_OVERRIDE;
signals:
	void finsh_compute();

public:
	DialCut(){};
	DialCut(IndexType _w, IndexType _h, vector<Mat> _imgData, MatrixXXi& _mask, MatrixXXi _initLabels);
	~DialCut(){};

public:
	double BVZ_ComputeEnergy();
	double BVZ_ComputeSpatialOnly();
	double BVZ_ComputeDataOnly();
	double BVZ_Expand(IndexType a, double E_old);

	float BVZ_data_penalty(Coord p, IndexType d);

	float BVZ_interaction_penalty(Coord p, Coord np, IndexType l, IndexType nl);

	float smoothItem(Coord p, Coord np, IndexType l, IndexType nl);

	void compute(); //graph cut main interface

	MatrixXXi& getLabels(){ return m_labels;}

	void makeInertiaRelevant(const vector<RowSpan>& spans);

	void makeInertiaIrrelevant();

	void compactStroke(MatrixXXi& _strokesMask,std::vector<RowSpan>& spans) const;

	float vertGradMag(int imgIdx, Coord& pixPos);

	float horizGraMag(int imgIdx, Coord& pixPos);

private:


	IndexType     m_width;
	IndexType     m_height;
	IndexType     m_curSrImgId;

	vector<Mat>  m_imgData;
	MatrixXXi     m_labels;
	MatrixXXi     m_initLabels;
	MatrixXXi     m_mask;

	vector<IndexType>  m_newRelevant;
	std::set<IndexType>   m_relevant;//record the source image id;
	//bool m_isInnerRelevant;

	ScalarType m_regular;
	ScalarType m_potts;
	ScalarType m_inertia;

private:
	PtrImage		indeces_a;		/* node indeces for assignments originated from pixels in the left image */
	DoubleImage		D_a;					/* penalty for making an assignment active */
	Coord           _size;   
	DistanceMap* _strokeDmap;
	bool _inertiaRelevant;

};

#endif
