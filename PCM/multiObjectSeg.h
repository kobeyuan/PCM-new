#ifndef _MULTI_OBJECT_SEG_H
#define _MULTI_OBJECT_SEG_H

#include <QThread>
#include "sample_set.h"
#include <vector>
#include "globals.h"
#include "basic_types.h"
#include "sample.h"
#include <algorithm>
#include "BoostGraph.h"
#include "pool_allocator.h"

#include "pcl/octree/octree.h"
#include "pcl/octree/octree_pointcloud_changedetector.h"
#include "pcl/point_cloud.h"

#include "pcl/octree/impl/octree2buf_base.hpp"
#include "pcl/octree/impl/octree_base.hpp"
#include "pcl/octree/impl/octree_pointcloud.hpp"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLXYZCloud;

class MultiObjectSeg : public QThread
{
	Q_OBJECT

public:
	MultiObjectSeg()
	{
		downSample = NULL;
	};

	~MultiObjectSeg(){};

public:
	void run() Q_DECL_OVERRIDE;

signals:
	void finish_compute();

public:
	void detectChangePoints(std::vector<IndexType>& changePoints);

	void convertCloudFormat(PCLXYZCloud& tgCloud, Sample& srCloud);

	void showChangePoints(vector<IndexType>& changePs, Sample& pCloud);

public:
	void buildGraphs();

	void buildCorrepondence();// backward and forward directions

public: 
	void addGraphVertex(PCloudGraph& pcGraph, Sample& smp);

	void addGraphEdge(PCloudGraph& pcGraph, Sample& smp);

	void buildKdTree(Sample& smp);

	void nCutProcess(HFrame& curFrame);

	void stillSegmentation();

private:

	map<IndexType, HFrame> hier_componets_; // record a sequence

	static  PoolAllocator allocator_;

	Sample* downSample;
};

#endif