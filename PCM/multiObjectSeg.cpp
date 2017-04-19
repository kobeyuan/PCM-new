#include "multiObjectSeg.h"

#include "engine.h"
#ifdef WIN32
#include <direct.h>
#define get_current_dir _getcwd
#else
#include <unistd.h>
#define get_current_dir getcwd
#endif

PoolAllocator MultiObjectSeg::allocator_;

void MultiObjectSeg::run()
{
	Loggger<<"Begin multi-object segment!\n";

	//std::vector<IndexType> changePs;

	//detectChangePoints(changePs);

	// estimate the motion based on over-segmentation of still point cloud. 3/22/2016

	stillSegmentation();

	Loggger<<"End multi-object segment!\n";
}

void MultiObjectSeg::detectChangePoints(std::vector<IndexType>& changePoints)
{

	SampleSet& set = SampleSet::get_instance();

	if (set.empty())
	{
		Loggger<< " End Clustering(Warning: Empty Set).\n";
		emit finish_compute();
		return;
	}

	PCLXYZCloud srCloud (new pcl::PointCloud<pcl::PointXYZ>); 

	PCLXYZCloud tgCloud (new pcl::PointCloud<pcl::PointXYZ>); 

	IndexType tgFrameId = 10;

	convertCloudFormat(srCloud, set[0]);

	convertCloudFormat(tgCloud, set[tgFrameId]);

	//ScalarType resolution = 0.05f;//horse depth = 7

	ScalarType resolution = 0.25f;//fan--0.25/6;0.5/5

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> cgOctree (resolution);
	
	Loggger<<"Tree depth = "<<cgOctree.getTreeDepth()<<endl;

	cgOctree.setInputCloud(srCloud);

	cgOctree.addPointsFromInputCloud();

	cgOctree.switchBuffers();

	cgOctree.setInputCloud(tgCloud);

	cgOctree.addPointsFromInputCloud();

	Loggger<<"Tree depth = "<<cgOctree.getTreeDepth()<<endl;

	cgOctree.getPointIndicesFromNewVoxels(changePoints);

	Loggger<<"Change points size = "<<changePoints.size()<<endl;

	// visualization of the change points
	showChangePoints(changePoints,set[tgFrameId]);

}

void MultiObjectSeg::convertCloudFormat(PCLXYZCloud& tgCloud, Sample& srCloud)
{
	IndexType psSize = srCloud.num_vertices();

	tgCloud->width = psSize;

	tgCloud->height = 1;

	tgCloud->resize(psSize);

	for (unsigned int pIt = 0; pIt < psSize; ++ pIt )
	{
		Vertex& vtx = srCloud[pIt];

		tgCloud->points[pIt].x = vtx.x();
		tgCloud->points[pIt].y = vtx.y();
	    tgCloud->points[pIt].z = vtx.z();	
	}

}

void MultiObjectSeg::showChangePoints(vector<IndexType>& changePs, Sample& pCLoud)
{
	vector<bool> isChange(pCLoud.num_vertices(),false);

	for (auto cIter = changePs.begin(); cIter != changePs.end(); ++ cIter)
	{
		isChange[(*cIter)] = true;
	}

	IndexType i = 0;
	for (Sample::vtx_iterator vIter = pCLoud.begin();
		  vIter != pCLoud.end(); ++ vIter, ++i)
	{
		if (isChange[i])
		{
			//(*vIter)->set_visble(false);
			(*vIter)->set_label(2);
		}
	}
}

void MultiObjectSeg::buildGraphs()
{
	SampleSet& set = SampleSet::get_instance();

	assert(!set.empty() );

	IndexType iFrame = set.size();

	for (IndexType i = 0; i < iFrame; ++ i)
	{
		if ( hier_componets_.find(i)== hier_componets_.end() )
		{
			hier_componets_.insert(make_pair(i, HFrame()));
			hier_componets_[i].frame_id = i;	
			hier_componets_[i].hier_label_bucket.resize(1);
		}

		// construct a graph of this frame.

		PCloudGraph* new_pcGraph_space = allocator_.allocate<PCloudGraph>();

		PCloudGraph* new_pcGraph = new (new_pcGraph_space)PCloudGraph;

		addGraphVertex(*new_pcGraph, set[i]);

		addGraphEdge(*new_pcGraph,set[i]);

		hier_componets_[i].pcGraph = new_pcGraph;

	}
}

void MultiObjectSeg::addGraphVertex(PCloudGraph& pcGraph, Sample& smp)
{
	IndexType nSize = smp.num_vertices();

	IndexType gIndex = 0;

	map<IndexType,IndexType> gNodeIdx;

	for (; gIndex < nSize; ++ gIndex)
	{
		HVertex* vtx = NULL;
		PCVertexProperty vp;
		vp.index = gIndex;
		vp.vtxSite = vtx;
		boost::add_vertex(vp,pcGraph);	
	}
}

void MultiObjectSeg::addGraphEdge(PCloudGraph& pcGraph, Sample& smp)
{
	IndexType nSize = smp.num_vertices();

	unordered_map<IndexType,bool> recordEdges;

	buildKdTree(smp);

	IndexType gIndex = 0;
	const IndexType k = 6;
	IndexType neighbours[k];
	ScalarType dist[k];

	IndexType edNum = 0;

	for (; gIndex < nSize; ++ gIndex)
	{
		downSample->neighbours(gIndex,k,neighbours,dist);

		for (IndexType i = 1; i < k; ++i)
		{

			bool temp = recordEdges[frame_index_to_key(gIndex,neighbours[i]) ];

			if (!temp)
			{
				PCEdgeProperty ep;
				ep.index = edNum;
				ep.start_ = gIndex;
				ep.end_ = neighbours[i];
				ep.dist = dist[i];

				boost::add_edge(gIndex,neighbours[i],ep,pcGraph);

				recordEdges[frame_index_to_key(neighbours[i],gIndex)] = true;

				++ edNum;
			}
		}
	}
}

void MultiObjectSeg::buildKdTree(Sample& smp)
{
	if (downSample != NULL)
	{
		delete downSample;
	}

	downSample = new Sample();

	IndexType i = 0;

	IndexType vSize = smp.num_vertices();

	for(; i < vSize; ++ i)
	{
		Vertex& vtx = smp[i];

		PointType v( vtx.x(), vtx.y(), vtx.z() );
		ColorType cv(vtx.r(), vtx.g(), vtx.b(), vtx.alpha());
		NormalType nv(vtx.nx(), vtx.ny(), vtx.nz());

		downSample->add_vertex(v,nv,cv);

	}

	downSample->build_kdtree();

}

void MultiObjectSeg::nCutProcess(HFrame& curFrame)
{
	Loggger<<"Start Graph Cut!.\n";

	IndexType nbCluster = 8;
	Loggger<<"segment number = "<<nbCluster<<endl;


	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	//	///set buffer to display result
	IndexType	result_buffer_size = 1024*1000;
	char*		result_buffer = new char[result_buffer_size];
	engOutputBuffer(ep, result_buffer, result_buffer_size);

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\nCut");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );

	IndexType fId = curFrame.frame_id;
	IndexType n = boost::num_vertices(*curFrame.pcGraph);  //cur_graph_index_;
	//	//

	//	//
	mxArray *mx_distMat = NULL;
	numeric::float64* dm_buffer;
	dm_buffer = new numeric::float64[n*n];
	mx_distMat = mxCreateDoubleMatrix(n,n,mxREAL);

	//	///all
	for (int rowId = 0; rowId < n; rowId++)
	{
		/*dm_buffer[rowId*(n+1)] = 0;*/
		for (int colId = 0; colId < n; colId++)
		{
			ScalarType dist = 0.;//weight2nodes_smooth(node_vec[rowId],node_vec[colId]);
			dm_buffer[rowId * n + colId] = (numeric::float64)dist;
		}
	}

	memcpy((char*)mxGetPr(mx_distMat),(char*)dm_buffer,n*n*sizeof(numeric::float64));
	delete [] dm_buffer;
	engPutVariable(ep,"W",mx_distMat);

	char cmd_buf[128];
	sprintf(cmd_buf,"[NcutDiscrete,NcutEigenvectors,NcutEigenvalues] = ncutW(W,%d);",nbCluster);
	engEvalString(ep,cmd_buf);

	//	///Display output information
	//		Loggger<<result_buffer<<std::endl;

	mxArray *mx_NcutDiscrete = NULL;
	mx_NcutDiscrete = engGetVariable(ep,"NcutDiscrete");

	numeric::float64 *ncutDiscrete = mxGetPr(mx_NcutDiscrete);

	Loggger<<"End for each nCut.\n";
}

void MultiObjectSeg::stillSegmentation()
{
	buildGraphs();

	for (auto fiter = hier_componets_.begin(); fiter != hier_componets_.end(); ++ fiter)
	{
		HFrame& curF = fiter->second;

		nCutProcess(curF);
	}
}
