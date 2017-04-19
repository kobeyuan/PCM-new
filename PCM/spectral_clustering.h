#ifndef _SPECTRAL_CLUSTERING_H
#define _SPECTRAL_CLUSTERING_H

#include <QThread>
#include "sample_set.h"

#include <vector>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
using namespace cv;

class SpectralClusteringThread : public QThread
{
	Q_OBJECT

public:
	void run() Q_DECL_OVERRIDE;
	SpectralClusteringThread(){}
	~SpectralClusteringThread(){}

	void classifyFromFeatures(MatrixXX& _features, std::vector<IndexType>& _labels, IndexType nbCluster);
    // for visualization
	void drawVectorByMatlab(std::vector<ScalarType>& stData, IndexType startId);

	void drawVectorMinMaxByMatlab(std::vector<ScalarType>& stData, IndexType startId,
		std::vector<bool>& isloLocal);

	void drawVectorMinMaxByMatlabMark(std::vector<ScalarType>& stData, IndexType startId,
		std::vector<bool>& isloLocal);

	void detectEdges(cv::Mat& inputImg, cv::Mat& edges);

	void detectEdgesName(cv::Mat& inputImg, cv::Mat& edges,bool isRight);

	void findSeamPosition(Mat& labImg, vector<Point2f>& ptsList);

signals:
	void finish_compute();

private:
	
};


#endif