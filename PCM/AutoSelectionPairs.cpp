#include "AutoSelectionPairs.h"

#define UNKNOWN_FLOW_THRESH 1e3  
#define MIN_FLOW_THRESH 1e-3 

#define  READ_FLOW

IndexType gFId = 0;

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
		if (startFrame  > 0 && middleFrame > 0 && endFrame > 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

void AutoSelect::run()
{
	Loggger<<"Do nothing.\n";
}

void AutoSelect::visOptialFlow(IndexType _start, IndexType _end)
{
	if (_start < 0 || _end > m_totFrames)
	{
		Loggger<<"Out of size.\n";
		return;
	}

	for (IndexType i = _start; i < _end - 1; ++ i)
	{
	   m_video.set(CV_CAP_PROP_POS_FRAMES, i );
	   
	   m_video>>m_curFrameData;

	   Mat nFrame;

	   m_video.set(CV_CAP_PROP_POS_FRAMES, i + 1 );

	   m_video>>nFrame;

	   calculateFlow(m_curFrameData,nFrame, i);
	}

	Loggger<<"End visu!.\n";
}


void AutoSelect::calculateFlow(Mat& _curF, Mat& _nFrame, IndexType curIdx)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;

	Mat motion2Color;

	cvtColor(_curF, srcGray, CV_RGB2GRAY);
	cvtColor(_nFrame, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	//flowSover->calc(desGray,srcGray, outputFlow);

	//denoiseFlow(outputFlow);

	normaliFlow(outputFlow,curIdx);

// 	motionToColor(outputFlow,motion2Color);
// 
// 	char imName[1024];
// 
// 	sprintf(imName,"%.2d-Flow ",curIdx);
// 
// 	imshow(imName,motion2Color);
// 
// 	char imOrName[1024];
// 
// 	sprintf(imOrName,"%.2d-Orignal ",curIdx);
// 
// 	imshow(imOrName,_curF);
}

void AutoSelect::calculateFlow(Mat& _curF, Mat& _nFrame, Mat& flow, IndexType curIdx)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray;

	cvtColor(_curF, srcGray, CV_RGB2GRAY);
	cvtColor(_nFrame, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, flow);

	//flowSover->calc(desGray,srcGray, flow);

	//denoiseFlow(flow);
	normaliFlow(flow,curIdx);
}


void AutoSelect::motionToColor(Mat flow, Mat& color)
{
	if (color.empty())  
		color.create(flow.rows, flow.cols, CV_8UC3);  

	static vector<Scalar> colorwheel; //Scalar r,g,b  
	if (colorwheel.empty())  
		makecolorwheel(colorwheel);  

	// determine motion range:  
	float maxrad = -1;  

	//float minrad = 1e5;

	// Find max flow to normalize fx and fy  
	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
				continue;  
			float rad = sqrt(fx * fx + fy * fy);  
			maxrad = maxrad > rad ? maxrad : rad;  
			//minrad = minrad < rad ? minrad : rad;
		}  
	}  

	//printf("motion min = %f, max = %f.\n",minrad, maxrad);

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  

			float fx = flow_at_point[0] / maxrad;  
			float fy = flow_at_point[1] / maxrad;  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
			{  
				data[0] = data[1] = data[2] = 0;  
				continue;  
			}  
			float rad = sqrt(fx * fx + fy * fy);  

			float angle = atan2(-fy, -fx) / CV_PI;  
			float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
			int k0 = (int)fk;  
			int k1 = (k0 + 1) % colorwheel.size();  
			float f = fk - k0;  
			//f = 0; // uncomment to see original color wheel  

			for (int b = 0; b < 3; b++)   
			{  
				float col0 = colorwheel[k0][b] / 255.0;  
				float col1 = colorwheel[k1][b] / 255.0;  
				float col = (1 - f) * col0 + f * col1;  
				if (rad <= 1)  
					col = 1 - rad * (1 - col); // increase saturation with radius  
				else  
					col *= .75; // out of range  
				data[2 - b] = (int)(255.0 * col);  
			}  
		}  
	}  
}

void AutoSelect::makecolorwheel(vector<Scalar> &colorwheel)
{
	int RY = 15;  
	int YG = 6;  
	int GC = 4;  
	int CB = 11;  
	int BM = 13;  
	int MR = 6;  

	int i;  

	for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
	for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
	for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
	for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
	for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
	for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR)); 
}

void AutoSelect::denoiseFlow(Mat& flow)
{

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f& flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
			{
				flow_at_point[0] = 0.;
				flow_at_point[1] = 0.;
				continue;  
			}

			float rad = sqrt(fx * fx + fy * fy);  

			if (rad < MIN_FLOW_THRESH)
			{
				flow_at_point[0] = 0.;
				flow_at_point[1] = 0.;
			}
		}  
	}  

}

void AutoSelect::normaliFlow(Mat& flow, IndexType fId)
{
	// determine motion range:  
	float maxrad = -1;  

	float minrad = 1e5;

	// Find max flow to normalize fx and fy  
	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f& flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
			{
				flow_at_point[0] = 0.;
				flow_at_point[1] = 0.;
				continue;  
			}

			float rad = sqrt(fx * fx + fy * fy);  

// 			if (rad < MIN_FLOW_THRESH)
// 			{
// 				flow_at_point[0] = 0.;
// 				flow_at_point[1] = 0.;
// 			}

			maxrad = maxrad > rad ? maxrad : rad;  
			minrad = minrad < rad ? minrad : rad;
		}  
	}  


	Mat maskImg;
	maskImg.create(flow.rows,flow.cols, CV_8UC1);

	float thre = 0.20 * (maxrad - minrad);

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f& flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  

			float rad = sqrt(fx * fx + fy * fy);  

			if (rad <= thre)
			{
				maskImg.at<uchar>(i,j) = 255;

				flow_at_point[0] = 0.;
				flow_at_point[1] = 0.; // delete the smooth elements

			}else
			{
				float grayMap = (rad - minrad)/(maxrad- minrad);
				uchar gVal = 255 - (int)(grayMap*255);
				maskImg.at<uchar>(i,j) = gVal;
			}
		}  
	} 

	// save the results
 	char comName[1024];
 
 	sprintf(comName,".\\normalize-flow\\nFlow-%.3d.jpg",fId);
 
 	imwrite(comName, maskImg);
 
 	//imshow(comName,maskImg);

	findEdgeOfNormFlow(maskImg,fId);
}


void AutoSelect::findEdgeOfNormFlow(Mat& flow, IndexType fId)
{
	Mat filterMark,edgeTemp;

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(10,10));

	Mat edgeStruct = getStructuringElement(MORPH_RECT,Size(2,2));

	morphologyEx(flow, filterMark,CV_MOP_CLOSE,erodeStruct);

	morphologyEx(filterMark,edgeTemp,CV_MOP_GRADIENT, edgeStruct);// find the edges of an image	

	Mat tp = 255 - edgeTemp;
	// save the results
	char edgeName[1024];

	sprintf(edgeName,".\\normalize-flow\\edgeFlow-%.3d.jpg",fId);

	//imwrite(edgeName, edgeTemp);
	imwrite(edgeName, tp);
}

void AutoSelect::labelingOneProcess(IndexType _start, IndexType _end, vector<IndexType>& _labels)
{
	if (_start < 0 || _end > m_totFrames - 1)
	{
		Loggger<<"Out of size.\n";
		return;
	}

	vector<Mat> allFlow;
	allFlow.clear();

#ifdef READ_FLOW
		readOpticalFlow(_start, allFlow);
#else
		for (IndexType i = _start; i <= _end; ++ i)
		{
			m_video.set(CV_CAP_PROP_POS_FRAMES, i );

			m_video>>m_curFrameData;

			Mat nFrame;

			m_video.set(CV_CAP_PROP_POS_FRAMES, i + 1 );

			m_video>>nFrame;

    		Mat flow;

			calculateFlow(m_curFrameData,nFrame, flow, i);

			allFlow.push_back(flow);
		}

		saveOpticalFlow(_start, allFlow);
		

#endif
		printf("End for calculation of optical flow.\n");

//         // test ransac flow
// 		IndexType iterN = 0;
// 		m_nbiter = _start;
// 		while(iterN ++ < 20 && iterN < allFlow.size() )
// 		{
// 			ransacFlow(allFlow[iterN]);
// 			m_nbiter ++;
// 		}


  	MatrixXX features;
  
  	calculateFeatures(allFlow,features);
  	
  	//SpectralClusteringThread spClassify;
  
  	_labels.resize(allFlow.size(), 0);
 
 	spClassify.classifyFromFeatures(features,_labels,3);// labeling process

}

void AutoSelect::calculateFeatures(vector<Mat>& inputData, MatrixXX& smtMat)
{
	IndexType fSize = inputData.size();
	assert(fSize > 0);

	smtMat.resize(fSize,fSize);
	smtMat.setZero();

	MatrixXX dist;
	dist.setZero(fSize,fSize);

	for (IndexType i = 0; i < fSize; ++i)
	{
		for (IndexType j = i; j < fSize; ++ j)
		{
			if ( j == i)
			{
				dist(i,i) = 0.;

				continue;
			}

			ScalarType dis =  calcuFlowDis(inputData[i], inputData[j] );
			//ScalarType dis =  calcuUnDirFlowDis(inputData[i], inputData[j] );
			dist(i,j) = dist(j,i) = dis;
		}
	}

	dis2Similar(dist,smtMat);
}

ScalarType AutoSelect::calcuFlowDis(Mat& srFlow, Mat& tgFlow) //existing direction information
{
	IndexType rows = srFlow.rows;
	IndexType cols = srFlow.cols;

	assert(rows > 0 && cols > 0);

	ScalarType totDis = 0.;

	for (int i= 0; i < rows; ++i)   
	{  
		for (int j = 0; j < cols; ++j)   
		{  
			Vec2f srPflow = srFlow.at<Vec2f>(i, j);
			Vec2f tgPflow = tgFlow.at<Vec2f>(i, j);

			float dfx = srPflow[0] - tgPflow[0];  
			float dfy = srPflow[1] - tgPflow[1]; 

			float rad = sqrt(dfx * dfx + dfy * dfy);  
			totDis += rad;
		}
	}

	return totDis;
	//return totDis/(rows*cols);// do not operate this processing?
}

ScalarType AutoSelect::calcuUnDirFlowDis(Mat& srFlow, Mat& tgFlow)
{
	IndexType rows = srFlow.rows;
	IndexType cols = srFlow.cols;

	assert(rows > 0 && cols > 0);

	ScalarType totDis = 0.;

	for (int i= 0; i < rows; ++i)   
	{  
		for (int j = 0; j < cols; ++j)   
		{  
			Vec2f srPflow = srFlow.at<Vec2f>(i, j);
			Vec2f tgPflow = tgFlow.at<Vec2f>(i, j);

			ScalarType srLen = sqrt(srPflow[0]*srPflow[0] + srPflow[1]*srPflow[1]);
			ScalarType tgLen = sqrt(tgPflow[0]*tgPflow[0] + tgPflow[1]*tgPflow[1]);

			float rad = (srLen - tgLen)*(srLen - tgLen);  
			totDis += rad;
		}
	}

	return sqrt(totDis);
}
ScalarType AutoSelect::dis2Similar(ScalarType dis)
{
	//ScalarType simi = 0.;
	ScalarType siga = 1.;

	return (exp(-(dis*dis)/(2*siga*siga)) );
}

void AutoSelect::dis2Similar(MatrixXX& _dis, MatrixXX& _simi)
{
	IndexType rows = _dis.rows();
	IndexType cols = _dis.cols();

	if (rows < 1 || cols < 1)
	{
		Loggger<<"Distance matrix is error!.\n";
		return;
	}

	ScalarType scale_sig = 0.05 * _dis.maxCoeff();
	ScalarType orderS = 2.;
	ScalarType sigma = 10./*_dis.mean()*/;
    MatrixXX temp = _dis;

	_dis /= scale_sig;

	temp = _dis.array() * _dis.array();

	_dis = temp;

	temp = exp(-_dis.array()/(sigma*sigma) );

	_simi = temp;

}

void AutoSelect::saveOpticalFlow(IndexType _startFrame, vector<Mat>& _flow)
{
	// save the results
	IndexType flowSize = _flow.size();

	assert(flowSize > 0);

	IndexType _rows = _flow[0].rows;
	IndexType _cols = _flow[0].cols;

	char optical_name[1024];

	sprintf(optical_name,".\\normalize-flow\\nFlow-%.3d.txt",_startFrame);

	FILE *out_flow = fopen(optical_name,"w");

	fprintf(out_flow, "%d %d %d %d\n",_startFrame, flowSize, _rows, _cols);

	for (IndexType i = 0; i < flowSize; ++ i)
	{
		Mat curFlow = _flow[i];

		for (IndexType j = 0; j < _rows; ++j)
		{
			for (IndexType k = 0; k < _cols; ++ k)
			{
				fprintf(out_flow, "%d %d %f %f\n",j,k,curFlow.at<Vec2f>(j, k)[0],curFlow.at<Vec2f>(j, k)[1]);
			}
		}
	}

	fclose(out_flow);

}
void AutoSelect::readOpticalFlow(IndexType _startFrame,vector<Mat>& _flow)
{
	char optical_name[1024];

	sprintf(optical_name,".\\normalize-flow\\nFlow-%.3d.txt",_startFrame);

	FILE *in_flow = fopen(optical_name,"r");

	IndexType readStart = 0;
	IndexType frameSize = 0;
	IndexType _rows = 0;
	IndexType _cols = 0;

	int status = fscanf(in_flow,"%d %d %d %d", &readStart, &frameSize, &_rows,&_cols);

	for (IndexType i = 0; i < frameSize; ++i)
	{
		IndexType  pixId = 0;
		IndexType  xCoor,yCoor;
		Vec2f fxy;
		Mat curFlow;
		curFlow.create(_rows, _cols, CV_32FC2);
		//curFlow.zeros();

		while (pixId ++ < _rows*_cols)
		{
			int sta = fscanf(in_flow,"%d %d %f %f", &xCoor,&yCoor,&fxy[0],&fxy[1]);

			curFlow.at<Vec2f>(xCoor,yCoor) = fxy;

			if (sta == EOF) break;
		}

		_flow.push_back(curFlow);
	}

	fclose(in_flow);
}


//denoise
struct flowPoint
{
	IndexType x;
	IndexType y;
	ScalarType fx;
	ScalarType fy;
	bool isInner;
	flowPoint(){x = y = 0; fx = fy = 0.0; isInner = false;}
	flowPoint(IndexType _x,IndexType _y, ScalarType _fx, ScalarType _fy)
	{
		x = _x;
		y = _y;
		fx = _fx;
		fy = _fy;
		isInner = false;
	}

	flowPoint(IndexType _x,IndexType _y, Vec2f& _flow)
	{
		x = _x;
		y = _y;
		fx = _flow[0];
		fy = _flow[1];
		isInner = false;
	}

	ScalarType dist2Point(Vec2f& _point)
	{
		return sqrt((fx-_point[0])*(fx-_point[0]) + (fy-_point[1])*(fy-_point[1]) );
	}

	ScalarType arcAngle(Vec2f& _point)
	{
		ScalarType len1 = sqrt(fx *fx + fy*fy);
		ScalarType len2 = sqrt(_point[0]*_point[0] + _point[1]*_point[1]);
		ScalarType inne = fx * _point[0] + fy* _point[1];

	    return (acos((inne)/(len1*len2)));
	}
};


void AutoSelect::computerRansac(vector<flowPoint>& _pixPoints)
{
	IndexType pSize = _pixPoints.size();

	if (pSize < 50)
	{
		Loggger<<"pixPoint size is very small!.\n";
		return;
	}

	IndexType nbIter = 0;

	while (nbIter ++ < 3)
	{
    	Vec2f oldCtr, newCtr;

		samplePoint(_pixPoints,100,oldCtr);

		estimateCenter(_pixPoints,oldCtr,newCtr,0.9);
	}

	Mat ranFlow(m_rows,m_cols, CV_8UC1, 255);
	//ranFlow.create(m_rows,m_cols, CV_8UC1, 255);
	//ranFlow.zeros();

	for ( auto vIter = _pixPoints.begin(); vIter != _pixPoints.end(); ++ vIter)
	{
		flowPoint curP = *vIter;

		if (curP.isInner)
		{
		   ranFlow.at<uchar>(curP.x,curP.y) = 255;//inner -white
		}else
		{
           ranFlow.at<uchar>(curP.x,curP.y) = 0;//outlier
		}
	}


	char edgeName[1024];

	sprintf(edgeName,".\\normalize-flow\\FlowNoise-%.3d.jpg",m_nbiter);

	//imwrite(edgeName, edgeTemp);
	imwrite(edgeName, ranFlow);
	//imshow("ransac flow",ranFlow);

	printf("End for denoise.\n");

}

void AutoSelect::samplePoint(vector<flowPoint>& _pixPoints, IndexType spSize, Vec2f& ctr)
{
	MatrixXX smpFlow;
	smpFlow.setZero(2,_pixPoints.size());

	IndexType nbiter = 0;
	while(nbiter ++ < spSize)
	{
		IndexType idPix = rand()%_pixPoints.size();
		smpFlow(0,nbiter) = _pixPoints[idPix].fx;
		smpFlow(1,nbiter) = _pixPoints[idPix].fy;
	}

	ctr[0] = smpFlow.row(0).mean();
	ctr[1] = smpFlow.row(1).mean();
}

void AutoSelect::estimateCenter(vector<flowPoint>& _pixPoints, Vec2f& oldCtr, Vec2f& newCtr, ScalarType thres)
{
	vector<ScalarType> dist2Ctr;

	for (auto vIter = _pixPoints.begin(); vIter != _pixPoints.end(); ++ vIter)
	{
		dist2Ctr.push_back((*vIter).dist2Point(oldCtr) );
		//dist2Ctr.push_back((*vIter).arcAngle(oldCtr) );
	}

	sort(dist2Ctr.begin(),dist2Ctr.end());//up
	
	IndexType thPos  = (int)(thres * _pixPoints.size() );

	ScalarType disThres = dist2Ctr[thPos];

	vector<ScalarType> xFlows,yFlows;

	IndexType i = 0;
	for (auto vIter = dist2Ctr.begin(); vIter != dist2Ctr.end(); ++ vIter, ++ i)
	{
		if ( (*vIter) <= disThres )
		{
			_pixPoints[i].isInner = true;
			xFlows.push_back(_pixPoints[i].fx);
			yFlows.push_back(_pixPoints[i].fy);
		}
	}

	ScalarType xSum, ySum;
	xSum = 0.;
	ySum = 0.;
	for (IndexType i = 0; i < xFlows.size(); ++i)
	{
		xSum += xFlows[i];
		ySum += yFlows[i];
	}

	newCtr[0] = xSum/(xFlows.size());
	newCtr[1] = ySum/(yFlows.size());
}

void AutoSelect::ransacFlow(Mat& _flow)
{
  IndexType _rows = _flow.rows;
  IndexType _cols = _flow.cols;

  m_rows = _rows;
  m_cols = _cols;

  if (_rows <= 0 || _cols <= 0)
  {
	  Loggger<<"The input flow is error!.\n";
	  return;
  }


  vector<flowPoint> pixPoints;

  //  the loop method should like this.

  for (IndexType y = 0; y < _rows; ++y)
  {
	  for (IndexType x = 0; x < _cols; ++x)
	  { 
		  Vec2f curFlow = _flow.at<Vec2f>(y,x);

		  ScalarType len = curFlow[0]*curFlow[0] + curFlow[1]*curFlow[1];

		  if (len > 0)
		  {
			  flowPoint fP(y,x,curFlow);
			  pixPoints.push_back(fP);
		  }
	  }
  }

  computerRansac(pixPoints);

  //updateFlowWDNoise(_flow,pixPoints);

}

void AutoSelect::updateFlowWDNoise(Mat& flow, vector<flowPoint>& _pixels)
{
	//assign the flow with 0 if the pixel belongs to outlier  
	for (auto iter = _pixels.begin(); iter != _pixels.end(); ++ iter)
	{
		if (!(*iter).isInner)
		{
			IndexType xC = (*iter).x;
			IndexType yC = (*iter).y;
			flow.at<Vec2f>(xC,yC)[0] = 0.;
			flow.at<Vec2f>(xC,yC)[1] = 0.;
		}
	}
}

void AutoSelect::areaForeground(IndexType backgIdx,IndexType stId, IndexType Len)
{
	if (backgIdx <=0 || backgIdx > m_totFrames)
	{
		Loggger<<"Out of size!.\n";
	}

	vector<ScalarType> fgPixelNumber;

	calculateForeG(backgIdx,stId,Len,fgPixelNumber);

	//draw the stroke by matlab
	spClassify.drawVectorByMatlab(fgPixelNumber,stId);
}

// with rectangle 

void AutoSelect::areaForegWithRect(IndexType backgIdx,IndexType stId, 
								   IndexType Len,cv::Rect& _roi)
{
	if (backgIdx <=0 || backgIdx > m_totFrames)
	{
		Loggger<<"Out of size!.\n";
		return;
	}

	if (_roi.height<= 10 || _roi.width <= 10)
	{
		Loggger<< " The size of the rectangle is very small, setting all area!.\n";

		_roi.x = 0;
		_roi.y = 0;
		_roi.height = m_rows;
		_roi.width = m_cols;

	}

	m_roi = _roi;

    printf("ROI start at (%d,%d).\n",_roi.x,_roi.y);
	printf("ROI start width = %d, height = %d).\n",_roi.width,_roi.height);

	vector<ScalarType> fgPixelNumber;
	fgPixelNumber.clear();

	calculateForeGWithRect(backgIdx,stId,Len,_roi,fgPixelNumber);

//<<<<<<< HEAD
	smoothCurves(fgPixelNumber,10);

	vector<bool> isLocalMin;

	//detectMinValue(fgPixelNumber,m_band,isLocalMin);
	//detectMinMax(fgPixelNumber,m_band,isLocalMin);

	vector<IndexType> recordMinMax;
	detectMinMaxint(fgPixelNumber,m_band,recordMinMax);

	// for drawing the curves

	for (int i = 0; i < recordMinMax.size(); ++ i)
	{
		if (recordMinMax[i] == 1 || recordMinMax[i] == 0)
		{
			isLocalMin.push_back(true);
		}
		else
		{
			isLocalMin.push_back(false);
		}
	}

	//piecewiseCurve(fgPixelNumber,m_band);
// =======
// 	smoothCurve(fgPixelNumber);
// >>>>>>> origin/master
	//draw the stroke by matlab
	 //spClassify.drawVectorByMatlab(fgPixelNumber,stId);

	spClassify.drawVectorMinMaxByMatlab(fgPixelNumber,stId,isLocalMin);
	//spClassify.drawVectorMinMaxByMatlab(fgPixelNumber,stId, recordMinMax);

	//// for selection pairs

	vector<Shot> shots;

	splitVideo2Shots(stId,recordMinMax,shots);

	printf("The size of shots = %d.\n", shots.size() );

	vector<std::pair<IndexType,IndexType>> selectPairs;

	autoGetPairs(shots,selectPairs);

	savePairsafterSelection(selectPairs);

	//draw the selected frames position

	vector<bool> isSelected;
	isSelected.resize(Len,false);

	IndexType pSize = selectPairs.size();

	for (IndexType pIdx = 0; pIdx < pSize; ++ pIdx)
	{

		IndexType aId = selectPairs[pIdx].first;
		IndexType bId = selectPairs[pIdx].second;

		isSelected[aId-stId] = true;
		isSelected[bId-stId] = true;
	}

	if (Len != isSelected.size())
	{
		Loggger<<"Not equal, Error!.\n";
		return;
	}

	spClassify.drawVectorMinMaxByMatlabMark(fgPixelNumber,stId,isSelected);

	//
}

IndexType AutoSelect::areaForegWithRectSta(IndexType backgIdx,IndexType stId, 
										   IndexType Len,cv::Rect& _roi)
{
	if (backgIdx <=0 || backgIdx > m_totFrames)
	{
		Loggger<<"Out of size!.\n";
		return 0;
	}

	if (_roi.height<= 10 || _roi.width <= 10)
	{
		Loggger<< " The size of the rectangle is very small, setting all area!.\n";

		_roi.x = 0;
		_roi.y = 0;
		_roi.height = m_rows;
		_roi.width = m_cols;

	}

	m_roi = _roi;

	printf("ROI start at (%d,%d).\n",_roi.x,_roi.y);
	printf("ROI start width = %d, height = %d).\n",_roi.width,_roi.height);

	vector<ScalarType> fgPixelNumber;
	fgPixelNumber.clear();

	calculateForeGWithRect(backgIdx,stId,Len,_roi,fgPixelNumber);

	//<<<<<<< HEAD
	smoothCurves(fgPixelNumber,10);

	vector<bool> isLocalMin;


	vector<IndexType> recordMinMax;
	detectMinMaxint(fgPixelNumber,m_band,recordMinMax);

	// for drawing the curves

	for (int i = 0; i < recordMinMax.size(); ++ i)
	{
		if (recordMinMax[i] == 1 || recordMinMax[i] == 0)
		{
			isLocalMin.push_back(true);
		}
		else
		{
			isLocalMin.push_back(false);
		}
	}

	spClassify.drawVectorMinMaxByMatlab(fgPixelNumber,stId,isLocalMin);
	//spClassify.drawVectorMinMaxByMatlab(fgPixelNumber,stId, recordMinMax);

	//// for selection pairs


	//to obtain the pairs 0110

 	vector<Shot> shots;
 
 	splitVideo2Shots(stId,recordMinMax,shots);
 
 	printf("The size of shots = %d.\n", shots.size() );
 
 	vector<std::pair<IndexType,IndexType>> selectPairs;
 
 	autoGetPairs(shots,selectPairs);
 
 	savePairsafterSelection(selectPairs);
 
 	//draw the selected frames position
 
 	vector<bool> isSelected;
 	isSelected.resize(Len,false);
 
 	IndexType pSize = selectPairs.size();
 
 	for (IndexType pIdx = 0; pIdx < pSize; ++ pIdx)
 	{
 
 		IndexType aId = selectPairs[pIdx].first;
 		IndexType bId = selectPairs[pIdx].second;
 
 		isSelected[aId-stId] = true;
 		isSelected[bId-stId] = true;
 	}
 
 	if (Len != isSelected.size())
 	{
 		Loggger<<"Not equal, Error!.\n";
 		return 0;
 	}
 
 	spClassify.drawVectorMinMaxByMatlabMark(fgPixelNumber,stId,isSelected);

	return shots.size();
	//return 0;
}

ScalarType AutoSelect::foreGPixelsRect(Mat& backImg, Mat& curImg,cv::Rect& _roi)
{
	IndexType nPixels = 0;

	IndexType rows = _roi.y;
	IndexType cols = _roi.x;

	IndexType hig = _roi.height;
	IndexType wid = _roi.width;

	if (hig <=0 || wid <= 0)
	{
		Loggger<<"Rectangle equals to zero.\n";
		return 0.;
	}

	for (IndexType y = 0; y < hig; ++ y)
	{
		for (IndexType x = 0; x < wid; ++ x)
		{
			Vec3b bkRGB = backImg.at<Vec3b>(y + rows,x + cols);
			Vec3b fgRGB = curImg.at<Vec3b>(y + rows,x + cols);

			if (isDifferent(bkRGB,fgRGB) )
			{
				++ nPixels;
			}
		}
	}

	return ((ScalarType)nPixels)/(hig*wid);
}

void AutoSelect::calculateForeGWithRect(IndexType backgIdx,IndexType stId, IndexType fLen, cv::Rect& _roi, 
										std::vector<ScalarType>& pixels)
{
	m_video.set(CV_CAP_PROP_POS_FRAMES, backgIdx );

	m_video>>m_curFrameData;

	IndexType endFrame = stId + fLen;

	for (IndexType frameId = stId; frameId < endFrame; ++ frameId)
	{
		Mat curF;

		m_video.set(CV_CAP_PROP_POS_FRAMES, frameId );

		m_video >> curF;

		ScalarType nPix = foreGPixelsRect(m_curFrameData,curF,_roi);

		pixels.push_back(nPix);
	}
}

void AutoSelect::calculateForeG(IndexType backgIdx,IndexType stId,IndexType fLen, std::vector<ScalarType>& pixels)
{
	 	m_video.set(CV_CAP_PROP_POS_FRAMES, backgIdx );

	 	m_video>>m_curFrameData;

		IndexType endFrame = stId + fLen;

		for (IndexType frameId = stId; frameId < endFrame; ++ frameId)
		{
			Mat curF;

			m_video.set(CV_CAP_PROP_POS_FRAMES, frameId );

			m_video >> curF;

			ScalarType nPix = foreGPixels(m_curFrameData,curF);

			pixels.push_back(nPix);
		}
}

ScalarType AutoSelect::foreGPixels(Mat& backImg, Mat& curImg)
{
	IndexType nPixels = 0;

	IndexType rows = backImg.rows;
	IndexType cols = backImg.cols;

	for (IndexType y = 0; y < rows; ++ y)
	{
		for (IndexType x = 0; x < cols; ++ x)
		{
			Vec3b bkRGB = backImg.at<Vec3b>(y,x);
			Vec3b fgRGB = curImg.at<Vec3b>(y,x);

			if (isDifferent(bkRGB,fgRGB) )
			{
				++ nPixels;
			}
		}
	}

	return ((ScalarType)nPixels)/(rows*cols);
}

void AutoSelect::areaFlowVis(IndexType backgIdx,IndexType stId, IndexType Len)
{
	if (backgIdx <=0 || backgIdx > m_totFrames)
	{
		Loggger<<"Out of size!.\n";
		return;
	}

	gFId = stId;

	vector<ScalarType> fgPixelNumber;

	calculateFlowForeG(backgIdx,stId,Len,fgPixelNumber);

	smoothCurves(fgPixelNumber,8);

	//draw the stroke by matlab
	spClassify.drawVectorByMatlab(fgPixelNumber,stId);

}


void AutoSelect::calculateFlowForeG(IndexType backgIdx,IndexType stId, 
									IndexType Len,vector<ScalarType>& nPixels)
{

	vector<Mat> allFlow;
	allFlow.clear();

#ifdef READ_FLOW

	readOpticalFlow(stId,allFlow);

	IndexType oSize = allFlow.size();

	if (oSize<=1)
	{
		return;
	}

	ScalarType thre = 0.7;

	for (IndexType i = 0; i < oSize; ++ i)
	{
		Mat flow = allFlow[i];

		ScalarType val = calRationFlow(flow, thre);

		nPixels.push_back(val);
	}

#else

	IndexType endFrame = stId + Len;

	for (IndexType frameId = stId; frameId < endFrame; ++ frameId)
	{

		m_video.set(CV_CAP_PROP_POS_FRAMES, frameId);

		m_video>>m_curFrameData;

		Mat curF;

		m_video.set(CV_CAP_PROP_POS_FRAMES, frameId + 1);

		m_video >> curF;

		//ScalarType nPix = calFlow2Stat(m_curFrameData,curF);

		Mat flow;

		ScalarType nPix = calFlow2Stat(m_curFrameData,curF,flow);

		allFlow.push_back(flow);

		nPixels.push_back(nPix);

	}

	saveOpticalFlow(stId,allFlow);

#endif // _DEBUG


}


ScalarType AutoSelect::calFlow2Stat(Mat& fFrame, Mat&sFrame)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;

	Mat motion2Color;

	cvtColor(fFrame, srcGray, CV_RGB2GRAY);
	cvtColor(sFrame, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	// threshold
	ScalarType thre = 0.30;//5%

	return calRationFlow(outputFlow, thre);
}

ScalarType AutoSelect::calFlow2Stat(Mat& fFrame, Mat&sFrame, Mat& outputFlow)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray;

	Mat motion2Color;

	cvtColor(fFrame, srcGray, CV_RGB2GRAY);
	cvtColor(sFrame, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	// threshold
	ScalarType thre = 0.50;//5%

	return calRationFlow(outputFlow, thre);
}


ScalarType AutoSelect::calRationFlow(Mat& flow, ScalarType thres)
{
	// determine motion range:  
	float maxrad = -1;  

	float minrad = 1e5;

	if (flow.rows <=0 || flow.cols <= 0)
	{
		return 0;
	}

	//vector<ScalarType> recordNorm;
	// Find max flow to normalize fx and fy  
	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f& flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
			{
				flow_at_point[0] = 0.;
				flow_at_point[1] = 0.;
				continue;  
			}

			float rad = sqrt(fx * fx + fy * fy);  

			maxrad = maxrad > rad ? maxrad : rad;  
			minrad = minrad < rad ? minrad : rad;

// 			if (rad > 0.001)
// 			{
// 				recordNorm.push_back(rad);
// 			}

		}  
	}  

	//sort(recordNorm.begin(), recordNorm.end() );
// 	IndexType tIdx = thres * recordNorm.size();
// 	float thre = recordNorm[tIdx];


	ScalarType nPixels = 0.;

	float threshold = thres * (maxrad - minrad);

	Mat maskImg;
	maskImg.create(flow.rows,flow.cols, CV_8UC1);

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			Vec2f& flow_at_point = flow.at<Vec2f>(i, j);  
			float fx = flow_at_point[0];  
			float fy = flow_at_point[1];  

			float rad = sqrt(fx * fx + fy * fy);  

			if (rad >= threshold)
			{
				++ nPixels;
				maskImg.at<uchar>(i,j) = 0;
			}else
			{
				maskImg.at<uchar>(i,j) = 255;
			}

		}  
	} 

	char comName[1024];

	sprintf(comName,".\\normalize-flow\\nFlow-%.3d.jpg",gFId);

	imwrite(comName, maskImg);

	++ gFId;

	return nPixels/(flow.rows * flow.cols);

}



void AutoSelect::areaFlowVisRect(IndexType backgIdx,IndexType stId, 
								 IndexType Len,cv::Rect& _roi)
{
	if (backgIdx <=0 || backgIdx > m_totFrames)
	{
		Loggger<<"Out of size!.\n";
		return;
	}

	if (_roi.width <= 10 || _roi.height <= 10)
	{
		Loggger<<"ROI is small.\n";
		return;
	}

	gFId = stId;

	vector<ScalarType> fgPixelNumber;

	calculateFlowForeGRect(backgIdx,stId,Len,fgPixelNumber,_roi);

	smoothCurves(fgPixelNumber,8);

	//draw the stroke by matlab
	spClassify.drawVectorByMatlab(fgPixelNumber,stId);

}

void AutoSelect::calculateFlowForeGRect(IndexType backgIdx,IndexType stId, IndexType Len, 
										vector<ScalarType>& nPixels,cv::Rect& _roi)
{
	vector<Mat> allFlow;
	allFlow.clear();

#ifdef READ_FLOW

	readOpticalFlow(stId,allFlow);

	IndexType oSize = allFlow.size();

	if (oSize<=1)
	{
		return;
	}

	ScalarType thre = 0.6;

	for (IndexType i = 0; i < oSize; ++ i)
	{
		Mat flow = allFlow[i];

		ScalarType val = calRationFlow(flow, thre);

		nPixels.push_back(val);
	}

#else

	IndexType endFrame = stId + Len;

	for (IndexType frameId = stId; frameId < endFrame; ++ frameId)
	{

		m_video.set(CV_CAP_PROP_POS_FRAMES, frameId);

		m_video>>m_curFrameData;

		Mat roiImgFirst;

		(m_curFrameData)(_roi).copyTo(roiImgFirst);

		Mat curF;

		m_video.set(CV_CAP_PROP_POS_FRAMES, frameId + 1);

		m_video >> curF;

		Mat roiImgSecond;
		
		curF(_roi).copyTo(roiImgSecond);

		Mat flow;

		ScalarType nPix = calFlow2Stat(roiImgFirst,roiImgSecond,flow);

		allFlow.push_back(flow);

		nPixels.push_back(nPix);

	}

	saveOpticalFlow(stId,allFlow);

#endif // _DEBUG

}


bool AutoSelect::isDifferent(Vec3b& bgPix, Vec3b& cufPix)
{
	IndexType m_thresDiff = 20;

	if (abs(bgPix[0] - cufPix[0]) > m_thresDiff &&  abs(bgPix[1] - cufPix[1]) > m_thresDiff &&
		abs(bgPix[2] - cufPix[2]) > m_thresDiff)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//<<<<<<< HEAD
void AutoSelect::smoothCurve(std::vector<ScalarType>& pixels)
{
	IndexType vSize = pixels.size();

	if (vSize < 100)
	{
		Loggger<<"The input is very small!.\n";
		return;
	}

	std::vector<ScalarType> temp;
	temp = pixels;

	IndexType endP = vSize - 5;

	for (IndexType i = 5; i < endP; ++ i)
	{
		ScalarType sumW = pixels[i -2] + pixels[i -1] + pixels[i] + pixels[i + 1] + pixels[i + 2];
		 temp[i] = 0.2 * sumW;
	}

	pixels = temp;
}
//=======
void AutoSelect::smoothCurves(vector<ScalarType>& sgls, IndexType band)
{
	IndexType sSize = sgls.size();

	if (sSize <= band || band <= 2)
	{
		Loggger<<" The signs are error!.\n";
		return;
	}
	
	vector<ScalarType> temp;

	temp = sgls;

	ScalarType aW = 1.0/(band);

	MatrixXX weights;
	weights.resize(1,band);

	weights.row(0).setConstant(aW);

	MatrixXX values;
	values.resize(band,1);
	values.col(0).setConstant(0);

	IndexType hLen = band/2;

	IndexType sIdx, i, wS;

	for (sIdx = band; sIdx < sSize - band; ++ sIdx)
	{
		for ( wS = sIdx - hLen, i = 0; wS <= sIdx + hLen && i < band; ++ wS,++ i)
		{
			values(i,0) = sgls[wS];
		}

		ScalarType nVal = (weights * values)(0,0);

		temp[sIdx] = nVal;
	}

	sgls = temp;
}

void AutoSelect::truncationCurves(std::vector<ScalarType>& ptslist, ScalarType threshold)
{

	if (ptslist.size() <= 0)
	{
		return;
	}

	ScalarType vMax = *(max_element(ptslist.begin(),ptslist.end() ) );

	ScalarType vMin = *(min_element(ptslist.begin(),ptslist.end() ) );

	ScalarType thres = vMin + threshold * (vMax - vMin);

	for (IndexType i = 0; i < ptslist.size(); ++ i)
	{
		if (ptslist[i] < thres)
		{
			ptslist[i] = 0.;
		}
	}
}

void AutoSelect::truncationCurves(std::vector<ScalarType>& ptslist, ScalarType threshold, 
								  std::vector<ScalarType>& aftlist,vector<IndexType>& recIdx)
{
	if (ptslist.size() <= 0)
	{
		return;
	}

// 	ScalarType vMax = *(max_element(ptslist.begin(),ptslist.end() ) );
// 
// 	ScalarType vMin = *(min_element(ptslist.begin(),ptslist.end() ) );
// 
// 	ScalarType thres = vMin + threshold * (vMax - vMin);


	//using median value

	vector<ScalarType> temp;

	temp = ptslist;

	sort(temp.begin(),temp.end());

	IndexType mPos = (int)(threshold * temp.size());

	assert(mPos > 0 && mPos < temp.size());

	ScalarType mThres = temp[mPos];

	for (IndexType i = 0; i < ptslist.size(); ++ i)
	{

		if (ptslist[i] > mThres)
		{
			aftlist.push_back(ptslist[i]);
			recIdx.push_back(i);
		}
	}
}
//<<<<<<< HEAD

void AutoSelect::piecewiseCurve(vector<ScalarType>& sgls, IndexType band)
{
	IndexType sSize = sgls.size();

	if (sSize <= band || band <= 2)
	{
		Loggger<<" The signs are error!.\n";
		return;
	}

	printf("The band of this function is %d.\n",band);

	vector<ScalarType> temp;

	temp = sgls;

	for (IndexType i= 0; i < sSize; i += band)
	{
		ScalarType totVal = 0.;

		for ( IndexType j = 0; j < band; ++ j)
		{
			totVal += temp[ i + j];
		}

		totVal /= band;

		for ( IndexType k = 0; k < band; ++ k)
		{
			sgls[i + k] = totVal;
		}
	}

}

bool AutoSelect::isLocalMin(IndexType pos, vector<ScalarType>& sgls, IndexType band)
{
	IndexType pSize = sgls.size();

	if (pos < 0 || pos >= pSize)
	{
		Loggger<<"Out of size!.\n";
		return false;
	}

	IndexType startIdx = pos - band/2;

	IndexType endIdx = pos + band/2;

	if ( band%2 !=0)
	{
		++endIdx;
	}

	if (startIdx < 0)
	{
		startIdx = 0;
		endIdx = band;
	}

	if (endIdx > pSize - 1)
	{
		endIdx = pSize - 1;
		startIdx = endIdx - band ;
	}

	ScalarType itself = sgls[pos];

	for (IndexType i = startIdx; i <= endIdx;  ++ i)
	{
		if ( i == pos)
		{
			continue;
		}

		if ( sgls[i] < itself)
		{
			return false;
		}
	}

	return true;
}

void AutoSelect::detectMinValue(vector<ScalarType>& sgls, IndexType band, vector<bool>& localLab)
{

	if ( sgls.size() <= 0)
	{
		return;
	}

	for (IndexType i = 0; i < sgls.size(); ++ i)
	{
		bool isLoc = isLocalMin(i,sgls, band);
		localLab.push_back(isLoc);
	}

}

void AutoSelect::detectMinMax(vector<ScalarType>& sgls, IndexType band, vector<bool>& localLab)
{
	if ( sgls.size() <= 0)
	{
		return;
	}

	for (IndexType i = 0; i < sgls.size(); ++ i)
	{
		bool isLocMixMax = isLocalMinMax(i,sgls, band);
		localLab.push_back(isLocMixMax);
	}

}

bool AutoSelect::isLocalMinMax(IndexType pos, vector<ScalarType>& sgls, IndexType band)
{
	IndexType pSize = sgls.size();

	if (pos < 0 || pos >= pSize)
	{
		Loggger<<"Out of size!.\n";
		return false;
	}

	bool isMin = true;
	bool isMax = true;

	IndexType startIdx = pos - band/2;

	IndexType endIdx = pos + band/2;

	if ( band%2 !=0)
	{
		++endIdx;
	}

	if (startIdx < 0)
	{
		startIdx = 0;
		endIdx = band;
	}

	if (endIdx > pSize - 1)
	{
		endIdx = pSize - 1;
		startIdx = endIdx - band ;
	}

	ScalarType itself = sgls[pos];

	for (IndexType i = startIdx; i <= endIdx;  ++ i)
	{
		if ( i == pos)
		{
			continue;
		}

		if ( sgls[i] <= itself)
		{
			isMin = false;
			break;
		}

	}

	for (IndexType j = startIdx; j <= endIdx;  ++ j)
	{
		if ( j == pos)
		{
			continue;
		}

		if (sgls[j] > itself)
		{
			isMax = false;
			break;
		}
	}

	if (isMax || isMin)
	{
		return true;
	}else
	{
		return false;
	}

}

void AutoSelect::detectMinMaxint(vector<ScalarType>& sgls, IndexType band, vector<IndexType>& localLab)
{
	if ( sgls.size() <= 0)
	{
		return;
	}

	for (IndexType i = 0; i < sgls.size(); ++ i)
	{
		IndexType isLocMixMax = islocalMinMaxint(i,sgls, band);
		localLab.push_back(isLocMixMax);
	}

}

IndexType AutoSelect::islocalMinMaxint(IndexType pos, vector<ScalarType>& sgls, IndexType band)
{
	IndexType pSize = sgls.size();

	if (pos < 0 || pos >= pSize)
	{
		Loggger<<"Out of size!.\n";
		return false;
	}

	bool isMin = true;
	bool isMax = true;

	IndexType startIdx = pos - band/2;

	IndexType endIdx = pos + band/2;

	if ( band%2 !=0)
	{
		++endIdx;
	}

	if (startIdx < 0)
	{
		startIdx = 0;
		endIdx = band;
		if (endIdx > pSize - 1)
		{
			endIdx = pSize - 1;
		}
	}

	if (endIdx > pSize - 1)
	{
		endIdx = pSize - 1;
		startIdx = endIdx - band ;
		if (startIdx <= 0)
		{
			startIdx = 0;
		}
	}

	ScalarType itself = sgls[pos];

	for (IndexType i = startIdx; i <= endIdx;  ++ i)
	{
		if ( i == pos)
		{
			continue;
		}

		if ( sgls[i] < itself)
		{
			isMin = false;
			break;
		}

	}

	for (IndexType j = startIdx; j <= endIdx;  ++ j)
	{
		if ( j == pos)
		{
			continue;
		}

		if (sgls[j] > itself)
		{
			isMax = false;
			break;
		}
	}

	if (isMax)
	{
		return 1;
	}else if (isMin)
	{
		return 0;
	}else
	{
		return 2;
	}

}

void AutoSelect::splitVideo2Shots(IndexType startFrame, vector<IndexType>& recordMixMax, vector<Shot>& shots)
{
	IndexType vSize = recordMixMax.size();

	if (vSize <= 0)
	{
		Loggger<<"Record is error!.\n";
		return;
	}

	IndexType steps = 6;

	//start from the min value position
	//should consider the following two situations: two adjacent mix or min points!

	for (IndexType i = 0; i < vSize; ++ i)
	{
		Shot sigshot;

		if (i < vSize && i > 0 && recordMixMax[i] == 0 )
		{
			sigshot.startFrame = startFrame + i;//min
			++ i;

			while(i < vSize && ( recordMixMax[i] == 2 || recordMixMax[i] == 0) ){ ++i;} //ignore the normal and min points

			if (i < vSize && recordMixMax[i] == 1)
			{
				sigshot.middleFrame = startFrame + i;//max
				++ i;
			}

			while(i < vSize && (recordMixMax[i] == 2 || recordMixMax[i] == 1) ){ ++ i;} //ignore the normal and max points

			if (i < vSize && recordMixMax[i] == 0)
			{
				sigshot.endFrame = startFrame + i;//min
			}
				
		}//end if: for judging the start points at the min situation.


		//if (i < vSize && i > 0 && recordMixMax[i] == 0 )
		//{
		//	sigshot.startFrame = startFrame + i;//min
		//	
		//	++ i;
		//	while(i < vSize && recordMixMax[i] == 2 ){ ++i;}
		//	
 	//		if (i < vSize && recordMixMax[i] == 0)// cope with the two adjacent min points
 	//		{
 	//			Loggger<<"Happened two adjacent min points.\n";
  //				if ( i > steps + 1)
  //				{
  //					i -= steps; //back to the front of the second min points
  //				}
 	//		}
 	//		else //normal situation
 	//		{
		//		if (i < vSize && recordMixMax[i] == 1)
		//		{
		//			sigshot.middleFrame = startFrame + i;//max
		//			++ i;
		//		}
		//		while(i < vSize && recordMixMax[i] == 2){ ++ i;}
		//		if (i < vSize && recordMixMax[i] == 0)
		//		{
		//			sigshot.endFrame = startFrame + i;//min
		//		}else if(i < vSize && recordMixMax[i] == 1)//max two adjacent max points, equals to the last one
		//		{
		//			Loggger<<"Happened two adjacent max points.\n";
		//			sigshot.middleFrame = startFrame + i; //assign a new index for the middle frame
		//			++ i;
		//			while(i < vSize && recordMixMax[i] == 2){ ++ i;}
		//			if (i < vSize && recordMixMax[i] == 0)
		//			{
		//				sigshot.endFrame = startFrame + i;//min
		//			}
		//		}
		//	}//normal situation: min-max-min
		//}//end if: for judging the start points at the min situation.

		if (sigshot.isValid())
		{
			shots.push_back(sigshot);

		    i -= steps;
		}

	}// end for all points

}

void AutoSelect::autoGetPairs(vector<Shot>& shots, vector<std::pair<IndexType,IndexType>>& pairs)
{
	IndexType stSize = shots.size();
	if (stSize <= 0)
	{
		Loggger<<"The shots is very small!.\n";
	}

	for (IndexType i = 0; i < stSize; ++ i)
	{
		std::pair<IndexType,IndexType> keyPairs;

		//selectWithRegular(shots[i],keyPairs);// av
		selectWithAlinment(shots[i],keyPairs);

		pairs.push_back(keyPairs);
	}
}

void AutoSelect::selectWithRegular(Shot& oneShot, std::pair<IndexType,IndexType>& keyPair)
{
	IndexType firstFrame = (oneShot.middleFrame + oneShot.startFrame)/2 -15;

	IndexType secondFrame = (oneShot.endFrame + oneShot.middleFrame)/2 + 15;

	if (firstFrame > 0 && firstFrame < m_totFrames 
		&& secondFrame > 0 && secondFrame < m_totFrames)
	{
		keyPair.first = firstFrame;
		keyPair.second = secondFrame;
	}

}

void  AutoSelect::selectWithAlinment(Shot& oneShot, std::pair<IndexType,IndexType>& keyPair)
{
	MatrixXX disMat;
	IndexType fRange;

	fRange = 10;

	calDisMatrix(oneShot,fRange,disMat,keyPair);

	//findPairFormMat(disMat,keyPair);
}

// selection with a minor distance between two frames

void AutoSelect::calDisMatrix(Shot& oneShot, IndexType fRange,
							  MatrixXX& disMat,std::pair<IndexType,IndexType>& keyPair)
{
	// starts from the center frame, and expand to two directions.

	printf("Shot start from %d, the middle is %d, and the end is %d.\n",
		   oneShot.startFrame,oneShot.middleFrame, oneShot.endFrame);

	IndexType lMiddle = (oneShot.middleFrame + oneShot.startFrame)/2 - 5;

	IndexType rMiddle = (oneShot.endFrame + oneShot.middleFrame)/2 + 5;

	IndexType lStart = lMiddle - fRange/2;
	IndexType lEnd = lMiddle + fRange/2;

	IndexType rStart = rMiddle - fRange/2;
	IndexType rEnd = rMiddle + fRange/2;

	if (fRange%2 != 0)
	{
		++lEnd;
		++rEnd;
	}

	if (lStart < 0 || rStart < 0)
	{
		lStart = 0;
		lEnd = fRange;

		rStart = 0;
		rEnd = fRange;
	}

	if (lEnd > m_totFrames - 1 || rEnd > m_totFrames - 1)
	{
		lEnd = m_totFrames - 1;
		lStart = lEnd - fRange;

		rEnd = m_totFrames - 1;
		rStart = rEnd -fRange;
	}

	disMat.resize(lEnd - lStart + 1, rEnd - rStart + 1);
	disMat.setZero();

		
	if (m_roi.height <= 0 || m_roi.width <= 0)
	{
		Loggger<<"ROI is error!\n";//should only for ROI
		return;
	}

   IndexType lIdx = lStart;

	for ( IndexType i = 0; lIdx <= lEnd; ++ lIdx, ++ i)
	{
        IndexType rIdx = rStart;

		for (IndexType j = 0; rIdx <= rEnd; ++ rIdx, ++ j)
		{
			Mat srImg,tgImg;

			m_video.set(CV_CAP_PROP_POS_FRAMES, lIdx);
			m_video>>srImg;

			m_video.set(CV_CAP_PROP_POS_FRAMES, rIdx);
			m_video>>tgImg;


			Mat srcRoi, tgRoi;
			srImg(m_roi).copyTo(srcRoi);
			tgImg(m_roi).copyTo(tgRoi);


			ScalarType err = calAlignError(srcRoi,tgRoi);

			disMat(i,j) = err;
		}
	}

// 	Loggger<<"Distance matrix.\n";
// 	Loggger<<disMat<<endl;

	Eigen::MatrixXf::Index minRow,minCol;

	ScalarType min = disMat.minCoeff(&minRow,&minCol);

	keyPair.first = minRow + lStart;
	keyPair.second = minCol + rStart;

}

void AutoSelect::findPairFormMat(MatrixXX& disMat, std::pair<IndexType,IndexType>& keyPair)
{
	// upper triangle matrix
}

ScalarType AutoSelect::calAlignError(Mat& srImg, Mat& tgImg)
{
	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;

	cvtColor(srImg, srcGray, CV_RGB2GRAY);
	cvtColor(tgImg, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	vector<Point2f> srcMatch, desMatch;

	IndexType height = srImg.rows;
	IndexType width = srImg.cols;

	Matrix3X srCoor, tgCoor;

	for (IndexType hId = 0; hId < height; ++ hId  )
	{
		for (IndexType wId = 0; wId < width; ++ wId )
		{

			Vec2f& elem = outputFlow.at<Vec2f>(hId,wId);
			
			ScalarType len = sqrt(elem[0]*elem[0] + elem[1] * elem[1]);

			if ( len < MIN_FLOW_THRESH)
			{
				continue;
			}

			Point2f curPos(hId,wId);
			Point2f nexPos(hId + elem[0], wId + elem[1]);

			if (nexPos.x <0 || nexPos.x > height || nexPos.y <0 || nexPos.y > width)
			{
				continue;
			}

			srcMatch.push_back(curPos);
			desMatch.push_back(nexPos);

		}
	}

	if (srcMatch.size() < 4 || desMatch.size() < 4)
	{
		Loggger<<"Large distortion between frames!\n";
		return 1e5;

	}

	Mat homo2 = findHomography(srcMatch,desMatch,CV_RANSAC);

	return backProjectionError(srcMatch,desMatch,homo2);
}


ScalarType AutoSelect::backProjectionError(vector<Point2f>& srCoor, vector<Point2f>& tgCoor, Mat&homo)
{
	Matrix3X srhomoCoor, tghomoCoor;

	Matrix33 homo2;

	for (IndexType i = 0; i< 3; ++ i)
	{
		for (IndexType j = 0; j < 3; ++ j)
		{
			homo2(i,j) = homo.at<double>(i,j);
		}
	}

// 	Loggger<<"Affine matrix!.\n";
// 	Loggger<<homo2<<endl;

	IndexType pSize = srCoor.size();

	srhomoCoor.resize(3,pSize);
	tghomoCoor.resize(3,pSize);

	for (IndexType i = 0; i < pSize; ++ i)
	{
		srhomoCoor(0,i) = srCoor[i].x;
		srhomoCoor(1,i) = srCoor[i].y;
		srhomoCoor(2,i) = 1;

		tghomoCoor(0,i) = tgCoor[i].x;
		tghomoCoor(1,i) = tgCoor[i].y;
		tghomoCoor(2,i) = 1;
	}

	MatrixXX transCoor;

	transCoor = homo2 * srhomoCoor;

	MatrixXX hoTransCoor;

	hoTransCoor.setZero(3,pSize);


	for (IndexType i = 0; i < pSize; ++ i)
	{
		hoTransCoor(0,i) = transCoor(0,i)/transCoor(2,i);

		hoTransCoor(1,i) = transCoor(1,i)/transCoor(2,i);

		hoTransCoor(2,i) = 1;
	}

	MatrixXX diff = tghomoCoor- hoTransCoor;

	return diff.norm();//sqrt(x^2 + y^2)
}


void AutoSelect::savePairsafterSelection(vector<std::pair<IndexType,IndexType> >& keyPairs)
{
	IndexType pSize = keyPairs.size();
	assert(pSize > 0);

	char fName[1024];
	char sName[1024];

	SingleFrameSet::get_instance().clear();

	//wirteBackground();

	//IndexType fId = 0;

	SingleFrame* curF = new SingleFrame;

    IndexType keyOrder = 0;
	for (IndexType pIdx = 0; pIdx < pSize; ++ pIdx, ++ keyOrder)
	{

		IndexType aId = keyPairs[pIdx].first;
		IndexType bId = keyPairs[pIdx].second;

		sprintf(fName,".\\Pairs\\%.2d-a-%4d.jpg",keyOrder,aId);
		sprintf(sName,".\\Pairs\\%.2d-b-%4d.jpg",keyOrder,bId);

		//SingleFrame* curFa = new SingleFrame;

		Mat imgTemp, mTemp;

		m_video.set(CV_CAP_PROP_POS_FRAMES, aId);

		m_video>>imgTemp;

// 		cvtColor(imgTemp,mTemp,CV_BGR2RGB);
// 
// 		curFa->setBoby(&mTemp);
// 
// 		curFa->frameIdx = fId;
// 
// 		SingleFrameSet::get_instance().push_back(curFa);
		//++ fId;

		imwrite(fName,imgTemp);


		//SingleFrame* curFb = new SingleFrame;
		  
		m_video.set(CV_CAP_PROP_POS_FRAMES, bId);

		m_video>>imgTemp;

// 		cvtColor(imgTemp,mTemp,CV_BGR2RGB);
// 
// 		curFb->setBoby(&mTemp);
// 
// 		curFb->frameIdx = fId;
// 
// 		SingleFrameSet::get_instance().push_back(curFb);
		//++ fId;

		imwrite(sName,imgTemp);
	}


	Loggger<<"Done for selecting images.\n";
}

