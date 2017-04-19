#include "spectral_clustering.h"
#include "Eigen/Sparse"
#include <math.h>
#include "math_fun.h"
#include "globals.h"
#include "engine.h"
#include <assert.h>
#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WIN32
#include <direct.h>
#define get_current_dir _getcwd
#else
#include <unistd.h>
#define get_current_dir getcwd
#endif



using namespace Eigen;


void SpectralClusteringThread::run()
{

 	 SampleSet& set = SampleSet::get_instance();

 	 const IndexType	num_vtx = set[0].num_vertices();
 	 const IndexType num_of_neighbours = 20;
	 IndexType num_of_cluster = 10;
 	 IndexType	num_sample = set.size();
 	 IndexType	dim = 4*(num_sample-1) + 3*num_sample;
 	 MatrixXX	feature_vector( num_vtx, dim );

	 std::cout<<" Number of Cluster :";
	 std::cin >> num_of_cluster;

	 Loggger << "Begin Clustering.\n";
 	 
	 //** Step 1: Compute feature vector for every vertex **/
	 for(IndexType s_idx = 0; s_idx < num_sample - 1;
		 s_idx++ )
	 {

		 Matrix3X&	orig_vtx_coord_matrix = set[s_idx].vertices_matrix();
		 Matrix3X&	dest_vtx_coord_matrix = set[s_idx + 1].vertices_matrix();


		 for (IndexType v_idx = 0; v_idx < num_vtx; v_idx++)
		 {

			 IndexType origin_neighbours[num_of_neighbours];
			 IndexType dest_neighbours[num_of_neighbours];

			 //Get neighbours of the specific vertex between two sample
			 set[s_idx].neighbours(v_idx, num_of_neighbours, origin_neighbours);
			 set[s_idx + 1].neighbours(v_idx, num_of_neighbours, dest_neighbours);
			 MatrixX3	X(num_of_neighbours, 3);
			 MatrixX3	Y(num_of_neighbours, 3);

			 for ( int j = 0; j<num_of_neighbours; j++ )
			 {
				 X.row(j) << orig_vtx_coord_matrix(0, origin_neighbours[j]),
					 orig_vtx_coord_matrix(1, origin_neighbours[j]),
					 orig_vtx_coord_matrix(2, origin_neighbours[j]);
				 Y.row(j) << dest_vtx_coord_matrix(0, origin_neighbours[j]),
					 dest_vtx_coord_matrix(1, origin_neighbours[j]),
					 dest_vtx_coord_matrix(2, origin_neighbours[j]);
			 }

			 Matrix33 sigma = (X.rowwise() - X.colwise().mean()).transpose() * (Y.rowwise() - Y.colwise().mean());
			 Matrix33 rot_mat;

			 Eigen::JacobiSVD<Matrix33> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
			 if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
				 Vec3 S = Vec3::Ones(); S(2) = -1.0;
				 rot_mat = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
			 } else {
				 rot_mat = svd.matrixV()*svd.matrixU().transpose();
			 }
			 Vec4 quat;
			 Math_Utility::rotation_matrix2quat(rot_mat,quat);

			 feature_vector.block<1,4>( v_idx,s_idx*4 ) << quat.transpose();
		 }
	 }

	 ///Add position features
	 for(IndexType s_idx = 0; s_idx < num_sample;
		 s_idx++ )
	 {
		 for (IndexType v_idx = 0; v_idx < num_vtx; v_idx++)
		 {
			feature_vector.block<1,3>( v_idx, (num_sample-1)*4+s_idx*3 ) << set[s_idx][v_idx].x(), set[s_idx][v_idx].y(), set[s_idx][v_idx].z();
		 }

	 }




	//** Step 2: Spectral Clustering, use Matlab **/
  	Engine*	ep;
 	if (! (ep = engOpen(NULL)) )
 	{
 		Loggger<< "Can't not start Matlab engine.\n";
 		return;
 	}
 
 	// set buffer to display result
 	IndexType	result_buffer_size = 1024*1000;
 	char*		result_buffer = new char[result_buffer_size];
 	engOutputBuffer(ep, result_buffer, result_buffer_size);
 

	//Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}

	cur_path[sizeof(cur_path) - 1] = '\0'; 
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );

	mxArray*	mx_feature_vec = NULL;
	numeric::float64*	fv_buffer;
	fv_buffer = new numeric::float64[ feature_vector.rows()*feature_vector.cols() ];
	IndexType k =0;
	for (IndexType i =0 ; i<feature_vector.cols(); i++)
	{
		for (IndexType j=0; j<feature_vector.rows(); j++)
		{
			fv_buffer[k++] = feature_vector(j, i);
		}
	}

	mx_feature_vec = mxCreateDoubleMatrix( feature_vector.rows(),
						feature_vector.cols(), mxREAL);
	memcpy( (char*)mxGetPr(mx_feature_vec),(char*)fv_buffer, 
		feature_vector.rows()*feature_vector.cols()*sizeof(numeric::float64) );

  	engPutVariable( ep, "data", mx_feature_vec );
	char	cmd_buf[128];
	engEvalString( ep, cmd_buf );
	///use default sigma
	///sprintf(cmd_buf,"[labels]=sc(data,%d,100,0,%d);",num_of_neighbours, num_of_cluster);
	///just kmeans here
	sprintf(cmd_buf, "[labels]=k_means(data,\'random\',%d);",num_of_cluster);
	engEvalString(ep,cmd_buf);
 
  
  	///Display output information
  	Loggger<<result_buffer<<std::endl;
 

	mxArray* mx_labels = NULL;
	mx_labels = engGetVariable( ep, "labels" ); 

	if ( mx_labels == NULL )
	{

		mxDestroyArray( mx_feature_vec );
		mxDestroyArray(mx_labels);
		engClose(ep);
		delete [] fv_buffer;
		delete	[] result_buffer;
		Loggger << "Get Data from Matlab error. End Clustering."<<std::endl;
		return;
	}

	numeric::float64 *labels = mxGetPr(mx_labels);	
	assert(( mxGetNumberOfElements(mx_labels))==num_vtx );


    ///** Step 3: Store the results **/
	LOCK( set[0] );
	Sample& sample0 = set[0];
	IndexType i = 0;
	for (Sample::vtx_iterator v_iter = sample0.begin();
		v_iter != sample0.end();
		v_iter++,i++ )
	{
		(*v_iter)->set_label( labels[i] );
	}
	UNLOCK(set[0]);

	mxDestroyArray( mx_feature_vec );
	mxDestroyArray(mx_labels);

 	engClose(ep);
	delete [] fv_buffer;
 	delete	[] result_buffer;


	Loggger << "End Clustering."<<std::endl;
}

void SpectralClusteringThread::classifyFromFeatures(MatrixXX& _features, std::vector<IndexType>& _labels, IndexType nbCluster)
{
// motion recognition- classify the frames

  Loggger<<"Start classify.\n";

  //IndexType nbCluster = 3;

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

  IndexType n = _labels.size();
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
		  /*ScalarType dist = weight2nodes_smooth(node_vec[rowId],node_vec[colId]);*/
		  ScalarType dist = _features(rowId,colId);
		  dm_buffer[rowId * n + colId] = (numeric::float64)dist;
	  }
  }

  memcpy((char*)mxGetPr(mx_distMat),(char*)dm_buffer,n*n*sizeof(numeric::float64));
  delete [] dm_buffer;
  engPutVariable(ep,"W",mx_distMat);

  char cmd_buf[128];
  sprintf(cmd_buf,"[NcutDiscrete,NcutEigenvectors,NcutEigenvalues] = ncutW(W,%d);",nbCluster);
  engEvalString(ep,cmd_buf);

  mxArray *mx_NcutDiscrete = NULL;
  mx_NcutDiscrete = engGetVariable(ep,"NcutDiscrete");

  numeric::float64 *ncutDiscrete = mxGetPr(mx_NcutDiscrete);
  IndexType k=0;

  for ( IndexType i=0;i<nbCluster;i++ )
  {
	  for (IndexType j=0;j<n;j++)
	  {
		  if ( ncutDiscrete[k++]!=0 )
		  {
			  _labels[j] = i;
		  }
	  }
  }

  std::copy_n(_labels.begin(), _labels.size(), 
	  std::ostream_iterator<int>(std::cout, " "));

  
}

void SpectralClusteringThread::drawVectorByMatlab(std::vector<ScalarType>& stData, IndexType startId)
{
	IndexType dSize = stData.size();

	if (dSize <=0)
	{
		Loggger<<" data are empty!.\n";
		return;
	}


	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	Loggger<<"Start to draw the lines.\n";

	std::vector<numeric::float64> tempDt;
	for (IndexType itV = 0; itV < stData.size(); ++ itV)
	{
		tempDt.push_back(stData[itV] );
	}

	mxArray *mx_distMat = NULL;
	mx_distMat = mxCreateDoubleMatrix(dSize,1,mxREAL);

	memcpy((char*)mxGetPr(mx_distMat),(char*)tempDt.data(),dSize*1*sizeof(numeric::float64));
	
	engPutVariable(ep,"W",mx_distMat);

	char cmd_buf[128];
	char cmd_buf2[128];

	sprintf(cmd_buf,"x = %d:1:%d",startId,startId + dSize - 1);// control statement
	sprintf(cmd_buf2,"plot(x,W);hold on;");

	engEvalString(ep,cmd_buf);
	engEvalString(ep,cmd_buf2);

	//delete mx_distMat;

	Loggger<<"End for drawing.\n";
}

void SpectralClusteringThread::drawVectorMinMaxByMatlab(std::vector<ScalarType>& stData, IndexType startId,
														std::vector<bool>& isloLocal)
{
	IndexType dSize = stData.size();

	if (dSize <=0)
	{
		Loggger<<" data are empty!.\n";
		return;
	}


	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\drawfig2");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );


	Loggger<<"Start to draw the lines.\n";

	std::vector<numeric::float64> tempDt;
	for (IndexType itV = 0; itV < stData.size(); ++ itV)
	{
		tempDt.push_back(stData[itV] );
	}

	mxArray *mx_distMat = NULL;
	mx_distMat = mxCreateDoubleMatrix(dSize,1,mxREAL);

	std::vector<numeric::float64> temp;
	temp.resize(dSize);

	for (int i = 0; i < isloLocal.size(); ++ i)
	{
		if (isloLocal[i])
		{
			temp[i] = 1.;
		}else
		{
			temp[i] = 0.;
		}
	}

	mxArray *mx_islocal = NULL;
	mx_islocal = mxCreateDoubleMatrix(dSize,1,mxREAL);
	
	memcpy((char*)mxGetPr(mx_distMat),(char*)tempDt.data(),dSize*1*sizeof(numeric::float64));
	memcpy((char*)mxGetPr(mx_islocal),(char*)temp.data(),dSize*1*sizeof(numeric::float64));

	engPutVariable(ep,"W",mx_distMat);
	engPutVariable(ep,"islocal",mx_islocal);

	char cmd_buf[128];

	sprintf(cmd_buf,"drawcurves(W,%d,islocal)",startId);// control statement

	engEvalString(ep,cmd_buf);


	Loggger<<"End for drawing.\n";

}

void SpectralClusteringThread::drawVectorMinMaxByMatlabMark(std::vector<ScalarType>& stData, 
															IndexType startId, 
															std::vector<bool>& isloLocal)
{
	IndexType dSize = stData.size();

	if (dSize <=0)
	{
		Loggger<<" data are empty!.\n";
		return;
	}


	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\drawfig2");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );


	Loggger<<"Start to draw the lines.\n";

	std::vector<numeric::float64> tempDt;
	for (IndexType itV = 0; itV < stData.size(); ++ itV)
	{
		tempDt.push_back(stData[itV] );
	}

	mxArray *mx_distMat = NULL;
	mx_distMat = mxCreateDoubleMatrix(dSize,1,mxREAL);

	std::vector<numeric::float64> temp;
	temp.resize(dSize);

	for (int i = 0; i < isloLocal.size(); ++ i)
	{ 
		if (isloLocal[i])
		{
			temp[i] = 1.;
		}else
		{
			temp[i] = 0.;
		}
	}

	mxArray *mx_islocal = NULL;
	mx_islocal = mxCreateDoubleMatrix(dSize,1,mxREAL);

	memcpy((char*)mxGetPr(mx_distMat),(char*)tempDt.data(),dSize*1*sizeof(numeric::float64));
	memcpy((char*)mxGetPr(mx_islocal),(char*)temp.data(),dSize*1*sizeof(numeric::float64));

	engPutVariable(ep,"W",mx_distMat);
	engPutVariable(ep,"islocal",mx_islocal);

	char cmd_buf[128];

	sprintf(cmd_buf,"drawcurvesmark(W,%d,islocal)",startId);// control statement

	engEvalString(ep,cmd_buf);


	Loggger<<"End for drawing.\n";
}

void SpectralClusteringThread::detectEdges(cv::Mat& inputImg, cv::Mat& edges)
{
	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\edgesdetec");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );


	Loggger<<"Start to detect edges.\n";

	int rows= inputImg.rows;
	int cols= inputImg.cols;  

	Mat temp;
	cvtColor(inputImg,temp,CV_RGB2BGR);

	mwSize dims[] = {rows, cols,3};
	mxArray *T = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);

	UINT8 *ptr = (UINT8 *) mxGetData(T);

	std::vector<cv::Mat> channels; // B, G, R channels
	cv::split(temp, channels);

	cv::transpose(channels[0], channels[0]);
	cv::transpose(channels[1], channels[1]);
	cv::transpose(channels[2], channels[2]);


	memcpy(ptr, channels[2].ptr(), rows*cols*sizeof(UINT8));
	memcpy(ptr+rows*cols, channels[1].ptr(), rows*cols*sizeof(UINT8));
	memcpy(ptr+2*rows*cols, channels[0].ptr(), rows*cols*sizeof(UINT8));

	engPutVariable(ep, "inImage", T); // put into matlab

	char cmd_buf[128];

	sprintf(cmd_buf,"edge = getEdges(inImage)");// control statement

	engEvalString(ep,cmd_buf);

	mxArray *mx_edges = NULL;
	mx_edges = engGetVariable(ep,"edge");

	if ( mx_edges == NULL )
	{

		mxDestroyArray( mx_edges );
		engClose(ep);
		Loggger << "Get Data from Matlab error. End Detction.\n";
		return;
	}

	assert(( mxGetNumberOfElements(mx_edges)) == rows * cols );

	numeric::float64 *edgePtr = mxGetPr(mx_edges);

	edges.create(rows,cols,CV_8UC1);

	for (int i = 0; i < rows; i++)  
	{  
		for (int j = 0; j < cols; j++)  
		{    
			edges.at<uchar>(i,j) = (int)(edgePtr[j*rows + i]*255);
		}  
	} 


	imshow("Edges",edges);

	printf("End for edges detection!\n");
}

void SpectralClusteringThread::detectEdgesName(cv::Mat& inputImg, cv::Mat& edges,bool isRight)
{
	Engine*	ep;
	if (! (ep = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\edgesdetec");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(ep, cd_cur_path );


	Loggger<<"Start to detect edges.\n";

	int rows= inputImg.rows;
	int cols= inputImg.cols;  

	Mat temp;
	cvtColor(inputImg,temp,CV_RGB2BGR);

	mwSize dims[] = {rows, cols,3};
	mxArray *T = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);

	UINT8 *ptr = (UINT8 *) mxGetData(T);

	std::vector<cv::Mat> channels; // B, G, R channels
	cv::split(temp, channels);

	cv::transpose(channels[0], channels[0]);
	cv::transpose(channels[1], channels[1]);
	cv::transpose(channels[2], channels[2]);


	memcpy(ptr, channels[2].ptr(), rows*cols*sizeof(UINT8));
	memcpy(ptr+rows*cols, channels[1].ptr(), rows*cols*sizeof(UINT8));
	memcpy(ptr+2*rows*cols, channels[0].ptr(), rows*cols*sizeof(UINT8));

	engPutVariable(ep, "inImage", T); // put into matlab

	char cmd_buf[128];

	sprintf(cmd_buf,"edge = getEdges(inImage)");// control statement

	engEvalString(ep,cmd_buf);

	mxArray *mx_edges = NULL;
	mx_edges = engGetVariable(ep,"edge");

	if ( mx_edges == NULL )
	{

		mxDestroyArray( mx_edges );
		engClose(ep);
		Loggger << "Get Data from Matlab error. End Detction.\n";
		return;
	}

	assert(( mxGetNumberOfElements(mx_edges)) == rows * cols );

	numeric::float64 *edgePtr = mxGetPr(mx_edges);

	edges.create(rows,cols,CV_8UC1);

	for (int i = 0; i < rows; i++)  
	{  
		for (int j = 0; j < cols; j++)  
		{    
			edges.at<uchar>(i,j) = (int)(edgePtr[j*rows + i]*255);
		}  
	} 

// 	char fileName[1024];
// 
// 	if (isRight)
// 	{
// 		sprintf(fileName,"Right_edges");
// 	}else
// 	{
// 		sprintf(fileName,"Left_edges");
// 	}

	/*imshow(fileName,edges);*/

	printf("End for edges detection!\n");
}

void SpectralClusteringThread::findSeamPosition(Mat& labImg, vector<Point2f>& ptsList)
{
	Engine*	engM;

	if (! (engM = engOpen(NULL)) )
	{
		Loggger<< "Can't not start Matlab engine.\n";
		return;
	}

	///Get the executable file's path
	char cur_path[FILENAME_MAX];
	if (!get_current_dir(cur_path, sizeof(cur_path)))
	{
		Loggger<<"path is error.\n";
		return;
	}
	cur_path[sizeof(cur_path) - 1] = '\0';
	strcat(cur_path,"\\drawfig2");
	char cd_cur_path[FILENAME_MAX + 3] = "cd ";
	strcat(cd_cur_path, cur_path);
	engEvalString(engM, cd_cur_path );


	Loggger<<"Start to find the pts of the seam.\n";

	int rows= labImg.rows;
	int cols= labImg.cols;  

	//RGB image
// 	Mat temp;
// 	cvtColor(labImg,temp,CV_RGB2BGR);
// 
// 	mwSize dims[] = {rows, cols,3};
// 	mxArray *T = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);
// 
// 	UINT8 *ptr = (UINT8 *) mxGetData(T);
// 
// 	std::vector<cv::Mat> channels; // B, G, R channels
// 	cv::split(temp, channels);
// 
// 	cv::transpose(channels[0], channels[0]);
// 	cv::transpose(channels[1], channels[1]);
// 	cv::transpose(channels[2], channels[2]);
// 
// 
// 	memcpy(ptr, channels[2].ptr(), rows*cols*sizeof(UINT8));
// 	memcpy(ptr+rows*cols, channels[1].ptr(), rows*cols*sizeof(UINT8));
// 	memcpy(ptr+2*rows*cols, channels[0].ptr(), rows*cols*sizeof(UINT8));


	//gray image

	mwSize dims[] = {rows, cols};
	mxArray *T = mxCreateNumericArray(2, dims, mxUINT8_CLASS, mxREAL);
	char *ptr = (char *) mxGetData(T);

	for (int i = 0; i < rows; i++)  
	{  
		for (int j = 0; j < cols; j++)  
		{    
			ptr[j*rows + i] =  (* labImg.row(i).col(j).data);  
		}  
	}  

	engPutVariable(engM, "labs", T); // put into matlab

	char cmd_buf[128];

	sprintf(cmd_buf,"ptslist = findSeamPos(labs)");// control statement

	engEvalString(engM,cmd_buf);


	mxArray *mx_pstlist = NULL;
	mx_pstlist = engGetVariable(engM,"ptslist");

	if ( mx_pstlist == NULL )
	{

		mxDestroyArray( mx_pstlist );
		engClose(engM);
		Loggger << "Get Data from Matlab error. End Detction.\n";
		return;
	}

	IndexType nPos = mxGetNumberOfElements(mx_pstlist)/2;

	numeric::float64 *edgePtr = mxGetPr(mx_pstlist);

	//printf("Size of ps = %d.\n",nPos);

	for (IndexType i = 0; i < nPos; ++ i)
	{
		Point2f tPos;
		tPos.y = edgePtr[i] - 1;
		tPos.x = edgePtr[i + nPos] - 1;

		ptsList.push_back(tPos);
	}

	//matlab 是从1开始计数
	Loggger<<"End for finding seam position.\n";
}