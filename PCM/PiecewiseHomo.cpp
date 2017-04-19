#include "PiecewiseHomo.h"

//#include "Math/LeastSquaresSparseSolver.h"

#include "Math/mathlib.h"
#include <math.h>

#include "windows.h"
#include <gl/gl.h>
#include <gl/glu.h>

PieceHomo::PieceHomo(Mat& _srimg, Mat& _tgimg, IndexType _res):m_srImg(_srimg),m_tgImg(_tgimg)
{
	if (m_srImg.rows > 0 && m_srImg.cols > 0)
	{
		m_width = m_srImg.cols;
		m_heigh = m_srImg.rows;
	}else
	{
		m_width = 0;
		m_heigh = 0;
	}

	if (_res > 0)
	{
	  m_res = _res;
	}else
	{
	  m_res = 10;
	}

	m_srLines.clear();
	m_tgLines.clear();
	m_linesMathingInfo.clear();

	//
}

void PieceHomo::run()
{

}

void PieceHomo::alignmentPiece(Mat& outImg)
{
	Mat interImg;

	featureBasedAlignment(interImg);

	m_srImg = interImg;

	calFeaPoUsingFlow();

	constructGridMesh();

	initFeatureCoordinate();

	//initWarp();

 	warpingGrid();
// 
// 	renderWarpedImg();
}

void PieceHomo::deformMatchingImg(Mat& outImg)
{
  constructGridMesh();

  // for initialize the transformation

  initFeatureCoordinate();

  IndexType nIter = 1;

  while (nIter -- > 0)
  {
     warpingGrid();//using Eigen Sparse Linear Systems Solver

	 //warpingPiecewiseHomo();//estimate each homo

     //estimateHomo();

	 renderWarpedImg();// draw quads only 

	 Mat tpimg;

	 tpimg = imread(".\\tempImages\\homoDeform2.png");//buffer object

	 cvtColor(tpimg,outImg,CV_RGB2BGR);

  }

}


void PieceHomo::deformMatchingLinesPoints(Mat& outImg)
{
	constructGridMesh();

	// for initialize the transformation
	initFeatureCoordinate();

	IndexType nIter = 1;

	while (nIter -- > 0)
	{
		warpingGridwithLines();

		renderWarpedImg();// draw quads only 

		Mat tpimg;
		tpimg = imread(".\\tempImages\\homoDeform2.png");//buffer object
		cvtColor(tpimg,outImg,CV_RGB2BGR);
	}

}


void PieceHomo::calFeaPoUsingFlow()
{
	// find the featured points and those correspondence.
	//fill the data m_sr/tgFeaturePs;

	Ptr<DenseOpticalFlow> flowSover = createOptFlow_DualTVL1();

	Mat srcGray, desGray, outputFlow;

	//Mat motion2Color;

	cvtColor(m_srImg, srcGray, CV_RGB2GRAY);
	cvtColor(m_tgImg, desGray, CV_RGB2GRAY);

	flowSover->calc(srcGray,desGray, outputFlow);

	//flowSover->calc(desGray,srcGray, outputFlow);

	//motionToColor(outputFlow,motion2Color);

	//imshow("Flow",motion2Color);

	vector<Point2f> srcMatch, desMatch;

	IndexType height = m_heigh;
	IndexType width = m_width;

	for (IndexType hId = 0; hId < height; /*++ */hId += 10 )
	{
		for (IndexType wId = 0; wId < width; /*++ */wId += 10)
		{
			Vec2f& elem = outputFlow.at<Vec2f>(hId,wId);

			Point2f curPos(hId,wId);
			Point2f nexPos(hId + elem[0], wId + elem[1]);

			if (nexPos.x <0 || nexPos.x > height || nexPos.y <0 || nexPos.y > width)
			{
				continue;
			}

			m_srFeaturePs.push_back(curPos);
			m_tgFeaturePs.push_back(nexPos);

		}
	}

	printf("The size of sr and tg feature points %d, %d.\n",m_srFeaturePs.size(), m_tgFeaturePs.size() );

}

void PieceHomo::initFeatures(vector<CvPoint2D32f>& srPs, vector<CvPoint2D32f>& tgPs)
{
	m_srFeaturePs.clear();
	m_tgFeaturePs.clear();

	m_srFeaturePs = srPs;
	m_tgFeaturePs = tgPs;
}

void PieceHomo::constructGridMesh()
{
	// fill the data- m_grid;

	m_grid = new Cquadmesh(m_width,m_heigh,m_res); 
	//_H = cvCreateMat(3,3,CV_32FC1);
	_H.create(3,3,CV_32FC1);
	// texCoord
	_texCoord.resize(m_grid->numofnodes());

	m_oriCtlCoor.setZero(m_grid->numofnodes(),2);

	for (int i=0;i<m_grid->numofnodes();i++)
	{
		float x,y;
		x = m_grid->_nodes[i].x;
		y = m_grid->_nodes[i].y;

		_texCoord[i].x = x/(m_width*1.0);
		_texCoord[i].y = y/(m_heigh*1.0);

		m_oriCtlCoor(i,0) = x;
		m_oriCtlCoor(i,1) = y; //using points index
	}

	m_oldCtlCoor = m_oriCtlCoor;

	//for corners
	_corners.clear();

	for (IndexType i = 0; i < m_grid->_corner.size(); ++ i)
	{
		CvPoint2D32f cor;
		cor.x = m_grid->_nodes[m_grid->_corner[i]].x;
		cor.y = m_grid->_nodes[m_grid->_corner[i]].y;
		_corners.push_back(cor);
	}

}

void PieceHomo::featureBasedAlignment(Mat& interImg)
{
	string detectorType = "SIFT";  
	string descriptorType = "SIFT";  
	string matcherType = "FlannBased";

	Ptr<FeatureDetector> featureDetector;  
	Ptr<DescriptorExtractor> descriptorExtractor;  
	Ptr<DescriptorMatcher> descriptorMatcher; 

	if( !createDetectorDescriptorMatcher( detectorType, descriptorType,
		matcherType, featureDetector, 
		descriptorExtractor,descriptorMatcher ) )  
	{  
		Loggger<<"Create Detector Descriptor Matcher False!"<<endl;  
		return ;  
	} 

	//get the srcImg keypoints

	vector<KeyPoint> queryKeyPs, refQueryKP;
	vector<KeyPoint> trainKeyPs, refTrainKP;
	Mat queryDesc;
	Mat trainDesc;

	queryKeyPs.clear();
	trainKeyPs.clear();
	queryDesc.setTo(0);
	trainDesc.setTo(0);

	featureDetector->detect(m_srImg, queryKeyPs);

	//refineKeyps(m_srImg,queryKeyPs,refQueryKP);

	if (/*refQueryKP.size() > 3*/queryKeyPs.size() > 3)
	{
		descriptorExtractor->compute(m_srImg,queryKeyPs,queryDesc);

		//descriptorExtractor->compute(m_srImg,refQueryKP,queryDesc);

		//  	   for (IndexType i = 0; i < refQueryKP.size(); ++i)
		//  	   {
		//  		   cv::circle(srcImg, refQueryKP[i].pt, 5, cv::Scalar(255.), -1);
		//  	   }
	}


	featureDetector->detect(m_tgImg,trainKeyPs);

	//refineKeyps(m_tgImg,trainKeyPs,refTrainKP);

	if (/*refTrainKP.size() > 3*/trainKeyPs.size() > 0)
	{
		descriptorExtractor->compute(m_tgImg,trainKeyPs,trainDesc);

		//descriptorExtractor->compute(m_tgImg,refTrainKP,trainDesc);

		// 		for (IndexType i = 0; i < refTrainKP.size(); ++i)
		// 		{
		// 			cv::circle(desImg, refTrainKP[i].pt, 5, cv::Scalar(255.), -1);
		// 		}

	}else
	{
		Loggger<<"The feature points are very small.\n";
		return;
	}


	Mat sTemp,tTemp;

	cvtColor(m_srImg,sTemp,CV_RGB2BGR);
	cvtColor(m_tgImg,tTemp,CV_RGB2BGR);

	// 	imshow("Sr-feature",sTemp);
	// 	imshow("Tg-feature",tTemp);


	Mat homo;

	vector<DMatch> matchPs;

	bool isFound = matchingDescriptor(queryKeyPs,trainKeyPs,queryDesc,trainDesc,descriptorMatcher,homo, matchPs,true);

	//bool isFound = matchingDescriptor(refQueryKP,refTrainKP,queryDesc,trainDesc,descriptorMatcher,homo, matchPs,true);

	vector<Point2f> srcMatch, desMatch;

	Loggger<<"Matches size = "<<matchPs.size()<<endl;

	for (IndexType i = 0; i < matchPs.size(); ++i)
	{
		Point2f pt1 = queryKeyPs[matchPs[i].queryIdx].pt;
		Point2f pt2 = trainKeyPs[matchPs[i].trainIdx].pt;

// 		Point2f pt1 = refQueryKP[matchPs[i].queryIdx].pt;
// 		Point2f pt2 = refTrainKP[matchPs[i].trainIdx].pt;

		srcMatch.push_back(pt1);
		desMatch.push_back(pt2);

		//cv::circle(desImg, trainKeyPs[matchPs[i].trainIdx].pt, 5, cv::Scalar(255.), -1);
	}

	Mat drawIMg, drawTemp;

	drawMatches(m_srImg, queryKeyPs,m_tgImg,trainKeyPs, matchPs, drawIMg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//drawMatches(m_srImg, refQueryKP,m_tgImg,refTrainKP, matchPs, drawIMg,
		//Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cvtColor(drawIMg,drawTemp,CV_RGB2BGR);

	imshow("matches", drawTemp);

	if (srcMatch.size() > 3 && desMatch.size() > 3)
	{
		Mat homo2 = findHomography(srcMatch,desMatch,CV_RANSAC);

		interImg.create(interImg.rows, m_srImg.cols, m_srImg.type() );

		warpPerspective(m_srImg,interImg,homo2,interImg.size() );

		Mat temp;

		cvtColor(interImg,temp,CV_RGB2BGR);

		imshow("DEF-RANSAC", temp);

	}else
	{
		Loggger<<"The size of the matching points is small!.\n";
	}
}

void PieceHomo::initWarp()
{

}

void PieceHomo::initHomo(Mat& _homo)
{
	m_glbHomo = _homo;
}

void PieceHomo::initLines(vector<lineSeg>& lLines, vector<lineSeg>& rLines,
						  vector<IndexType>& matchingInfo)
{
	m_srLines = lLines;
	m_tgLines = rLines;
	m_linesMathingInfo = matchingInfo;
}

void PieceHomo::combineHomos2Img(Mat& _outImg)
{
	//for each face
// 	outImg.create(srcImg.rows, srcImg.cols, srcImg.type() );
// 	warpPerspective(srcImg,outImg,m_glbHomo,outImg.size() );

	IndexType nFaces = m_grid->numoffaces();// equals to the size of homo

	Point pA,pC;
	int a, b, c, d; 

	//Mat outImg(m_srImg.size(),m_srImg.type(),Scalar::all(0));

	Mat srTemp;
	cvtColor(m_srImg,srTemp,CV_RGB2BGR);
	imshow("oriBBX",srTemp);

	_outImg.create(m_srImg.size(),m_srImg.type());

	_outImg.setTo(0);

	for (IndexType i = 0; i < nFaces;  ++ i)
	{
		a = m_grid->_faces[i].v0; /*b = m_grid->_faces[i].v1;*/
		c = m_grid->_faces[i].v2; /*d = m_grid->_faces[i].v3;*/
		
		//a
		pA.x = m_oriCtlCoor(a,0);//x
		pA.y = m_oriCtlCoor(a,1);//y

		//c
		pC.x = m_oriCtlCoor(c,0);//x
		pC.y = m_oriCtlCoor(c,1);//y

		Rect oriS(pA,pC);

		Mat oriImg(m_srImg.size(),m_srImg.type(),Scalar::all(0));

		Mat roitemp;
		m_srImg(oriS).copyTo(roitemp);

		roitemp.copyTo(oriImg(oriS) );

		Mat locT(m_srImg.size(),m_srImg.type(),Scalar::all(0));	

		warpPerspective(oriImg,locT,/*m_glbHomo*/m_homo[i],locT.size() );

		add(_outImg,locT,_outImg);

	}

	Mat temp;

	cvtColor(_outImg,temp,CV_RGB2BGR);
	imshow("resTrans",temp);

}

void PieceHomo::warpingGrid()
{
	//equation number: edges + features + corners =  2*(m+1)(m+2) + K + 4;

	bool isFixBoundary = false;

	IndexType cols = m_grid->numofnodes(); 
	
	IndexType rows = 0;

	if (isFixBoundary)
	{
		rows = m_grid->numofedges() + m_tgFeaturePs.size() + m_grid->_boundaries.size();
	}else
	{
        rows = m_grid->numofedges()+ m_tgFeaturePs.size()+ 4;
	}

	Eigen::SparseMatrix<ScalarType> lA;
	lA.resize(rows,cols);

	typedef Eigen::Triplet<ScalarType> T;

	vector<T> eleT;

	MatrixXX rB,lX,initX;
	rB.resize(rows,2);
	lX.resize(rows,2);
	initX.resize(rows,2);

	//fill lA and rB

	//for each edge?some problems exiting with the cost function.

	IndexType nEquation = 0;
	for (IndexType eId = 0; eId < m_grid->numofedges(); ++ eId,++ nEquation)
	{
		IndexType sPId = m_grid->_edges[eId].v0;
		IndexType ePId = m_grid->_edges[eId].v1;

	    //from the above step
		Point2f sCoor(m_oldCtlCoor(sPId,0),m_oldCtlCoor(sPId,1) ); 
		Point2f eCoor(m_oldCtlCoor(ePId,0),m_oldCtlCoor(ePId,1) );

		Point2f sdCoor,edCoor;

		//using which one?
// 		transHomo(sCoor,sdCoor,m_homo[shomoIdx]);
// 		transHomo(eCoor,edCoor,m_homo[ehomoIdx]);//piecewise
// 		transHomo(sCoor,sdCoor,m_glbHomo);
// 		transHomo(eCoor,edCoor,m_glbHomo);//before

		transHomo(sCoor,sdCoor,_H);
		transHomo(eCoor,edCoor,_H);//before

		ScalarType dx = sdCoor.x - edCoor.x;
		ScalarType dy = sdCoor.y - edCoor.y;

		eleT.push_back(T(nEquation,sPId,1));
		eleT.push_back(T(nEquation,ePId,-1));

		rB(nEquation,0) = dx;
		rB(nEquation,1) = dy;
	}

	//for each feature
	ScalarType fweight = 1000;

	for (IndexType i = 0; i < m_srFeaturePs.size(); ++ i,++ nEquation)
	{
		//dis = 0.;
		for (IndexType j = 0; j < _np[i].size(); ++j)
		{
			IndexType feaId = _np[i][j];
			eleT.push_back(T(nEquation,feaId,fweight * _cd[i][j]));
		}

		rB(nEquation,0) = fweight * m_tgFeaturePs[i].x;
		rB(nEquation,1) = fweight * m_tgFeaturePs[i].y;
	}


	//for each line segments (start: middle: end)



	//fix the boundary edges/nodes
	//.--.--.
	//.     .
	//.--.--.

	if (isFixBoundary)
	{
		for (IndexType i = 0; i < m_grid->_boundaries.size(); ++ i, ++ nEquation)
		{
			IndexType corId = m_grid->_boundaries[i];
			eleT.push_back(T(nEquation,corId,1));
			rB(nEquation,0) = m_oriCtlCoor(corId,0);
			rB(nEquation,1) = m_oriCtlCoor(corId,1);
		}

	}else
	{
		//only for each corners
		ScalarType cweight = 1000;
		for (IndexType j = 0; j < _corners.size(); ++ j, ++ nEquation)
		{
			IndexType corId = m_grid->_corner[j];
			eleT.push_back(T(nEquation,corId,1*cweight));
			rB(nEquation,0) = cweight * _corners[j].x;
			rB(nEquation,1) = cweight * _corners[j].y;

		}
	}

	//assign the left value
	lA.setFromTriplets(eleT.begin(),eleT.end());

	//guess the initial solvers//using the original coordinate
	// the original coordinates of the control points

	for (IndexType i = 0; i < cols; ++ i)
	{
		initX(i,0) = m_oldCtlCoor(i,0);
		initX(i,1) = m_oldCtlCoor(i,1);
	}

    Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<ScalarType> > lscg;

	lscg.compute(lA);

	lscg.solveWithGuess(rB,initX);

	lX = lscg.solve(rB);
    
	m_newCtlCoor = lX; //update coordinate.

}

//preserving lines
void PieceHomo::warpingGridwithLines()
{
	//equation number: edges + features + corners +  lines =  2*(m+1)(m+2) + K + 4 + n;

	bool isFixBoundary = false;

	IndexType cols = m_grid->numofnodes(); 

	IndexType rows = 0;

	if (isFixBoundary)
	{
		rows = m_grid->numofedges() + m_tgFeaturePs.size() + 
			   m_tgLines.size() + m_grid->_boundaries.size();
	}else
	{
		rows = m_grid->numofedges() + m_tgFeaturePs.size() + 4 + m_srLines.size();
	}

	Eigen::SparseMatrix<ScalarType> lA;
	lA.resize(rows,cols);

	typedef Eigen::Triplet<ScalarType> T;

	vector<T> eleT;

	MatrixXX rB,lX,initX;
	rB.resize(rows,2);
	lX.resize(rows,2);
	initX.resize(rows,2);

	//fill lA and rB

	//for each edge?some problems exiting with the cost function.

	IndexType nEquation = 0;
	for (IndexType eId = 0; eId < m_grid->numofedges(); ++ eId,++ nEquation)
	{
		IndexType sPId = m_grid->_edges[eId].v0;
		IndexType ePId = m_grid->_edges[eId].v1;

		//from the above step
		Point2f sCoor(m_oldCtlCoor(sPId,0),m_oldCtlCoor(sPId,1) ); 
		Point2f eCoor(m_oldCtlCoor(ePId,0),m_oldCtlCoor(ePId,1) );

		Point2f sdCoor,edCoor;

		transHomo(sCoor,sdCoor,_H);
		transHomo(eCoor,edCoor,_H);//before

		ScalarType dx = sdCoor.x - edCoor.x;
		ScalarType dy = sdCoor.y - edCoor.y;

		eleT.push_back(T(nEquation,sPId,1));
		eleT.push_back(T(nEquation,ePId,-1));

		rB(nEquation,0) = dx;
		rB(nEquation,1) = dy;
	}

	//for each feature
	ScalarType fweight = 1000;

	for (IndexType i = 0; i < m_srFeaturePs.size(); ++ i,++ nEquation)
	{
		//dis = 0.;
		for (IndexType j = 0; j < _np[i].size(); ++j)
		{
			IndexType feaId = _np[i][j];
			eleT.push_back(T(nEquation,feaId,fweight * _cd[i][j]));
		}

		rB(nEquation,0) = fweight * m_tgFeaturePs[i].x;
		rB(nEquation,1) = fweight * m_tgFeaturePs[i].y;
	}


	//for each line segments (start: middle: end)

	m_linesNodeIdx.clear();
	m_lineNodeCoor.clear();

	ScalarType lineW = 100.;

	//IndexType nParts = 4;

	for (IndexType k = 0; k < m_srLines.size(); ++ k, ++ nEquation)
	{
		lineSeg curLine = m_srLines[k];
		vector<vector<IndexType> > nodeIdx; //
		vector<vector<ScalarType> > coords;

		vector<int> tp; vector<float> tw; 
		m_grid->Coord(curLine.start_.x, curLine.start_.y, tp, tw); 
		nodeIdx.push_back(tp);
		coords.push_back(tw);
		tp.clear(); tw.clear();

		m_grid->Coord(curLine.middle_.x, curLine.middle_.y, tp, tw); 
		nodeIdx.push_back(tp);
		coords.push_back(tw);
		tp.clear(); tw.clear();

		m_grid->Coord(curLine.end_.x, curLine.end_.y, tp, tw); 
		nodeIdx.push_back(tp);
		coords.push_back(tw);
		tp.clear(); tw.clear();

		//record the for the global
		m_linesNodeIdx.push_back(nodeIdx);
		m_lineNodeCoor.push_back(coords);
		// assign the value

		//left:  b- a/2 -c/2 = 0.;
		//for a:
		for (IndexType i = 0; i < 4; ++ i)
		{
			IndexType nid = nodeIdx[0][i];
			eleT.push_back(T(nEquation, nid, (- 0.5) * lineW * coords[0][i]));
		}

		//for b:
		for (IndexType i = 0; i < 4; ++ i)
		{
			IndexType nid = nodeIdx[1][i];
			eleT.push_back(T(nEquation, nid, lineW * coords[1][i]));
		}

		//divide the line into n parts, not only 2 parts.

		//for c:
		for (IndexType i = 0; i < 4; ++ i)
		{
			IndexType nid = nodeIdx[2][i];
			eleT.push_back(T(nEquation, nid, (- 0.5) * lineW * coords[2][i]));
		}

		//right
		rB(nEquation,0) = 0.;
		rB(nEquation,1) = 0.;

	}//end for lines.

	//fix the boundary edges/nodes
	//.--.--.
	//.     .
	//.--.--.

	if (isFixBoundary)
	{
		for (IndexType i = 0; i < m_grid->_boundaries.size(); ++ i, ++ nEquation)
		{
			IndexType corId = m_grid->_boundaries[i];
			eleT.push_back(T(nEquation,corId,1));
			rB(nEquation,0) = m_oriCtlCoor(corId,0);
			rB(nEquation,1) = m_oriCtlCoor(corId,1);
		}

	}else
	{
		//only for each corners
		ScalarType cweight = 1000;
		for (IndexType j = 0; j < _corners.size(); ++ j, ++ nEquation)
		{
			IndexType corId = m_grid->_corner[j];
			eleT.push_back(T(nEquation,corId,1*cweight));
			rB(nEquation,0) = cweight * _corners[j].x;
			rB(nEquation,1) = cweight * _corners[j].y;

		}
	}

	//assign the left value
	lA.setFromTriplets(eleT.begin(),eleT.end());

	//guess the initial solvers//using the original coordinate
	// the original coordinates of the control points

	for (IndexType i = 0; i < cols; ++ i)
	{
		initX(i,0) = m_oldCtlCoor(i,0);
		initX(i,1) = m_oldCtlCoor(i,1);
	}

	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<ScalarType> > lscg;

	lscg.compute(lA);

	lscg.solveWithGuess(rB,initX);

	lX = lscg.solve(rB);

	m_newCtlCoor = lX; //update coordinate.

}

//piecewise for each quad
void PieceHomo::warpingPiecewiseHomo()
{
	IndexType cols = m_grid->numofnodes(); 

	IndexType rows = 0;

	assert(m_grid->numoffaces() == m_oldHomo.size() );

	bool isFixBoundary = true;

	if (isFixBoundary)
	{
		rows = 4 * m_grid->numoffaces() + m_tgFeaturePs.size() + m_grid->_boundaries.size();
	}else
	{
		rows = 4 * m_grid->numoffaces() + m_tgFeaturePs.size()+ 4;
	}

	Eigen::SparseMatrix<ScalarType> lA;
	lA.resize(rows,cols);

	typedef Eigen::Triplet<ScalarType> T;

	vector<T> eleT;

	MatrixXX rB,lX,initX;
	rB.resize(rows,2);
	lX.resize(rows,2);
	initX.resize(rows,2);

	IndexType nEquation = 0;

	//for each quad
	for (IndexType fId = 0; fId < m_grid->numoffaces(); ++ fId, nEquation += 4)
	{
		int a, b, c, d; 
		a = m_grid->_faces[fId].v0; b = m_grid->_faces[fId].v1;
		c = m_grid->_faces[fId].v2; d = m_grid->_faces[fId].v3;

		Point2f saCoor,sbCoor,scCoor,sdCoor; 
		Point2f taCoor,tbCoor,tcCoor,tdCoor;

		//original coordinates
		saCoor.x = m_oldCtlCoor(a,0);
		saCoor.y = m_oldCtlCoor(a,1);    

		sbCoor.x = m_oldCtlCoor(b,0);
		sbCoor.y = m_oldCtlCoor(b,1);

		scCoor.x = m_oldCtlCoor(c,0);
		scCoor.y = m_oldCtlCoor(c,1);

		sdCoor.x = m_oldCtlCoor(d,0);
		sdCoor.y = m_oldCtlCoor(d,1);

		//transformed coordinate
		transHomo(saCoor,taCoor,m_oldHomo[fId]);
		transHomo(sbCoor,tbCoor,m_oldHomo[fId]);
		transHomo(scCoor,tcCoor,m_oldHomo[fId]);
		transHomo(sdCoor,tdCoor,m_oldHomo[fId]);

		//a-b
		ScalarType abdx = taCoor.x - tbCoor.x;
		ScalarType abdy = taCoor.y - tbCoor.y;

		eleT.push_back(T(nEquation,a,1));
		eleT.push_back(T(nEquation,b,-1));

		rB(nEquation,0) = abdx;
		rB(nEquation,1) = abdy;

		//b-c
		ScalarType bcdx = tbCoor.x - tcCoor.x;
		ScalarType bcdy = tbCoor.y - tcCoor.y;

		eleT.push_back(T(nEquation + 1,b,1));
		eleT.push_back(T(nEquation + 1,c,-1));

		rB(nEquation + 1,0) = bcdx;
		rB(nEquation + 1,1) = bcdy;

		//c-d
		ScalarType cddx = tcCoor.x - tdCoor.x;
		ScalarType cddy = tcCoor.y - tdCoor.y;

		eleT.push_back(T(nEquation + 2,c,1));
		eleT.push_back(T(nEquation + 2,d,-1));

		rB(nEquation + 2,0) = cddx;
		rB(nEquation + 2,1) = cddy;

		//d-a
		ScalarType dadx = tdCoor.x - taCoor.x;
		ScalarType dady = tdCoor.y - taCoor.y;

		eleT.push_back(T(nEquation + 3,d,1));
		eleT.push_back(T(nEquation + 3,a,-1));

		rB(nEquation + 3,0) = dadx;
		rB(nEquation + 3,1) = dady;

	}

	//for each feature

	for (IndexType i = 0; i < m_srFeaturePs.size(); ++ i,++ nEquation)
	{
		for (IndexType j = 0; j < _np[i].size(); ++j)
		{
			IndexType feaId = _np[i][j];
			eleT.push_back(T(nEquation,feaId,_cd[i][j]));
		}

		rB(nEquation,0) = m_tgFeaturePs[i].x;
		rB(nEquation,1) = m_tgFeaturePs[i].y;

	}

	//for fix corners
	if (isFixBoundary)
	{
		for (IndexType i = 0; i < m_grid->_boundaries.size(); ++ i, ++ nEquation)
		{
			IndexType corId = m_grid->_boundaries[i];
			eleT.push_back(T(nEquation,corId,1));
			rB(nEquation,0) = m_oriCtlCoor(corId,0);
			rB(nEquation,1) = m_oriCtlCoor(corId,1);
		}

	}else
	{
		//only for each corners
		for (IndexType j = 0; j < _corners.size(); ++ j, ++ nEquation)
		{
			IndexType corId = m_grid->_corner[j];
			eleT.push_back(T(nEquation,corId,1));
			rB(nEquation,0) = _corners[j].x;
			rB(nEquation,1) = _corners[j].y;
		}
	}

	//assign the left value
	lA.setFromTriplets(eleT.begin(),eleT.end());

	//guess the initial solvers//using the original coordinate
	// the original coordinates of the control points
// 	Loggger<<lA<<endl;
// 	Loggger<<rB<<endl;

// 	char dist[1024];
// 	sprintf(dist,"warpLa.txt");
// 	FILE* in_dist = fopen(dist,"w");
// 
// 	for (IndexType i = 0; i < rows; ++ i)
// 	{
// 		for (IndexType j = 0; j < cols; ++ i)
// 		{
// 			fprintf(in_dist,"%f ",rB(i,j));
// 		}
// 	}


	for (IndexType i = 0; i < cols; ++ i)
	{
		initX(i,0) = m_oldCtlCoor(i,0);
		initX(i,1) = m_oldCtlCoor(i,1);
	}

	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<ScalarType> > lscg;

	lscg.compute(lA);

	lscg.solveWithGuess(rB,initX);

	lX = lscg.solve(rB);

	m_newCtlCoor = lX; //update coordinate.

	//Loggger<<lX<<endl;
}

void PieceHomo::estimateHomo()
{
	//equation numbers: 12*m*m + 2*m*(m+1),m is the division number.
	IndexType nFaces = m_grid->numoffaces();// equals to the size of homo

	//for test
	//IndexType nFaces = 1;

	//bool isSmooth = true;
	bool isSmooth = false;

	IndexType rows = 0;

	if (isSmooth)
	{
		rows = 12* nFaces + m_grid->f2f.size();
	}else
	{
        rows = 12 * nFaces;
	}

	// old coordinates and new coordinates of the control points
	IndexType cols = 9 * nFaces;

	Eigen::SparseMatrix<ScalarType> lA;
	lA.resize(rows,cols);

	typedef Eigen::Triplet<ScalarType> T;

	vector<T> eleT;

	MatrixXX rB,lX,initX;
	rB.resize(rows,1);
	lX.resize(cols,1);
	initX.resize(cols,1);

    //fill lA and rB
	for (IndexType i = 0; i < nFaces; ++ i)
	{
		int a, b, c, d; 
		a = m_grid->_faces[i].v0; b = m_grid->_faces[i].v1;
		c = m_grid->_faces[i].v2; d = m_grid->_faces[i].v3;

		//for point a
		ScalarType lax = m_oldCtlCoor(a,0);
		ScalarType lay = m_oldCtlCoor(a,1);

		eleT.push_back(T(12*i,9*i,    lax));
		eleT.push_back(T(12*i,9*i + 1,lay));
		eleT.push_back(T(12*i,9*i + 2,1));

		eleT.push_back(T(12*i + 1,9*i + 3,lax));
		eleT.push_back(T(12*i + 1,9*i + 4,lay));
		eleT.push_back(T(12*i + 1,9*i + 5,1));

		eleT.push_back(T(12*i + 2,9*i + 6,lax));
		eleT.push_back(T(12*i + 2,9*i + 7,lay));
		eleT.push_back(T(12*i + 2,9*i + 8,1));

		ScalarType rax = m_newCtlCoor(a,0);
		ScalarType ray = m_newCtlCoor(a,1);

		rB(12*i,0) = rax;
		rB(12*i + 1,0) = ray;
		rB(12*i + 2,0) = 1;

		//for point b
		ScalarType lbx = m_oldCtlCoor(b,0);
		ScalarType lby = m_oldCtlCoor(b,1);

		eleT.push_back(T(12*i + 3, 9*i , lbx));
		eleT.push_back(T(12*i + 3, 9*i + 1,lby));
		eleT.push_back(T(12*i + 3, 9*i + 2,1));

		eleT.push_back(T(12*i + 4, 9*i + 3,lbx));
		eleT.push_back(T(12*i + 4, 9*i + 4,lby));
		eleT.push_back(T(12*i + 4, 9*i + 5,1));

		eleT.push_back(T(12*i + 5, 9*i + 6,lbx));
		eleT.push_back(T(12*i + 5, 9*i + 7,lby));
		eleT.push_back(T(12*i + 5, 9*i + 8,1));

		ScalarType rbx = m_newCtlCoor(b,0);
		ScalarType rby = m_newCtlCoor(b,1);

		rB(12*i + 3,0) = rbx;
		rB(12*i + 4,0) = rby;
		rB(12*i + 5,0) = 1;

		//for point c
		ScalarType lcx = m_oldCtlCoor(c,0);
		ScalarType lcy = m_oldCtlCoor(c,1);

		eleT.push_back(T(12*i + 6, 9*i, lcx));
		eleT.push_back(T(12*i + 6, 9*i + 1,lcy));
		eleT.push_back(T(12*i + 6, 9*i + 2,1));

		eleT.push_back(T(12*i + 7, 9*i + 3,lcx));
		eleT.push_back(T(12*i + 7, 9*i + 4,lcy));
		eleT.push_back(T(12*i + 7, 9*i + 5,1));

		eleT.push_back(T(12*i + 8, 9*i + 6,lcx));
		eleT.push_back(T(12*i + 8, 9*i + 7,lcy));
		eleT.push_back(T(12*i + 8, 9*i + 8,1));

		ScalarType rcx = m_newCtlCoor(c,0);
		ScalarType rcy = m_newCtlCoor(c,1);

		rB(12*i + 6,0) = rcx;
		rB(12*i + 7,0) = rcy;
		rB(12*i + 8,0) = 1;

		//for point d
		ScalarType ldx = m_oldCtlCoor(d,0);
		ScalarType ldy = m_oldCtlCoor(d,1);

		eleT.push_back(T(12*i + 9, 9*i, ldx));
		eleT.push_back(T(12*i + 9, 9*i + 1,ldy));
		eleT.push_back(T(12*i + 9, 9*i + 2,1));

		eleT.push_back(T(12*i + 10, 9*i + 3,ldx));
		eleT.push_back(T(12*i + 10, 9*i + 4,ldy));
		eleT.push_back(T(12*i + 10, 9*i + 5,1));

		eleT.push_back(T(12*i + 11, 9*i + 6,ldx));
		eleT.push_back(T(12*i + 11, 9*i + 7,ldy));
		eleT.push_back(T(12*i + 11, 9*i + 8,1));

		ScalarType rdx = m_newCtlCoor(d,0);
		ScalarType rdy = m_newCtlCoor(d,1);

		rB(12*i + 9, 0) = rdx;
		rB(12*i + 10,0) = rdy;
		rB(12*i + 11,0) = 1;
	}


	//consider the smooth of the homos

	if (isSmooth)
	{
		IndexType equaId = 12* nFaces;

		for (IndexType i = 0; i < m_grid->f2f.size(); ++ i, ++ equaId)
		{
			IndexType f1 = m_grid->f2f[i].first;
			IndexType f2 = m_grid->f2f[i].second;

			IndexType sf1 = 9 * f1;
			IndexType sf2 = 9 * f2;

			for (IndexType j = 0; j < 9; ++ j)
			{
				eleT.push_back(T(equaId, sf1 + j,1));
				eleT.push_back(T(equaId, sf2 + j,-1));
			}
		}
	}

	//for initialize the homo matrix

	bool isInit = false;

	if (m_oldHomo.size() == nFaces)
	{
		isInit = true;

		for (IndexType i = 0; i < nFaces;  ++ i)
		{
			Mat temp = m_oldHomo[i];

			initX(9*i,0)     = temp.at<double>(0,0);
			initX(9*i + 1,0) = temp.at<double>(0,1);
			initX(9*i + 2,0) = temp.at<double>(0,2);

			initX(9*i + 3,0) = temp.at<double>(1,0);
			initX(9*i + 4,0) = temp.at<double>(1,1);
			initX(9*i + 5,0) = temp.at<double>(1,2);

			initX(9*i + 6,0) = temp.at<double>(2,0);
			initX(9*i + 7,0) = temp.at<double>(2,1);
			initX(9*i + 8,0) = temp.at<double>(2,2);

		}
	}

	//assign the left value
	lA.setFromTriplets(eleT.begin(),eleT.end());

	//for testing
//  	Loggger<<lA<<endl;
//  	Loggger<<rB<<endl;

	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<ScalarType> > lscg;

	lscg.compute(lA);

	if (isInit)
	{
	  lscg.solveWithGuess(rB,initX);
	}

	lX = lscg.solve(rB);

	//Loggger<<lX<<endl;

	//end least square 
	//assign the results for homo graphics

	m_homo.clear();
	for (IndexType i = 0; i < nFaces; ++ i)
	{
		MatrixXX temp = lX.block(9*i,0,9,1);

		//Loggger<<temp<<endl;

		Mat homo(3,3,CV_32FC1,Scalar::all(1.));
	    	
        homo.at<float>(0,0) = temp(0,0);homo.at<float>(0,1) = temp(1,0);homo.at<float>(0,2) = temp(2,0);
		homo.at<float>(1,0) = temp(3,0);homo.at<float>(1,1) = temp(4,0);homo.at<float>(1,2) = temp(5,0);
		homo.at<float>(2,0) = temp(6,0);homo.at<float>(2,1) = temp(7,0);homo.at<float>(2,2) = temp(8,0);

		m_homo.push_back(homo);

		//Loggger<<homo<<endl;

	}

	m_oldCtlCoor = m_newCtlCoor;

}

void PieceHomo::renderWarpedImg()
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

	MatrixXX quadCoor,textureCoor;
	IndexType nQuad = m_grid->numoffaces();
	quadCoor.setZero(4*nQuad,2);
	textureCoor.setZero(4*nQuad,2);

	for (int i = 0;i < nQuad; ++ i) 
	{
		int a, b, c, d; 
		a = m_grid->_faces[i].v0; b = m_grid->_faces[i].v1;
		c = m_grid->_faces[i].v2; d = m_grid->_faces[i].v3;
		   
		//a
		quadCoor(4*i,0) = m_newCtlCoor(a,0); 
		quadCoor(4*i,1) = m_newCtlCoor(a,1);

		textureCoor(4*i,0) = _texCoord[a].x;
		textureCoor(4*i,1) = _texCoord[a].y;

		//b
		quadCoor(4*i + 1,0) = m_newCtlCoor(b,0); 
		quadCoor(4*i + 1,1) = m_newCtlCoor(b,1);

		textureCoor(4*i + 1,0) = _texCoord[b].x;
		textureCoor(4*i + 1,1) = _texCoord[b].y;

		//c
		quadCoor(4*i + 2,0) = m_newCtlCoor(c,0); 
		quadCoor(4*i + 2,1) = m_newCtlCoor(c,1);
		textureCoor(4*i + 2,0) = _texCoord[c].x;
		textureCoor(4*i + 2,1) = _texCoord[c].y;

		//d
		quadCoor(4*i + 3,0) = m_newCtlCoor(d,0); 
		quadCoor(4*i + 3,1) = m_newCtlCoor(d,1);

		textureCoor(4*i + 3,0) = _texCoord[d].x;
		textureCoor(4*i + 3,1) = _texCoord[d].y;
	}

	//drawLines();  //

	emit drawDeformedImg(m_srImg,m_heigh,m_width,quadCoor,textureCoor);

// 	Loggger<<quadCoor<<endl;
// 	Loggger<<textureCoor<<endl;
	//for testing the deformation 
// 	ScalarType nxCoor,nyCoor;
//     vector<ScalarType> allError;
// 	allError.clear();
// 	for (IndexType i = 0; i < m_srFeaturePs.size(); ++ i)
// 	{
// 		nxCoor = 0.;
// 		nyCoor = 0.;
// 
// 		for (IndexType j = 0; j < _np[i].size(); ++j)
// 		{
// 			IndexType feaId = _np[i][j];//vertex idx
// 
// 			ScalarType nxConCoor = m_newCtlCoor(feaId,0);//quadCoor(feaId,0);//error
// 			ScalarType nyConCoor = m_newCtlCoor(feaId,1);
// 
// 			nxCoor += (nxConCoor * _cd[i][j]);
// 			nyCoor += (nyConCoor * _cd[i][j]);
// 		}
// 		ScalarType txCoor = m_tgFeaturePs[i].x;
// 		ScalarType tyCoor = m_tgFeaturePs[i].y;
// 		ScalarType err = 0.;
// 		err = sqrt( (nxCoor - txCoor)*(nxCoor - txCoor) + (nyCoor - tyCoor)*(nyCoor - tyCoor));
// 		allError.push_back(err);
// 	}
// 
// 	for (IndexType i = 0; i < allError.size(); ++ i)
// 	{
// 		printf("%d matching error is %f.\n",i,allError[i]);
// 	}
// 	m_rendWid = new dTexture;
// 	m_rendWid->drawDeformedImg(m_srImg,m_heigh,m_width,quadCoor,textureCoor);
// 	m_rendWid->setFixedHeight(m_heigh);
// 	m_rendWid->setFixedWidth(m_width);
// 	m_rendWid->show();
// 	m_rendWid->updateGL();


}


void PieceHomo::drawLines()
{
	Mat leftT;
	m_srImg.copyTo(leftT);

	Scalar color(0,0,255);

	for (IndexType i = 0; i < m_srLines.size(); ++ i)
	{
		vector<vector<IndexType> > nIdx = m_linesNodeIdx[i];
		vector<vector<ScalarType>> nCoor = m_lineNodeCoor[i];

		vector<IndexType> stNIdx = nIdx[0]; //start point
		vector<IndexType> edNIdx = nIdx[2]; // end point

		vector<ScalarType> stCoor = nCoor[0];
		vector<ScalarType> edCoor = nCoor[2];

		CvPoint2D32f stP, edP;

		float spx = 0.;
		float spy = 0.;

		float epx = 0.;
		float epy = 0.;

		//draw new lines
		for (IndexType corIdx = 0; corIdx < 4; ++ corIdx)
		{
			spx += stCoor[corIdx] * m_newCtlCoor(stNIdx[corIdx],0); 
			spy += stCoor[corIdx] * m_newCtlCoor(stNIdx[corIdx],1); 

			epx += edCoor[corIdx] * m_newCtlCoor(edNIdx[corIdx],0);
			epy += edCoor[corIdx] * m_newCtlCoor(edNIdx[corIdx],1);
		}

		stP.x = spx; stP.y = spy;
		edP.x = epx; edP.y = epy;

		cv::line(leftT,stP,edP,color);

		//draw old lines
		Scalar color2(255,0,0);

		 spx = 0.;
		 spy = 0.;

		 epx = 0.;
		 epy = 0.;

		for (IndexType corIdx = 0; corIdx < 4; ++ corIdx)
		{
			spx += stCoor[corIdx] * m_oldCtlCoor(stNIdx[corIdx],0); 
			spy += stCoor[corIdx] * m_oldCtlCoor(stNIdx[corIdx],1); 

			epx += edCoor[corIdx] * m_oldCtlCoor(edNIdx[corIdx],0);
			epy += edCoor[corIdx] * m_oldCtlCoor(edNIdx[corIdx],1);
		}

		stP.x = spx; stP.y = spy;
		edP.x = epx; edP.y = epy;

		cv::line(leftT,stP,edP,color2);

	}

	Mat tt;
	cvtColor(leftT,tt,CV_RGB2BGR);
	imshow("Lines_deformation", tt);
	imwrite(".\\tempImages\\lines_deformation.jpg",tt);

}

void PieceHomo::mat2Qimage(const Mat& srcMat, QImage& desQImage)
{
	IndexType nChannel=srcMat.channels();

	if (nChannel==3)
	{			
		Mat srcTemp = Mat(srcMat.rows,srcMat.cols,srcMat.type());
		//cvtColor(srcMat,srcTemp,CV_BGR2RGB);
		//srcTemp = srcMat.copy();
		srcMat.copyTo(srcTemp);
		desQImage=QImage(srcTemp.cols,srcTemp.rows,QImage::Format_RGB888);
		memcpy(desQImage.bits(),srcTemp.data,srcTemp.cols*srcTemp.rows*3*sizeof(unsigned char));	
	}
	else if (nChannel==4||nChannel==1)
	{
		desQImage = QImage((const unsigned char*)srcMat.data,srcMat.cols,srcMat.rows,srcMat.step,QImage::Format_ARGB32);
	}
}

CvPoint2D32f PieceHomo::transform(Mat* R, ScalarType x, ScalarType y)
{
	float X, Y, Z;
	float h0, h1, h2, h3, h4, h5, h6, h7, h8;
	if(R->cols==2&&R->rows==2)
	{
		h0 = R->at<float>(0,0); h1 = R->at<float>(0,1); 
		h2 = R->at<float>(1,0); h3 = R->at<float>(1,1);

		X = h0*x+h1*y; Y = h2*x+h3*y;
	}
	else if(R->cols==3&&R->rows==3)
	{

		h0 = R->at<float>(0,0); h1 = R->at<float>(0,1); h2 = R->at<float>(0,2);
		h3 = R->at<float>(1,0); h4 = R->at<float>(1,1); h5 = R->at<float>(1,2);
		h6 = R->at<float>(2,0); h7 = R->at<float>(2,1); h8 = R->at<float>(2,2); 
		Z = 1./(h6*x + h7*y + h8); 
		X = (h0*x + h1*y + h2)*Z;
		Y = (h3*x + h4*y + h5)*Z;
	}

	CvPoint2D32f q; q.x = X; q.y = Y; 
	return q; 
}

void PieceHomo::transHomo(Point2f& oriPs, Point2f& transfPs, Mat& homo)
{
	Matrix33 eHomo;

	//Loggger<<homo<<endl;

	cv2eigen(homo,eHomo);

	//Loggger<<eHomo<<endl;

	ScalarType zCoor = oriPs.x * eHomo(2,0) + oriPs.y * eHomo(2,1) + eHomo(2,2);
	assert(zCoor > 1e-4 || zCoor < -1e-4);

    ScalarType xCoor = oriPs.x * eHomo(0,0) + oriPs.y * eHomo(0,1) + eHomo(0,2);
	ScalarType yCoor = oriPs.x * eHomo(1,0) + oriPs.y * eHomo(1,1) + eHomo(1,2);

	transfPs.x = xCoor/zCoor;
	transfPs.y = yCoor/zCoor;
}

void PieceHomo::bilinearInterpo(Mat& resItp)
{
	IndexType hg = resItp.rows;
	IndexType wd = resItp.cols;
	resItp.create(hg,wd,m_srImg.type());
	resItp.setTo(0);

	//only interpolate the pixel located on the middle of the quad

	IndexType nQuad = m_grid->numoffaces();

	//  a--b
	//  |  |
	//  d--c
	Point2f pA,pB,pC,pD;

	for (int i = 0;i < nQuad; ++ i) 
	{
		int a, b, c, d; 
		a = m_grid->_faces[i].v0; b = m_grid->_faces[i].v1;
		c = m_grid->_faces[i].v2; d = m_grid->_faces[i].v3;

		//a
		pA.x = m_newCtlCoor(a,0);//x
		pA.y = m_newCtlCoor(a,1);//y

		//b
		pB.x = m_newCtlCoor(b,0);//x
		pB.y = m_newCtlCoor(b,1);//y

		//c
		pC.x = m_newCtlCoor(c,0);//x
		pC.y = m_newCtlCoor(c,1);//y

		//d
		pD.x = m_newCtlCoor(d,0);//x
		pD.y = m_newCtlCoor(d,1);//y


	}

}

void PieceHomo::initFeatureCoordinate()
{

	homography(m_srFeaturePs,m_tgFeaturePs, _H); 

	m_oldHomo.resize(m_grid->numoffaces(),_H);// initial for each quad face

	for (int i=0;i<m_srFeaturePs.size();i++)
	{
		float x, y; x = m_srFeaturePs[i].x; y = m_srFeaturePs[i].y; 
		vector<int> tp; vector<float> tw; 
		m_grid->Coord(x, y, tp, tw); 
		_np.push_back(tp); _cd.push_back(tw);
	}

}

void PieceHomo::homography(vector<CvPoint2D32f>& srPoints, vector<CvPoint2D32f>& tgPoints,
						   Mat& _homo)
{
	//using the default transformation
	_homo.create(3,3,CV_64FC1);
	_homo.setTo(0.);
	_homo += Mat::eye(3,3,CV_64FC1);

	// another choice
// 	vector<Point2f> srPs,tgPs; 
// 	if (srPoints.size() < 4 )
// 	{
// 		Loggger<<"The size of feature points is small!.\n";
// 		_homo.create(3,3,CV_64FC1);
// 		_homo.setTo(0.);
// 		_homo += Mat::eye(3,3,CV_64FC1);
// 
// 		//Loggger<<_homo<<endl;
// 		return ;
// 	}

// 	for (int i = 0; i < srPoints.size(); ++ i)
// 	{
// 		Point2f srtemp,tgtemp;
// 
// 		srtemp.x = srPoints[i].x;
// 		srtemp.y = srPoints[i].y;
// 		tgtemp.x = tgPoints[i].x;
// 		tgtemp.y = tgPoints[i].y;
// 		srPs.push_back(srtemp);
// 		tgPs.push_back(tgtemp);
// 	}
// 
// 	_homo = findHomography(srPs,tgPs,CV_RANSAC);

}

void PieceHomo::refineKeyps(Mat& cufImg, vector<KeyPoint>& oriKeyPs, vector<KeyPoint>& outKeyPs)
{
// 	assert(oriKeyPs.size() > 0 && m_sltRect.size() > 0);
// 
// 	if (m_sltRect[0].width == 0 || m_sltRect[0].height == 0)
// 	{
// 		Loggger<<"Fail to initialize the select regions.\n";
// 		return;
// 	}
// 
// 	Mat resROI;
// 	Mat bg;
// 
// 	bg = * (SingleFrameSet::get_instance().getBackGround().body);
// 
// 	bg(m_sltRect[0]).copyTo(resROI);
// 
// 	for (auto vIt = oriKeyPs.begin(); vIt != oriKeyPs.end(); ++ vIt )
// 	{
// 		KeyPoint kPs = * vIt;
// 
// 		Point2f keyPosk = kPs.pt;
// 
// 		Vec3b bgPix = resROI.at<Vec3b>(keyPosk.y,keyPosk.x);
// 
// 		Vec3b curPix = cufImg.at<Vec3b>(keyPosk.y,keyPosk.x);
// 
// 		if ( isDifferent(bgPix,curPix))
// 		{
// 			outKeyPs.push_back(kPs);
// 
// 		}else
// 		{
// 			resROI.at<Vec3b>(keyPosk.y,keyPosk.x) = bgMask;
// 			cv::circle(resROI, keyPosk, 5, cv::Scalar(255.), -1);
// 		}
// 	}



	// 	Mat temp;
	// 
	// 	cvtColor(resROI,temp,CV_RGB2BGR);
	// 
	// 	imshow("Outliers", temp);


}

bool PieceHomo::matchingDescriptor(const vector<KeyPoint>& queryKeyPoints, 
									const vector<KeyPoint>& trainKeyPoints, 
									const Mat& queryDescriptors,
									const Mat& trainDescriptors, 
									Ptr<DescriptorMatcher>& descriptorMatcher, 
									Mat& homo,
									vector<DMatch>& matchPs,
									bool enableRatioTest /* = true */)
{
	vector<vector<DMatch>> m_knnMatches;  

	if (enableRatioTest)  
	{  
		cout<<"KNN Matching"<<endl;  
		const float minRatio = 1.f / 1.5f;  
		descriptorMatcher->knnMatch(queryDescriptors,trainDescriptors,m_knnMatches,2);  
		for (size_t i=0; i<m_knnMatches.size(); i++)  
		{  
			const cv::DMatch& bestMatch = m_knnMatches[i][0];  
			const cv::DMatch& betterMatch = m_knnMatches[i][1];  
			float distanceRatio = bestMatch.distance / betterMatch.distance;  
			if (distanceRatio < minRatio)  
			{  
				matchPs.push_back(bestMatch);  
			}  
		}  

	}  
	else  
	{  
		cout<<"Cross-Check"<<endl;  
		Ptr<cv::DescriptorMatcher> BFMatcher(new cv::BFMatcher(cv::NORM_HAMMING, true));  
		BFMatcher->match(queryDescriptors,trainDescriptors, matchPs );  
	}  

	float homographyReprojectionThreshold = 1.0;  
	//bool homographyFound = refineMatchesWithHomography(  
	//queryKeyPoints,trainKeyPoints,homographyReprojectionThreshold,matchPs,homo);  

	// 	if (!homographyFound) 
	// 	{
	// 		return false; 
	// 
	// 	}else  
	// 	{  

	return true;
	//}
}

bool PieceHomo::createDetectorDescriptorMatcher(const string& detectorType, 
												 const string& descriptorType, 
												 const string& matcherType, 
												 Ptr<FeatureDetector>& featureDetector,
												 Ptr<DescriptorExtractor>& descriptorExtractor,
												 Ptr<DescriptorMatcher>& descriptorMatcher )
{
	cout << "< Creating feature detector, descriptor extractor and descriptor matcher ..." << endl;  
	if (detectorType=="SIFT"||detectorType=="SURF")  
		initModule_nonfree();  

	featureDetector = FeatureDetector::create( detectorType );  
	descriptorExtractor = DescriptorExtractor::create( descriptorType );  
	descriptorMatcher = DescriptorMatcher::create( matcherType );  
	cout << ">" << endl;  
	bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );  
	if( !isCreated )  
		cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;  
	return isCreated; 

}