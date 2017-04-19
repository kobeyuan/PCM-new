#include "dlg_StopMotion.h"
#include "dlg_canvas.h"

#include "opencv2/video/video.hpp"
#include "opencv2/video/background_segm.hpp"


StopMotionUI::StopMotionUI() : m_video(SingleFrameSet::get_instance().getTotVideo() )
{
	ui.setupUi(this);
	
	m_timeCtl = new QTimer(this);

	m_curFrameIdx = 0;

	m_isPlay = false;

	m_keyFrame[0] = m_keyFrame[1] = 0;

	m_showPairIdx[0] = 0;
	
	m_showPairIdx[1] = 1;

	m_isVideo = false;

	m_isInitImSeq = false;

	m_gWeight = 0.5;

	m_thresDiff = 15;

	m_curPairIdx = 0;

	m_isDealLeftImg = true;// default for dealing the left image first.

	m_isOKDeformation = false;

	m_isPressDel = false;

	m_bgId = m_srtId = m_lenSeq = 0;

	m_band = 2;

	if (!m_dialCut)
	{
		delete m_dialCut; 
		m_dialCut = nullptr;
	}

	_blender = NULL;

	if (!m_initCurFrame.empty())
	{
		m_initCurFrame.release();
	}

	m_isOpticalFlow = true;

	m_lfeaIdx = 0;
	m_rfeaIdx = 0;
	m_lMatchingPs.clear();
	m_rMatchingPs.clear();

	m_totSeamPsLeft = m_totSeamPsright = 0;
	m_totSeamPsLeft = m_totEdgeRight = 0;

	m_lMPixels.clear();
	m_rMPixels.clear();
	m_seamPts.clear();
	
	m_isLeftBg = false;

	m_isRightBg = false;

	_gcutLabels.clear();

	m_isFullS = false;

	m_isRightMask = true;

	m_delPCoor.setX(0);

	m_delPCoor.setY(0);

	m_gloMatPsLeft.clear();

	m_gloMatPsRight.clear();
}

StopMotionUI::~StopMotionUI()
{
	if (m_timeCtl) 
	{
		delete m_timeCtl;
		m_timeCtl = NULL;
	}

	if (m_dialCut)
	{
		delete m_dialCut;
		m_dialCut = NULL;
	}

	if (m_pieceHomo)
	{
		delete m_pieceHomo;
		m_pieceHomo = NULL;
	}

	if (_blender)
	{
		delete _blender;
		_blender = NULL;
	}

}

void StopMotionUI::initConnect()
{
	connect(m_timeCtl,SIGNAL(timeout()), this, SLOT(nextFrame()));

	connect(ui.playAndPause, SIGNAL(clicked()),this, SLOT( playPause() ) );

	connect(ui.playSlow, SIGNAL(clicked()), this, SLOT(makeSlow() ) );

	connect(ui.StopPlay, SIGNAL(clicked()), this, SLOT(playStop() ) );

	connect(ui.previousFrame, SIGNAL(clicked()), this, SLOT(preFrame()) );

	connect(ui.nextFrame, SIGNAL(clicked()), this, SLOT(nextFrame() ) );

	connect(ui.isSelectKeyFrameA, SIGNAL( clicked() ), this, SLOT( selectKeyFrameA() ) );

	connect(ui.isSelectKeyFrameB, SIGNAL( clicked() ), this, SLOT( selectKeyFrameB() ) );

	//for matching
	connect(ui.lfeaselect,SIGNAL( clicked() ), this, SLOT( selectLeftFeatures() ) );

	connect(ui.rfeaselect,SIGNAL( clicked() ), this, SLOT( selectRighFeatures() ) );

	connect(ui.leftSeamIdx,SIGNAL(valueChanged(int) ), this, SLOT( feaIdxChangesLeft(int) ));

	connect(ui.rightSeamIdx,SIGNAL(valueChanged(int) ), this, SLOT( feaIdxChangesRight(int) ));

	connect(ui.matchingDeform,SIGNAL(clicked()), this, SLOT(deformImgInteraction() ));

	//end for matching
	connect(ui.isOpticalflow, SIGNAL(clicked()), this, SLOT(isOpticalflow()) );

	connect(ui.isBackground, SIGNAL( clicked() ), this, SLOT( selectBackground() ) );

	connect(ui.savePairs, SIGNAL(clicked()) ,this, SLOT(savePairs() ) );

	connect(ui.inVideoView, SIGNAL(valueChanged(int) ), this, SLOT( changeCurFrameIdx(int) ) );

	connect(ui.inVideoView, SIGNAL(valueChanged(int) ), this, SLOT( showTransForSelection(int) ) );

	connect(ui.pairIndex, SIGNAL(valueChanged(int) ), this, SLOT( showPairIdx(int) ) );

	connect(ui.frameIdx, SIGNAL(valueChanged(int) ), this, SLOT( assignFrameIdx(int) ) );

	// for autoselection 
	connect(ui.backgId, SIGNAL(valueChanged(int) ), this, SLOT(setBackgId(int)) );
	connect(ui.startId, SIGNAL(valueChanged(int) ), this, SLOT(setstartId(int)) );
	connect(ui.rangeFrame, SIGNAL(valueChanged(int) ), this, SLOT(setLen(int)) );
	connect(ui.band, SIGNAL(valueChanged(int) ), this, SLOT(setBand(int)) );
	//connect(ui.outputVideo, SIGNAL(changeRect()), this, SLOT(setROIs() ) );

	connect(ui.outputVideo, SIGNAL(changeRect()), this, SLOT(getROIsFromSign() ) );

	//for video 
	connect(ui.inputVideo, SIGNAL(changeRect()), this, SLOT(getROIsFromSign() ) );

	connect(ui.outputVideo, SIGNAL(deleteRect()), this, SLOT(setDeletePress() ) );

	connect(ui.outputVideo, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromImages()) );

	connect(ui.inputVideo, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromLeftImages()) );

	connect(ui.showBG,SIGNAL(clicked()), this, SLOT(showBackGround() ) );

	connect(ui.CombineOperation, SIGNAL(clicked()), this, SLOT(combinePairImages() ));

	connect(ui.newPipeline,  SIGNAL(clicked()), this, SLOT(tryDeformROIOnlyMethod() ));

	connect(ui.OKforDeformation, SIGNAL(clicked()), this, SLOT(okDeformation() ));

	//connect(ui.MergingImages, SIGNAL(clicked()), this, SLOT(fuseTwoROIs()) );

	connect(ui.MergingImages, SIGNAL(clicked()), this, SLOT(newFuseTwoROIs()) );

	connect(ui.UndoStroke, SIGNAL(clicked()), this, SLOT(undoStrokeDraw()) );

	connect(ui.handDectect, SIGNAL(clicked()), this, SLOT(handDectect() ) );

	connect(ui.outputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showCoorRGB(const QPoint&, const QRgb& )) );

	connect(ui.outputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showLabels(const QPoint&, const QRgb& )) );

	connect(ui.outputVideo, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	connect(ui.inputVideo, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	connect(ui.inputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showCoorRGB(const QPoint&, const QRgb& )) );

	connect(ui.inputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showLabels(const QPoint&, const QRgb& )) );

	//for delete the matching pairs
	connect(ui.outputVideo, SIGNAL(propaCoor(const QPoint& )), this, SLOT(obtainCoor(const QPoint& ) ));

	connect(ui.inputVideo, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoor(const QPoint& ) ));

	//delete which one?
	connect(ui.outputVideo, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));

	connect(ui.inputVideo, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));

	//for global matching points
	connect(ui.middleResults, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorLeft(const QPoint& ) ));

	connect(ui.middleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorRight(const QPoint& ) ));

	//delete matching points
	connect(ui.middleResults, SIGNAL(delGloCoor(const QPoint&)), this, SLOT(deleteGloMatchingPsLeft(const QPoint&)) );

	connect(ui.middleResultsB, SIGNAL(delGloCoor(const QPoint&)), this, SLOT(deleteGloMatchingPsRight(const QPoint&)));

	//update texture image

	//connect(m_sMotion, SIGNAL(transferImg(Mat&)),this,SLOT(drawEditImg(Mat&)) );

	connect(ui.gCutWeight, SIGNAL( valueChanged(int) ), this,SLOT(updateGraphweight(int)) );

	connect(ui.thresDifferent, SIGNAL( valueChanged(int) ), this,SLOT(updateTresDifference(int)) );

	connect(ui.afterConbine, SIGNAL(clicked()), this, SLOT(afterCombine() ) );

	connect(ui.selectROI, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.selectBackG, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.selectForeG, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.addMatching, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	//for automatic selection of pair 
	connect(ui.autoSelect, SIGNAL(clicked()), this, SLOT(autoSelectionPair() ) );

	//poisson smooth
	connect(ui.pSmooth, SIGNAL(clicked()), this, SLOT(pSmooth()) );

	connect(ui.sPSmooth, SIGNAL(clicked()), this, SLOT(stopPSmooth()) );

	//full screen
	connect(ui.fullScreen, SIGNAL(clicked()), this, SLOT(fullScreen()) );

	//using background image for completion
	connect(ui.setLeftBG,SIGNAL(clicked()), this, SLOT(setLeftWindowasBG()) );
	//connect(ui.playSlow, SIGNAL(clicked()), this, SLOT() );
	//connect(ui.playSlow, SIGNAL(clicked()), this, SLOT() );
	//connect(ui.startAlgorithm, SIGNAL(clicked(bool)), this, SLOT(run(bool)));

	//background subtraction

	connect(ui.bgSub, SIGNAL(clicked()), this, SLOT(detectFGObjects()) );

}

void StopMotionUI::getStatusTool()
{
	bool isSelect = ui.selectROI->isChecked();
	bool isBack = ui.selectBackG->isChecked();
	bool isFore = ui.selectForeG->isChecked();

	//for adding matching information
	bool isAdd = ui.addMatching->isChecked();

	ui.outputVideo->setStatusTool(isSelect,isBack,isFore,isAdd);

	ui.inputVideo->setStatusTool(isSelect,isBack,isFore,isAdd);

	ui.middleResults->setStatusTool(isSelect,isBack,isFore,isAdd);

	ui.middleResultsB->setStatusTool(isSelect,isBack,isFore,isAdd);

}
void StopMotionUI::playPause()
{
 	m_timeCtl->start();
 
 	ui.previousFrame->setEnabled(false);
 
 	ui.nextFrame->setEnabled(false);
 
 	ui.selectPairStatue->setEnabled(false);
 
 	m_isPlay = true;
 
 	ui.isSelectKeyFrameA->setEnabled(false);
 
 	ui.isSelectKeyFrameB->setEnabled(false);
 
//  	if (!m_isPlay)
//  	{
//  		m_timeCtl->setInterval(m_delay);
//  
//  		m_timeCtl->start();
//  
//  		ui.previousFrame->setEnabled(false);
//  
//  		ui.nextFrame->setEnabled(false);
//  
//  		ui.selectPairStatue->setEnabled(false);
//  
//  		m_isPlay = true;
//  
//  		ui.isSelectKeyFrameA->setEnabled(true);
//  
//  		ui.isSelectKeyFrameB->setEnabled(true);
//  
//  		ui.playAndPause->setEnabled(true);
//  
//  	}else
//  	{
//  		m_timeCtl->stop();
//  
//  		ui.previousFrame->setEnabled(true);
//  
//  		ui.nextFrame->setEnabled(true);
//  
//  		ui.selectPairStatue->setEnabled(true);
//  
//  		ui.playAndPause->setEnabled(true);
//  
//  		m_isPlay = false;
//  	}

}

void StopMotionUI::preFrame()
{
	m_curFrameIdx --;
	if (m_curFrameIdx >= 0)
	{
		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

		m_video>>m_curImgData;

		ui.inImgIdx->setText(QString::number(m_curFrameIdx) );
		Mat tempImg;
		cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			tempImg.cols,tempImg.rows,QImage::Format_RGB888);
		ui.inputVideo->setImage(&m_curShowImg);		
		ui.inputVideo->updateDisplayImage();


	}else
	{
		Loggger<<"Bound to beginning!.\n";
	}
}

void StopMotionUI::nextFrame()
{
    assert(m_curFrameIdx <= m_totFrames);

	m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

	m_video>>m_curImgData;

	if ( !m_curImgData.empty() )
	{
		Mat tempImg;

		cvtColor(m_curImgData,tempImg,CV_BGR2RGB);

		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			                   tempImg.cols,tempImg.rows,QImage::Format_RGB888);

		ui.inputVideo->setImage(&m_curShowImg);	

		ui.inputVideo->updateDisplayImage();

		++ m_curFrameIdx;

		ui.inVideoView->setValue(m_curFrameIdx);

		ui.inImgIdx->setText(QString::number(m_curFrameIdx) );

	}
	else
	{
		Loggger<<"The Final!.\n";
		m_timeCtl->stop();
	}
}


void StopMotionUI::showPairIdx(int idx)
{
	if (!m_isInitImSeq)
	{
		m_isInitImSeq = initPairFromImageSeq();
	}

	if (!m_initCurFrame.empty())
	{
		m_initCurFrame.release(); //for new cufFrame data
	}

	// refresh the strokes
	m_isDealLeftImg = true;

	//for deformation
	m_dfLeftROI.release();
	m_gcLeftROI.release();
	//end for clear the temp results.

	if (idx >= 0 && idx < m_keyPairs.size() )
	{
		// get the first image of the pair,means that the idx is the small one.
		m_curPairIdx = idx;

		IndexType showIdA = get_aframe_from_key(m_keyPairs[idx].second);

		IndexType showIdB = get_bframe_from_key(m_keyPairs[idx].second);

		m_showPairIdx[0] = showIdA;

		m_showPairIdx[1] = showIdB;

	}else
	{
		Loggger<<"Out Of Size, please initialize the key-pairs.\n";

		return;
	}
}


void StopMotionUI::assignFrameIdx(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_curFrameIdx = fId;
	}
}


void StopMotionUI::setBackgId(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_bgId = fId;
	}

}

void StopMotionUI::setstartId(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_srtId = fId;
	}

}

void StopMotionUI::setLen(int fId)
{
	if ( fId > 0 && fId < (m_totFrames - m_bgId) )
	{
		m_lenSeq = fId;
	}

}

void StopMotionUI::setBand(int _band)
{
	if (_band <=2 || _band > 1000)
	{
		return;
	}

	m_band = _band;
}

void StopMotionUI::combinePairImages() //the algorithm interface- stop motion class
{
	
 	if (/*!m_isVideo && */!m_isInitImSeq) //read an image sequence;
 	{
		m_isInitImSeq = initPairFromImageSeq();
	}

	m_isRightBg = false;
	m_isLeftBg = false;



	initInteracrionLabels();

	m_sMotion = new StopMotion(m_keyPairs);

	connect(m_sMotion, SIGNAL(transferImg(Mat&,Point&, int,int,MatrixXX&,MatrixXX&) ),this,
		SLOT(drawEditImg(Mat&,Point&, int,int,MatrixXX&,MatrixXX&)) );

	m_sMotion->setShowPairId(m_showPairIdx);
	
	ui.outputVideo->clearSltRegion();
	ui.inputVideo->clearSltRegion();
	ui.outputVideo->setRectZero();
	ui.inputVideo->setRectZero();

	m_isPressDel = false;

// 	if (!m_isVideo)
// 	{

	    IndexType aKeyFrame = m_showPairIdx[0];

		IndexType activePair = m_showPairIdx[1];

		assert(activePair >=0 && activePair < m_totFrames );

		SingleFrameSet& set_ = SingleFrameSet::get_instance();

		if (!m_initCurFrame.empty())
		{
		   //set_[m_showPairIdx[0]].body = *m_initCurFrame; //if the current frame changed by gm_outImg, here in order to set the oringal one.
		   //m_initCurFrame.copyTo();
		   set_[m_showPairIdx[0]].setBoby(&m_initCurFrame);
		   //Loggger<<"Assign new data!.\n";
		}

		SingleFrame& aImage = set_[aKeyFrame];
		SingleFrame& bImage = set_[activePair];

		aImage.sltRect.clear();
		bImage.sltRect.clear();

		m_gloMatPsLeft.clear();
		m_gloMatPsRight.clear();

		//drawKeyFrame(activePair);
		drawKeyFrame(aKeyFrame,false);
		drawKeyFrame(activePair, true);
	//}

}

void StopMotionUI::progressBar()
{
// 	IndexType maxSize = 2 * m_keyPairs.size(); 
// 	ui.selectPairStatue->setRange(1,maxSize);

	ui.selectPairStatue->setValue( m_keyPairs.size() );
}

void StopMotionUI::selectKeyFrameA()
{

	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_keyFrame[0] = m_curFrameIdx;

		drawKeyFrame(m_keyFrame[0]);

		if (!ui.isSelectKeyFrameB->isChecked() )
		{
            ui.isSelectKeyFrameB->setEnabled(false); // make sure the pair do not selected the same frame
		}
		   
	}

	bool isACheck = ui.isSelectKeyFrameA->isChecked();

	bool isBCheck = ui.isSelectKeyFrameB->isChecked();

	if (isACheck && isBCheck)
	{
		addPairs();
	}
}

void StopMotionUI::isOpticalflow()
{
	m_isOpticalFlow = !(ui.isOpticalflow->isChecked());
}

void StopMotionUI::selectKeyFrameB()
{

	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_keyFrame[1] = m_curFrameIdx;

		if (!ui.isSelectKeyFrameA->isChecked() )
		{
			ui.isSelectKeyFrameA->setEnabled(false); // make sure the pair do not selected the same frame
		}
	}

	bool isACheck = ui.isSelectKeyFrameA->isChecked();

	bool isBCheck = ui.isSelectKeyFrameB->isChecked();

	if (isACheck && isBCheck)
	{
		addPairs();
	}
}

void StopMotionUI::selectBackground()
{
	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_backGroundIdx = m_curFrameIdx;

		ui.isBackground->setChecked(true);
	}
}
void StopMotionUI::addPairs()
{
	assert(ui.isSelectKeyFrameA->isChecked() &&
		   ui.isSelectKeyFrameB->isChecked() );

		if (    m_keyFrame[0] >=0 && m_keyFrame[0]<= m_totFrames
			 && m_keyFrame[1] >=0 && m_keyFrame[1]<= m_totFrames)
		{
			 IndexType pairSize = m_keyPairs.size();
			 IndexType pIdx = pairSize + 1;
			 IndexType paKeysIdx = frame_frame_to_key(m_keyFrame[0], m_keyFrame[1]);
			 m_keyPairs.push_back(std::make_pair(pIdx, paKeysIdx) );

		}

		//drawKeyFrame(m_keyFrame[0]);

		m_keyFrame[0] = m_keyFrame[1] = 0;

		ui.isSelectKeyFrameA->setChecked(false);

		ui.isSelectKeyFrameB->setChecked(false);

		progressBar();
}


void StopMotionUI::savePairs()
{
	writeImages();
}

void StopMotionUI::drawKeyFrame(IndexType fIdx)
{
	if (!m_isVideo)
	{
		Loggger<<" Not a video.\n";

		SingleFrameSet& set_ = SingleFrameSet::get_instance();

		SingleFrame& curImage = set_[fIdx];

		Mat imgMat = *set_[fIdx].body;

		mat2Qimage(imgMat,m_curShowImg);

		ui.outputVideo->setImage(&m_curShowImg);	

		ui.outputVideo->updateDisplayImage();

	}else
	{
		assert(fIdx >= 0 && fIdx <= m_totFrames);

		m_video.set(CV_CAP_PROP_POS_FRAMES, fIdx );

		m_video>>m_curImgData;

		if ( !m_curImgData.empty() )
		{
			Mat tempImg;

			//ui.outputVideo->setIsVideo(true);

			cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
			m_curShowImg = QImage((const unsigned char*)(tempImg.data),
				tempImg.cols,tempImg.rows,QImage::Format_RGB888);
			ui.outputVideo->setImage(&m_curShowImg);		
			ui.outputVideo->updateDisplayImage();
		}
		else
		{
			Loggger<<"The Select is over!.\n";
			//m_timeCtl->stop();
		}
	}

}

void StopMotionUI::drawKeyFrame(IndexType fIdx, bool isRight)
{
	if (!m_isVideo)
	{
		//Loggger<<" Not a video.\n";

		SingleFrameSet& set_ = SingleFrameSet::get_instance();

		SingleFrame& curImage = set_[fIdx];

		Rect slt1;
		if (!curImage.sltRect.empty())
		{
			slt1 = curImage.sltRect[0]; //update, still have a rectangle
		}

		Mat imgMat = *set_[fIdx].body;

		if (isRight)
		{

			if (m_isPressDel)// delete the rectangle only, keeping the existing strokes. 
			{
				if (!m_rightmaskImgStack.empty())
				{
					m_rmaskImage = m_rightmaskImgStack.top();
				}else
				{
					m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
						QImage::Format_ARGB32);
				}

			}else
			{
				while (!m_rightmaskImgStack.empty())
				{
					m_rightmaskImgStack.pop();
				}

				m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
					QImage::Format_ARGB32);
			}

		    mat2Qimage(imgMat,m_curShowImg);

			//ui.strokeLevel->setText(QString::number(m_rightmaskImgStack.size() ));

 			if (slt1.width > 0 && slt1.height > 0)
 			{
				ui.outputVideo->setRect(slt1);
				//ui.outputVideo->setRectZero();
			}

			ui.outputVideo->setImage(&m_curShowImg);

			if (m_rmaskImage.width() > 0)
			{
			    ui.outputVideo->setMaskImage(m_rmaskImage);
			}

			ui.outputVideo->updateDisplayImage();

		}else
		{

			if (m_isPressDel)// delete the rectangle only, keeping the existing strokes. 
			{
				if (!m_leftmaskImgStack.empty())
				{
					m_lmaskImage = m_leftmaskImgStack.top();
				}else
				{
					m_lmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
						QImage::Format_ARGB32);
				}

			}else
			{
				while (!m_leftmaskImgStack.empty())
				{
					m_leftmaskImgStack.pop();
				}

				m_lmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
					QImage::Format_ARGB32);
			}

 			//if (slt1.width > 0 && slt1.height > 0)
 			//{
			   ui.inputVideo->setRect(slt1);
			//}

			mat2Qimage(imgMat,m_curShowLeftImg);

// 			m_lmaskImage = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
// 				QImage::Format_ARGB32);

			ui.inputVideo->setImage(&m_curShowLeftImg);	
			if (m_lmaskImage.width() > 0)
			{
			   ui.inputVideo->setMaskImage(m_lmaskImage);
			}

			ui.inputVideo->updateDisplayImage();

		}


	}else
	{
		assert(fIdx >= 0 && fIdx <= m_totFrames);

		m_video.set(CV_CAP_PROP_POS_FRAMES, fIdx );

		m_video>>m_curImgData;

		if ( !m_curImgData.empty() )
		{
			Mat tempImg;

			//ui.outputVideo->setIsVideo(true);

			cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
			m_curShowImg = QImage((const unsigned char*)(tempImg.data),
				tempImg.cols,tempImg.rows,QImage::Format_RGB888);

			if (isRight)
			{
				ui.outputVideo->setImage(&m_curShowImg);		
				ui.outputVideo->updateDisplayImage();
			}else
			{
				ui.inputVideo->setImage(&m_curShowImg);		
				ui.inputVideo->updateDisplayImage();
			}

		}
		else
		{
			Loggger<<"The Select is over!.\n";
			//m_timeCtl->stop();
		}
	}
}

void StopMotionUI::showTransForSelection(int curFrame)
{
	assert(m_keyFrame[0] >=0 && m_keyFrame[0] < m_totFrames);
	assert(curFrame >=0 && curFrame < m_totFrames);

	if (ui.isSelectKeyFrameA->isChecked() )
	{
		Mat aKey,bKey,transAB;

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_keyFrame[0] );
		m_video>>aKey;

		m_video.set(CV_CAP_PROP_POS_FRAMES, curFrame );
		m_video>>bKey;

		translucentImages(aKey,bKey,0.5, transAB);

		Mat temp;

		cvtColor(transAB,temp,CV_BGR2RGB);

		m_curShowImg = QImage((const unsigned char*)(temp.data),
			temp.cols,temp.rows,QImage::Format_RGB888);

		ui.outputVideo->setImage(&m_curShowImg);		
		ui.outputVideo->updateDisplayImage();
	}

}

void StopMotionUI::writeImages()
{
	IndexType pSize = m_keyPairs.size();
	assert(pSize > 0);

    char fName[1024];
	char sName[1024];

	SingleFrameSet::get_instance().clear();

	wirteBackground();

	IndexType fId = 0;

	SingleFrame* curF = new SingleFrame;

	for (IndexType pIdx = 0; pIdx < pSize; ++ pIdx)
	{
		IndexType keyOrder = m_keyPairs[pIdx].first;
		IndexType keyId = m_keyPairs[pIdx].second;

		IndexType aId = get_aframe_from_key(keyId);
		IndexType bId = get_bframe_from_key(keyId);

		sprintf(fName,".\\Pairs\\%.2d-a.jpg",keyOrder);
		sprintf(sName,".\\Pairs\\%.2d-b.jpg",keyOrder);

		SingleFrame* curFa = new SingleFrame;

		Mat imgTemp, mTemp;

		m_video.set(CV_CAP_PROP_POS_FRAMES, aId);

		m_video>>imgTemp;

		cvtColor(imgTemp,mTemp,CV_BGR2RGB);

		curFa->setBoby(&mTemp);

		curFa->frameIdx = fId;

		SingleFrameSet::get_instance().push_back(curFa);

		++ fId;

		imwrite(fName,imgTemp);


		SingleFrame* curFb = new SingleFrame;

		m_video.set(CV_CAP_PROP_POS_FRAMES, bId);

		m_video>>imgTemp;

		cvtColor(imgTemp,mTemp,CV_BGR2RGB);

		curFb->setBoby(&mTemp);

		curFb->frameIdx = fId;

		SingleFrameSet::get_instance().push_back(curFb);

		++ fId;

		imwrite(sName,imgTemp);
	}

	//m_isVideo = false;

	Loggger<<"Done for saving images.\n";

}

void StopMotionUI::wirteBackground()
{
	if (!!m_backGroundIdx)
	{
		char fName[1024];

        sprintf(fName,".\\Pairs\\background.jpg"); 

		Mat imgTemp, mTemp;

		SingleFrame* cufF = new SingleFrame;

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_backGroundIdx);

		m_video>>imgTemp;

		cvtColor(imgTemp,mTemp,CV_BGR2RGB);

		cufF->setBoby(&mTemp);

		cufF->frameIdx = -1;

		SingleFrameSet::get_instance().setBackGround(cufF);

		imwrite(fName,imgTemp);
	}
}

void StopMotionUI::displayAnImage(bool isRight, cv::Mat& showImg)
{
	QImage temp;

	mat2Qimage(showImg,temp);

	if (isRight)
	{
		ui.outputVideo->setImage(&temp);	

		ui.outputVideo->updateDisplayImage();

	}else
	{
		ui.inputVideo->setRect(m_sltRect[0]);

		ui.inputVideo->setImage(&temp);	

		ui.inputVideo->updateDisplayImage();
	}

}

void StopMotionUI::disPlayPureImage(bool isRight, QImage& showImg)
{
	if (isRight)
	{
		ui.outputVideo->setImage(&showImg);	

		ui.outputVideo->updateDisplayImage();

	}else
	{
		ui.inputVideo->setRect(m_sltRect[0]);

		ui.inputVideo->setImage(&showImg);	

		ui.inputVideo->updateDisplayImage();
	}
}

void StopMotionUI::makeSlow()
{
	IndexType dy_ = (m_delay * 10);

	 if ( dy_ >=0 && dy_ < 10000)
	 {
		 m_delay = dy_;

		 m_timeCtl->setInterval(m_delay);
	 }
}

void StopMotionUI::playStop()
{
	m_timeCtl->stop();

	m_isPlay = false;

	ui.previousFrame->setEnabled(true);

	ui.nextFrame->setEnabled(true);

	ui.selectPairStatue->setEnabled(true);

	ui.playAndPause->setEnabled(true);

	ui.isSelectKeyFrameA->setEnabled(true);

	ui.isSelectKeyFrameB->setEnabled(true);
}

void StopMotionUI::makeFast()
{
// 	IndexType dy_ = (m_delay * 0.1);
// 
// 	if ( dy_ >=0 && dy_ < 10000)
// 	{
// 		m_delay = dy_;
// 
// 		m_timeCtl->setTimerType(Qt::CoarseTimer);
// 
// 		m_timeCtl->setInterval(m_delay);
// 	}

}

void StopMotionUI::changeCurFrameIdx(int curFrmIdx)
{
	ui.previousFrame->setEnabled(false);

	ui.nextFrame->setEnabled(false);

	ui.playAndPause->setEnabled(false);

	ui.isSelectKeyFrameA->setEnabled(false);

	ui.isSelectKeyFrameB->setEnabled(false);


	if (curFrmIdx >= 0 && curFrmIdx < m_totFrames)
	{
		m_curFrameIdx = curFrmIdx;

		ui.inImgIdx->setText(QString::number(m_curFrameIdx) );

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

		m_video>>m_curImgData;

		Mat tempImg;
		cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
			tempImg.cols,tempImg.rows,QImage::Format_RGB888);
		ui.inputVideo->setImage(&m_curShowImg);		
		ui.inputVideo->updateDisplayImage();
	}


	ui.previousFrame->setEnabled(true);

	ui.nextFrame->setEnabled(true);

	ui.playAndPause->setEnabled(true);

	ui.isSelectKeyFrameA->setEnabled(true);

	ui.isSelectKeyFrameB->setEnabled(true);
}

void StopMotionUI::initBaseInfo()
{
	if (!m_isVideo) return;

	m_rate = m_video.get(CV_CAP_PROP_FPS);

	assert(m_rate>1);

	m_delay = 1000/m_rate;

	m_totFrames = m_video.get(CV_CAP_PROP_FRAME_COUNT);

	ui.inVideoView->setRange(0, m_totFrames - 1);

}

bool StopMotionUI::initPairFromImageSeq()
{
	SingleFrameSet& set_ = SingleFrameSet::get_instance();

	IndexType imSize = set_.size();

	if (imSize%2)
	{
		Loggger<<"Its not a good input about image sequences.\n";

		return false;
	}

	m_keyPairs.clear();
	
	m_totFrames = imSize;

	IndexType keyPaIdx = 0;

	for (IndexType im_idx = 0; im_idx < imSize; im_idx+=2, ++ keyPaIdx)
	{
		SingleFrame& tempA = set_[im_idx];
		SingleFrame& tempB = set_[im_idx + 1];

		IndexType aIdx = tempA.frameIdx;
		IndexType bIdx = tempB.frameIdx;

		IndexType fKey = frame_frame_to_key(aIdx,bIdx);

		m_keyPairs.push_back(std::make_pair(keyPaIdx, fKey) );

	}

	progressBar();

	return true;
}

// this is a old version function, just for test, the real function corresponding combine is the after()
void StopMotionUI::setROIs()
{
	//*m_sltRect = ui.outputVideo->getRect(); 
	ui.outputVideo->getRect(m_sltRect);

	assert(m_showPairIdx[0] >=0 && m_showPairIdx[1] <m_totFrames);

	SingleFrameSet& set_ = SingleFrameSet::get_instance();

	set_[m_showPairIdx[0] ].setSecRect(m_sltRect);

	set_[m_showPairIdx[1] ].setSecRect(m_sltRect);

	// test graph cut function
	Mat rImg =  *set_[m_showPairIdx[1] ].body;

	m_sMotion->setGraphCutWeight(m_gWeight);

	testGraphCut(rImg);

	//

// 	Mat roiL,roiR,tempL,tempR;
// 
// 	Mat signLHand,signRHand,outImg;
// 
// 
// 	m_sMotion->diffRoiBackG(m_showPairIdx[0],roiL); // detect the bg region
// 	roiL.copyTo(signLHand);
// 	
// 	m_sMotion->diffRoiBackG(m_showPairIdx[1],roiR);//symmetry situation
// 	roiR.copyTo(signRHand);
// 
// 
// 	cvtColor(signLHand,tempL,CV_RGB2BGR);
// 	imshow("A-ROI",tempL);
// 	cvtColor(signRHand,tempR,CV_RGB2BGR);
// 	imshow("B-ROI",tempR);
// 
// 	initStopMotionClass();
// 
// 
// 
// 
// 	m_sMotion->matchAndDeform(roiL,roiR,outImg);
// 
//  	MatrixXXi handMK; // mark the bg/fg
//  
//  	//handMarkerGenerate(roiL,handMK);
// 	//handMarkerGenerate(roiR,handMK);
// 
// 
//     handMarkerGenerate(outImg,roiR,handMK);
// 
// 	m_sMotion->initImageLabel(roiR);
// 
// // 
// // 	simpleHandClassify(&roiL,&signLHand);  //detect the hand region and visualize those pixel
// // 	simpleHandClassify(&roiR,&signRHand);
// 
// // 	//m_sMotion->matchAndDeform(signLHand,signRHand,outImg);
// // 	m_sMotion->matchAndDeform(roiL,roiR,outImg);
// 
// 	MatrixXXi outPutMark;
// 
// 	Mat outPutImg;
// 
// 	//m_sMotion->handRemoval(roiL,roiR,handMK,outPutMark);
// 
// 	//m_sMotion->imageGenerate(roiL,roiR,handMK,outPutImg);
// 
// 	m_sMotion->handRemoval(outImg,roiR,handMK,outPutMark);
// 
// 	m_sMotion->imageGenerate(outImg,roiR,outPutMark,outPutImg);// default R->L.

}

void StopMotionUI::getROIsFromSign()
{
	m_sltRect.clear();

	if (m_isVideo)
	{
      ui.inputVideo->getRect(m_sltRect);
	}else
	{

	  ui.outputVideo->getRect(m_sltRect);

	  assert(m_showPairIdx[0] >=0 && m_showPairIdx[1] <m_totFrames);

	  SingleFrameSet& set_ = SingleFrameSet::get_instance();

	  set_[m_showPairIdx[0] ].setSecRect(m_sltRect);

	  set_[m_showPairIdx[1] ].setSecRect(m_sltRect);

	  //ui.inputVideo->updateDisplayImage();
	  drawKeyFrame(m_showPairIdx[0],false);
	  drawKeyFrame(m_showPairIdx[1],true);
	}


	//show the ROIs;
	initalROIsShow();

	//test
// 	printf("ROI size = %d", m_sltRect.size());
// 	if (m_sltRect.size() > 0)
// 	{
// 		printf("Height = %d", m_sltRect[0].height);
// 		printf("Width = %d", m_sltRect[0].width);
// 	}

}

void StopMotionUI::getMaskFromImages()
{
   m_rmaskImage = ui.outputVideo->getMaskImage(); 

   //assert()
   IndexType width = m_rmaskImage.width();
   IndexType height = m_rmaskImage.height();

   IndexType oriW = m_curShowImg.width();
   IndexType oriH = m_curShowImg.height();

   assert(width == oriW && height == oriH);

   m_rightmaskImgStack.push(m_rmaskImage);

   QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
   ui.strokeLevel->setText(strSize_str);

   m_markLabels.resize(height,width);

  QRgb bgColor;  //();
  QRgb fgColor;

  bgColor = qRgb(bgMask[0], bgMask[1],bgMask[2]);
  fgColor = qRgb(fgMask[0], fgMask[1],fgMask[2]);

   for (IndexType hIdx = 0; hIdx < height; ++ hIdx)
   {
	   for (IndexType wIdx = 0; wIdx < width; ++ wIdx)
	   {
		   QRgb curColor = m_rmaskImage.pixel(wIdx,hIdx);

		   if (curColor == bgColor)
		   {
			   m_markLabels(hIdx,wIdx) = GC_BGD;
		   }
		   else if (curColor == fgColor)
		   {
			   m_markLabels(hIdx,wIdx) = GC_FGD;

		   }else
		   {
			   m_markLabels(hIdx,wIdx) = GC_PR_BGD;
		   }
	   }
   }

}

void StopMotionUI::getMaskFromImage(QImage& _maskIMg, MatrixXXi& _labels)
{
	if (m_sltRect.size() <=0)
	{
		Loggger<<"Please select the ROIs!.\n";
		return;
	}

	IndexType strRectX = m_sltRect[0].x;
	IndexType strRectY = m_sltRect[0].y;

	IndexType width = m_sltRect[0].width;

	IndexType height = m_sltRect[0].height;

	assert(width>0 && height>0);

	_labels.resize(height,width);

	_labels.setZero();

	QRgb bgColor;  //();
	QRgb fgColor;

	bgColor = qRgb(bgMask[0], bgMask[1],bgMask[2]);
	fgColor = qRgb(fgMask[0], fgMask[1],fgMask[2]);

	for (IndexType hIdx = 0; hIdx < height; ++ hIdx)
	{
		for (IndexType wIdx = 0; wIdx < width; ++ wIdx)
		{
			QRgb curColor = _maskIMg.pixel(wIdx + strRectX, hIdx + strRectY);

			if (curColor == bgColor)
			{
				_labels(hIdx,wIdx) = GC_FGD;
			}
			else if (curColor == fgColor)
			{
				_labels(hIdx,wIdx) = GC_FGD;

			}else
			{
				_labels(hIdx,wIdx) = GC_BGD;
			}
		}
	}
}



void StopMotionUI::getMaskFromLeftImages()
{
	 m_lmaskImage = ui.inputVideo->getMaskImage(); 

	IndexType width = m_lmaskImage.width();
	IndexType height = m_lmaskImage.height();

	IndexType oriW = m_curShowImg.width();
	IndexType oriH = m_curShowImg.height();

	assert(width == oriW && height == oriH);

	m_leftmaskImgStack.push(m_lmaskImage);

	QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
	ui.strokeLevel->setText(strSize_str);

	m_markLeftLabels.resize(height,width);

	QRgb bgColor; 
	QRgb fgColor;

	bgColor = qRgb(bgMask[0], bgMask[1],bgMask[2]);
	fgColor = qRgb(fgMask[0], fgMask[1],fgMask[2]);

	for (IndexType hIdx = 0; hIdx < height; ++ hIdx)
	{
		for (IndexType wIdx = 0; wIdx < width; ++ wIdx)
		{
			QRgb curColor = m_lmaskImage.pixel(wIdx,hIdx);

			if (curColor == bgColor)
			{
				m_markLeftLabels(hIdx,wIdx) = GC_BGD;
			}
			else if (curColor == fgColor)
			{
				m_markLeftLabels(hIdx,wIdx) = GC_FGD;

			}else
			{
				m_markLeftLabels(hIdx,wIdx) = GC_PR_BGD;
			}
		}
	}
}

 
void StopMotionUI::initalROIsShow() 
{
	Mat roiL,roiR,tempL,tempR, signLHand, signRHand;

	Mat outImg;

	initStopMotionClass(); 

	if (m_sltRect.size() <= 0)
	{
		Loggger<<"Please select the ROIs.\n";
		return;
	}

	m_sMotion->copyROIFromKey(m_showPairIdx[0],roiL);
	m_sMotion->copyROIFromKey(m_showPairIdx[1],roiR);

	roiL.copyTo(m_leftROI);
	roiR.copyTo(m_rightROI);

	Mat comImg,tComImg;
	translucentImages(m_leftROI,m_rightROI,0.7,comImg);

	cvtColor(comImg,tComImg,CV_RGB2BGR);
	imshow("Combine Image",tComImg);

	//show ROIs --
	cvtColor(roiL,tempL,CV_RGB2BGR);
	//imshow("A-ROI",tempL);
	cvtColor(roiR,tempR,CV_RGB2BGR);
	//imshow("B-ROI",tempR);

	tempL.copyTo(m_lROIshow);
	tempR.copyTo(m_rROIshow);

	showROIs(tempL,tempR);

}

void StopMotionUI::afterCombine()//registration button
{

	//11-20
	//add a global transformation for the two rois
	Mat gloDefLeft;
	bool isGloD = globalDeformation(m_leftROI,m_rightROI,gloDefLeft); //roiL->roiR

	//all of the ROIs--629
	//using piecewise homography method to align two images first.

	Mat outImg;

 	if (m_isOpticalFlow)
 	{
		if (isGloD)
		{
 	         m_sMotion->matchWithFlow(gloDefLeft,m_rightROI,outImg); //with optical flow method
		}else
		{
			 m_sMotion->matchWithFlow(m_leftROI,m_rightROI,outImg); //with optical flow method
		}

 	}else
 	{
		if (isGloD)
		{
 	        m_sMotion->matchAndDeform(gloDefLeft,m_rightROI,outImg);// featured based method;
		}else
		{
 	        m_sMotion->matchAndDeform(m_leftROI,m_rightROI,outImg);// featured based method;
		}

 	}
 
 	updateLeftWindowImage(outImg);

	//after homography transformation
	Mat comImg,tComImg;
	translucentImages(outImg,m_rightROI,0.7,comImg);


	cvtColor(comImg,tComImg,CV_RGB2BGR);
	imshow("Homography Image",tComImg);

 	//
	outImg.copyTo(m_dfLeftROI);

}


void StopMotionUI::tryDeformROIOnlyMethod()
{
	Mat roiL,roiR,tempL,tempR, signLHand, signRHand;

	Mat outImg;

	initStopMotionClass(); 

	m_sMotion->copyROIFromKey(m_showPairIdx[0],roiL);
	m_sMotion->copyROIFromKey(m_showPairIdx[1],roiR);


	cvtColor(roiL,tempL,CV_RGB2BGR);
	imshow("A-ROI",tempL);
	cvtColor(roiR,tempR,CV_RGB2BGR);
	imshow("B-ROI",tempR);

	m_isDealLeftImg = true;

// 	while (!m_isOKDeformation)
// 	{
		MatrixXXi roiMark;

		if (m_markLeftLabels.cols() > 0)
		{
			roiMark = m_markLeftLabels.block(m_sltRect[0].y,m_sltRect[0].x,m_sltRect[0].height,m_sltRect[0].width);

			checkTrainData(roiMark);
		}

		m_sMotion->matchByROIOnly(roiL,roiR,outImg,roiMark); // estimate the affine matrix for foreground only.

		updateLeftWindowImage(outImg);

		m_rightROI = roiR;

		m_dfLeftROI = outImg;

	//}


// 	if (m_isOKDeformation)
// 	{
// 		MatrixXXi handMarkers, stillPixel;
// 		//handMarkerGenerate(roiR,handMarkers); // RGB method
// 		m_sMotion->setOkDeformation(m_isOKDeformation);
// 
// 		m_sMotion->handMarkerRGBYUV(roiR,handMarkers); // combine RGB and YUV
// 
// 		//m_sMotion->itaHandMarkers(roiR,handMarkers);// do nothing
// 
// 		m_sMotion->getinitTrainData(roiR,handMarkers, stillPixel); //with unknown labels
// 
// 		m_diffLabels = handMarkers;
// 
// 		assignInitialLabels();
// 	
// 		 MatrixXXi roiMarkHand;
// 	 
// 		 if (m_markLabels.cols() > 0)
// 		 {
// 			 roiMarkHand = m_markLabels.block(m_sltRect[0].y,m_sltRect[0].x,m_sltRect[0].height,m_sltRect[0].width);
// 
// 			 m_sMotion->setInteractiveStatue(
// 				 m_sMotion->initImageLabelWithInteractive(roiMarkHand));
// 		 }
// 
// 		 MatrixXXi outPutMark;
// 
// 		 Mat outPutImg;
// 	    
// 		 if (!checkTrainData(m_initLabels))
// 		 {
// 			 Loggger<<" The train data are wrong!.\n";
// 
// 			 return;
// 		 }
// 
// 		 m_sMotion->handRemoval(outImg,roiR,m_initLabels,outPutMark);
// 	    
// 		 checkStillPixels(outPutMark,stillPixel);
// 
// 		 m_sMotion->imageGenerate(outImg,roiR,outPutMark,outPutImg);// default R->L.
// 
// 		 Loggger<<"Combine process is OK.\n";
	//}


}


void StopMotionUI::initStopMotionClass()
{
		m_sMotion->setRect(m_sltRect);
		m_sMotion->setGraphCutWeight(m_gWeight);
		m_sMotion->setTresChange(m_thresDiff);
		m_sMotion->setCurPairIdx(m_curPairIdx);
		m_isLeftBg = false;
}

void StopMotionUI::mat2Qimage(const Mat& srcMat, QImage& desQImage)
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

void StopMotionUI::qImage2Mat(const QImage& srcQImage,Mat& desMat)
{
	cv::Mat matQ=cv::Mat(srcQImage.height(), srcQImage.width(), CV_8UC4, (uchar*)srcQImage.bits(), srcQImage.bytesPerLine()); 
	desMat=Mat(matQ.rows,matQ.cols,CV_8UC3);
	int from_to[] = { 0,0, 1,1, 2,2 };
	cv::mixChannels(&matQ,1,&desMat,1,from_to,3);
}

void StopMotionUI::handDectect()
{
	SingleFrameSet& imgSet = SingleFrameSet::get_instance();

	assert( imgSet.size() > 0 );

	IndexType fSize = imgSet.size();

	Mat* curMat;
	Mat outMat;

 	curMat = imgSet[m_showPairIdx[0] ].body;

 	curMat->copyTo(outMat);

	simpleHandClassify(curMat,&outMat);

	displayAnImage(false,*curMat);

	displayAnImage(true,outMat);
}

void StopMotionUI::simpleHandClassify(const Mat* srImg, Mat* tgImg)
{
	 cv::Mat_<cv::Vec3b>::const_iterator itbe = srImg->begin<cv::Vec3b>();

	 cv::Mat_<cv::Vec3b>::const_iterator itend = srImg->end<cv::Vec3b>();

     cv::Mat_<cv::Vec3b>::iterator itTgbe = tgImg->begin<cv::Vec3b>();

     for ( ; itbe != itend; ++ itbe, ++ itTgbe)
     {
		  Vec3b clr = (*itbe);

		  bool isH = isHandPixel(clr);

		  if (isH)
		  {
			  (*itTgbe) = fgMask;
		  }

     }

}

void StopMotionUI::handMarkerGenerate(const Mat& srImg, MatrixXXi& handMK)
{

	IndexType hight = srImg.rows;

	IndexType width = srImg.cols;

	handMK.setZero(hight, width);

	Mat initMark,temp;
	initMark.create(hight,width,CV_8UC3);

	for (int y = 0; y < hight; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			Vec3b clr = srImg.at<cv::Vec3b>(y,x);

			bool isH = isHandPixel(clr);

			if (isH)
			{
				handMK(y,x) = 1;
				initMark.at<cv::Vec3b>(y,x) = clr;
			}
			else
			{
				initMark.at<cv::Vec3b>(y,x) = bgMask;
			}

		}
	}

	cvtColor(initMark,temp,CV_RGB2BGR);
	imshow("Hands-Markers",temp);

	//opening
	Mat gImg, gTemp;
	cvtColor(initMark,gImg,CV_RGB2GRAY);

	Mat erodeStruct = getStructuringElement(MORPH_RECT,Size(15,15));

	morphologyEx(gImg, gTemp,CV_MOP_CLOSE,erodeStruct);

	imshow("opening",gTemp);

}

void StopMotionUI::markersGenDiff(const Mat& srImg, const Mat& tgImg/*, MatrixXXi& diffMK*/)
{
	IndexType hight = srImg.rows;

	IndexType width = srImg.cols;

	m_diffLabels.setZero(hight, width);
	//diffMK.setZero(hight, width);

	Mat initMark,temp;
	initMark.create(hight,width,CV_8UC3);

	for (int y = 0; y < hight; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			Vec3b clrSr = srImg.at<cv::Vec3b>(y,x);
			Vec3b clrTg = tgImg.at<cv::Vec3b>(y,x);


			bool isDifferent = m_sMotion->isDifferent(clrSr,clrTg);

			bool isHand = isHandPixel(clrSr);

			//bool isHandN = isHandPixelNormal(clrSr);

			if (isDifferent && !isHand)
			{
				//diffMK(y,x) = 1;
				m_diffLabels(y,x) = GC_FGD;
				initMark.at<cv::Vec3b>(y,x) = fgMask;

			}else
			{
				//diffMK(y,x) = 0;
				m_diffLabels(y,x) = GC_BGD;
				initMark.at<cv::Vec3b>(y,x) = bgMask;
			}

// 			bool isHSr = isHandPixel(clrSr);
// 			bool isHTg = isHandPixel(clrTg);

// 			if (isHSr )
// 			{
// 				handMK(y,x) = 1;
// 			}
// 
// 			if (isHTg)
// 			{
//                handMK(y,x) = 2;
// 			}
		}
	}

	cvtColor(initMark,temp,CV_RGB2BGR);
	imshow("Init-Markers",temp);
}

bool StopMotionUI::isHandPixel(const Vec3b& oriColor)
{
	bool cdt_1,cdt_2,cdt_3;

	cdt_1 = false;

	cdt_2 = false;

	cdt_3 = false;

	if ( oriColor[0] > 95 && oriColor[1] > 40 && oriColor[2] > 20)
	{
		cdt_1 = true;
	}

	
	uchar maxRGB = max(oriColor[0], max(oriColor[1], oriColor[2]) );
	uchar minRGB = min(oriColor[0], min(oriColor[1], oriColor[2]) );

	if ( (maxRGB - minRGB) > 15)
	{
		cdt_2 = true;
	}

	if ( (oriColor[0] - oriColor[1]) > 15 && oriColor[0] > oriColor[1] && oriColor[0] > oriColor[2] )
	{
		cdt_3 = true;
	}

	if ( cdt_3 && cdt_2 && cdt_1)
	{
		return true;
	}else
	{
		return false;
	}
}

bool StopMotionUI::isHandPixelNormal(const Vec3b& oriColor)
{
// 	ScalarType thresA = 1.185;
// 	ScalarType thresB = 0.107;
// 	ScalarType thresC = 0.112;


	ScalarType thresA = 0.839;
	ScalarType thresB = 0.054;
	ScalarType thresC = 0.067;
	ScalarType thresD = 0.098;
	ScalarType thresE = 1.048;
	ScalarType thresF = 0.108;

	ScalarType sumAll = oriColor[0] + oriColor[1] + oriColor[2];

	if (oriColor[1]<= 0.1 || sumAll < 0.1)
	{
		return false;
	}

    ScalarType nColor[3];

	nColor[0] = oriColor[0]/sumAll;
	nColor[1] = oriColor[1]/sumAll;
	nColor[2] = oriColor[2]/sumAll;

	ScalarType nSumAll = nColor[0] + nColor[1] + nColor[2];

	bool cdt_1,cdt_2,cdt_3;

	cdt_1 = false;

	cdt_2 = false;

	cdt_3 = false;

	if (nColor[0]/nColor[1] > thresA)
	{
		cdt_1 = true;
	}


	ScalarType val2 = (nColor[0] * nColor[2] ) / (nSumAll * nSumAll);
	
	ScalarType val3 = (nColor[0] * nColor[1] ) / (nSumAll * nSumAll);

	if (val2 > thresB)
	{
		cdt_2 = true;
	}

	if (val3 > thresC)
	{
		cdt_3 = true;
	}

	if (cdt_3 && cdt_2 && cdt_1)
	{
		return true;
	}else
	{
		return false;
	}
}


bool StopMotionUI::isHandPixelNormal2(const Vec3b& oriColor)
{
	ScalarType thresA = 0.839;
	ScalarType thresB = 0.054;
	ScalarType thresC = 0.067;
	ScalarType thresD = 0.098;
	ScalarType thresE = 1.048;
	ScalarType thresF = 0.108;

	ScalarType sumAll = oriColor[0] + oriColor[1] + oriColor[2];

	if (oriColor[1]<= 0.1 || sumAll < 0.1 || oriColor[1] <= 0.1)
	{
		return false;
	}

	ScalarType nColor[3];

	nColor[0] = oriColor[0]/sumAll;
	nColor[1] = oriColor[1]/sumAll;
	nColor[2] = oriColor[2]/sumAll;

	ScalarType nSumAll = nColor[0] + nColor[1] + nColor[2];

	bool cdt_1,cdt_2,cdt_3,cdt_4,cdt_5,cdt_6;

	cdt_1 = false;

	cdt_2 = false;

	cdt_3 = false;

	cdt_4 = false;

	cdt_5 = false;

	cdt_6 = false;

	if (nColor[1]/nColor[0] <= thresA)
	{
		cdt_1 = true;
	}


	ScalarType val2 = (nColor[1] - nColor[2] ) / (nSumAll);

	ScalarType val3 = (nColor[2] * nColor[1] ) / (nSumAll * nSumAll);

	if (val2 <= thresB)
	{
		cdt_2 = true;
	}

	if (val3 > thresC)
	{
		cdt_3 = true;
	}

	if (val3 <= thresD)
	{
		cdt_4 = true;
	}

	if (nColor[2]/nColor[1] <= thresE)
	{
		cdt_5 = true;
	}

	if (nColor[1]/(3* nSumAll) <= thresF)
	{
		cdt_6 = true;
	}

	if (cdt_3 && cdt_2 && cdt_1 && cdt_4 && cdt_5 && cdt_6)
	{
		return true;

	}else
	{
		return false;
	}
}

void StopMotionUI::showBackGround()
{
	SingleFrame& bg = SingleFrameSet::get_instance().getBackGround();

	Mat* body_ = bg.body;

	Mat temp;

	cvtColor(*body_, temp,CV_RGB2BGR);

	imshow("BackGround", temp);  
}

void StopMotionUI::showCoorRGB(const QPoint& pos, const QRgb& rgb_)
{

	QString idx_str = QString("Current Position X = [%1], Y = [%2]").arg(pos.x()).arg(pos.y() );
    ui.coorMouse->setText(idx_str);

	QString rgb_str = QString("Current RGB R = [%1], G = [%2], B = [%3]").arg(qRed(rgb_) ).arg(qGreen(rgb_) ).arg(qBlue(rgb_) );
	ui.rgbMouse->setText(rgb_str);
}

void StopMotionUI::obtainCoor(const QPoint& pos)
{

	m_delPCoor.setX(pos.x());
	m_delPCoor.setY(pos.y());

	findNearestLine();
}

void StopMotionUI::obtainCoorLeft(const QPoint& pos)
{
	m_selectPointLeft.setX(pos.x());
	m_selectPointLeft.setY(pos.y());

	m_gloMatPsLeft.push_back(m_selectPointLeft);

	//draw the image again

	showOriROIs();
}

void StopMotionUI::obtainCoorRight(const QPoint& pos)
{
	m_selectPointRight.setX(pos.x());
	m_selectPointRight.setY(pos.y());

	m_gloMatPsRight.push_back(m_selectPointRight);

	//draw image again

	showOriROIs();
}

void StopMotionUI::delCurLine()
{

	if(m_delLineIdx >= 0 && m_delLineIdx < m_rMPixels.size())
	{
	    vector<CvPoint2D32f>::iterator lit = m_lMPixels.begin() + m_delLineIdx;
		m_lMPixels.erase(lit);

		vector<CvPoint2D32f>::iterator rit = m_rMPixels.begin() + m_delLineIdx;
		m_rMPixels.erase(rit);

		//update image of the window

		updateCurrentImage(m_seamPts, m_gcLeftROI,m_lMPixels,m_rMPixels);
	}else
	{
	    Loggger<<"The matching size is none.\n";
		return;
	}


}

void StopMotionUI::findNearestLine()
{
	if (m_lMPixels.size() <= 0)
	{
		return;
	}

	assert(m_delPCoor.x() > 0 && m_delPCoor.y() > 0 
		&& (m_lMPixels.size() == m_rMPixels.size()));

	vector<ScalarType> p2LineDis;
	p2LineDis.clear();

	for (IndexType i = 0; i < m_lMPixels.size(); ++ i)
	{	
		 ScalarType dis = p2LineSegDis(m_delPCoor, m_lMPixels[i],m_rMPixels[i]);
		 p2LineDis.push_back(dis);
	}

	IndexType idx = min_element(p2LineDis.begin(),p2LineDis.end()) - p2LineDis.begin();

	m_delLineIdx = idx;

	if (idx >= 0)
	{
		QPoint sP,eP;

		sP.setX(m_rMPixels[idx].x + m_sltRect[0].tl().x);
		sP.setY(m_rMPixels[idx].y + m_sltRect[0].tl().y);

		eP.setX(m_lMPixels[idx].x + m_sltRect[0].tl().x);
		eP.setY(m_lMPixels[idx].y + m_sltRect[0].tl().y);

	    if(m_isRightMask)
		{
	       ui.outputVideo->setDelLine(sP,eP);
		}else
		{
		   ui.inputVideo->setDelLine(sP,eP);
		}

	}

}

ScalarType StopMotionUI::p2LineSegDis(QPoint& oP, CvPoint2D32f& lsp, CvPoint2D32f& lep)
{
	//oP from global frame
  // lsp and lep are from ROI

	Eigen::Vector2f p,q1,q2;

	p[0] = oP.x(); p[1] = oP.y();

 	q1[0] = lsp.x; q1[1] = lsp.y;//start
 
 	q2[0] = lep.x; q2[1] = lep.y;//end

// 	q1[0] = lsp.x + m_sltRect[0].tl().x; q1[1] = lsp.y + m_sltRect[0].tl().y;//start
// 
// 	q2[0] = lep.x + m_sltRect[0].tl().x; q2[1] = lep.y + m_sltRect[0].tl().y;//end

     Eigen::Vector2f vL,vW;
	 vL = q2 - q1;

	 vW = p - q1;

	 ScalarType c1 = vL.dot(vW);

	 if (c1 <= 0.)
	 {
		 return  vW.norm();
	 }

	 ScalarType c2 = vL.dot(vL);

	 if (c2 <= c1)
	 {
		 return (p-q2).norm();
	 }

	 assert(c2 > 1e-3 || c2 <-1e-3);

	 ScalarType b = c1/c2;

	 Eigen::Vector2f pb = q1 + b* vL;

	 return (p-pb).norm();
}

void StopMotionUI::getMatchingPs(vector<QPoint>& mPs)
{
	if (mPs.size()%2)
	{
		Loggger<<"The selection of matching is not correct!.\n";
		return;
	}


	for (IndexType i = 0; i < mPs.size(); i += 2)
	{
         QPoint sP = mPs[i];
		 QPoint tP = mPs[i + 1];

		 //should find the closest point around the seam, m_seamPts.
		 Point scvp,tcvp;

		 findClosestPointOnSeam(sP,scvp);
		 findClosestPointOnSeam(tP,tcvp);

		 ScalarType dis= sqrt( (scvp.x-tcvp.x)* (scvp.x-tcvp.x) + 
			                   (scvp.y-tcvp.y)*(scvp.y-tcvp.y) );
		 if ( dis < 3.)
		 {
			 QMessageBox::information(this, "Information", "It's too short!",QMessageBox::Ok);
			 continue;
		 }

		 m_lMPixels.push_back(scvp);
		 m_rMPixels.push_back(tcvp);
	}

	//show the current pairs
	updateCurrentImage(m_seamPts, m_gcLeftROI,m_lMPixels,m_rMPixels);

}

void StopMotionUI::showLabels(const QPoint& pos, const QRgb& rgb_)
{
// 	SingleFrameSet& set_ = SingleFrameSet::get_instance();
// 
// 	SingleFrame& curImage = set_[0];
// 
// 	IndexType iHeight = (*set_[0].body).rows;
// 	IndexType iWidth = (*set_[0].body).cols;
// 	
// 	if (pos.x()> 0 && pos.x() < iWidth && pos.y() > 0 && pos.y() < iHeight)
// 	{

	    //IndexType initLabel, finalLabel;

// 		if (m_initLabels.cols() > 0 && m_initLabels.rows() > 0)
// 		{
// 			initLabel = m_initLabels(pos.y(), pos.x() );
// 		}
// 		else
// 		{
// 			initLabel = -1;
// 		}
// 
// 
// 		if (m_finalLabels.cols() > 0 && m_finalLabels.rows() > 0)
// 		{
// 			finalLabel = m_finalLabels(pos.y(), pos.x() );
// 
// 		}else
// 		{
//             finalLabel = -1 ;
// 		}

// 		if (m_markLabels.cols() > 0 && m_markLabels.rows() > 0)
// 		{
// 			initLabel = m_markLabels(pos.y(), pos.x() );
// 		}
// 		else
// 		{
// 			initLabel = -1;
// 		}
// 
// 		finalLabel = -1;
// 
// 		QString idx_str = QString("Current Labels init = [%1], final = [%2]").arg(initLabel).arg(finalLabel);
// 		ui.labels->setText(idx_str);
// 	}
// 	else
// 	{
// 		QString idx_str = QString("Current Labels init = [%1], final = [%2]").arg(-1).arg(-1);
// 		ui.labels->setText(idx_str);
// 	}


}

void StopMotionUI::updateGraphweight(int weight)
{
	if ( weight <=10 && weight > 0)
	{
		m_gWeight = 0.1 * weight;
	}

	if ( weight > 10 && weight <=100)
	{
		m_gWeight = (IndexType)(1.1 * weight - 10);
	}

	//m_gWeight =  weight /** (0.01)*/;

	ui.weightV->setText(QString::number(m_gWeight));
}

void StopMotionUI::updateTresDifference(int resChange)
{
	m_thresDiff = resChange;
	ui.showDifference->setText(QString::number(m_thresDiff));
}

void StopMotionUI::testGraphCut(Mat& inputIMg)
{
	MatrixXXi initLabels;

	initTwoRegion(inputIMg,m_initLabels);

	m_sMotion->setInteractiveStatue(
    m_sMotion->initImageLabelWithInteractive(m_initLabels) );

	MatrixXXi outSegmentation, finalBGFG;

	m_sMotion->trainGMM(inputIMg,m_initLabels);

	//m_sMotion->minCut(inputIMg,m_initLabels,outSegmentation);

	m_sMotion->minCut(inputIMg,m_initLabels,m_finalLabels);

	IndexType iterN = 0;

	while (iterN -- > 0)
	{
		m_sMotion->updateGMM(inputIMg,outSegmentation);

		m_sMotion->minCut(inputIMg,outSegmentation,m_finalLabels);
	}

}

void StopMotionUI::initTwoRegion(Mat& inputImg, MatrixXXi& handMark)
{
	IndexType hight = inputImg.rows;
	IndexType width = inputImg.cols;

	handMark.resize(hight,width);

	handMark.setZero();

	IndexType oStarX = m_sltRect[1].x;
	IndexType oStarY = m_sltRect[1].y;
	IndexType oHight = m_sltRect[1].height;
	IndexType oWidth = m_sltRect[1].width;

	for (IndexType hig = 0; hig < oHight; ++ hig)
	{
		for (IndexType wid = 0; wid < oWidth; ++ wid)
		{
			handMark(hig + oStarY,wid + oStarX) = 2;//unknow
		}
	}


	IndexType iStarX = m_sltRect[0].x;
	IndexType iStarY = m_sltRect[0].y;
	IndexType iHight = m_sltRect[0].height;
	IndexType iWidth = m_sltRect[0].width;


	for (IndexType hig = 0; hig < iHight; ++ hig)
	{
		for (IndexType wid = 0; wid < iWidth; ++ wid)
		{
			handMark(hig + iStarY,wid + iStarX) = 1;//unknow
		}
	}
}

void StopMotionUI::assignInitialLabels()
{
	// combine the labels from the m_diff and m_mark

	//'mark' label is a global marker;
	//'diff' label is a ROI marker

	//assert(m_markLabels.rows() > 0 && m_markLabels.cols() > 0);

	IndexType height = m_diffLabels.rows();

	IndexType width = m_diffLabels.cols();

	assert(height > 0  && width > 0);

	m_initLabels.setZero(height,width);

	m_initLabels = m_diffLabels;

	if (m_markLabels.rows() > 0 && m_markLabels.cols() > 0)
	{
		IndexType strRectX = m_sltRect[0].x;
		IndexType strRectY = m_sltRect[0].y;

		for (IndexType idH = 0; idH < height; ++ idH)
		{
			for (IndexType idW = 0; idW < width; ++ idW)
			{
				IndexType curY = idH + strRectY;
				IndexType curX = idW + strRectX;

				IndexType markLabel = m_markLabels(curY, curX);

				if (markLabel == GC_BGD || markLabel == GC_FGD)
				{
					m_initLabels(idH,idW) = markLabel;
				}

			}
		}
	}

}

void StopMotionUI::translucentImages(Mat& srIMg, Mat& tgImg, 
									 ScalarType alpha, Mat& resImg)
{
	assert(alpha >= 0. && alpha <= 1.);

	ScalarType beta = 1. - alpha;

	addWeighted(srIMg,alpha,tgImg,beta,0.0, resImg);

// 	Mat tImg;
// 
// 	tImg.release();
// 
// 	cvtColor(resImg,tImg,CV_RGB2BGR);
// 
// 	imshow("Vis_MinCut",tImg);

}

void StopMotionUI::checkStillPixels(MatrixXXi& outPutMark, MatrixXXi& stillPixels)
{
   assert(outPutMark.rows() == stillPixels.rows() );
   assert(outPutMark.cols() == stillPixels.cols() );


   IndexType height = outPutMark.rows();
   IndexType width = outPutMark.cols();

   for (IndexType hId = 0; hId < height; ++ hId)
   {
	   for (IndexType wId = 0; wId < width; ++ wId)
	   {
		   IndexType stillV = stillPixels(hId,wId);
		   IndexType outV = outPutMark(hId,wId);
		   if (stillV && outV)
		   {
              outPutMark(hId,wId) = 0;
		   }	       
	   }
   }

}

void StopMotionUI::initInteracrionLabels()
{
	if (m_markLabels.cols() > 0 || m_markLabels.rows() > 0)
	{
		m_markLabels.resize(0,0);
	}

	if (!m_dfLeftROI.empty())
	{
	   m_dfLeftROI.release();
	}

}

bool StopMotionUI::checkTrainData(MatrixXXi& initLabels)
{
	IndexType height = initLabels.rows();
	IndexType width = initLabels.cols();

	if ( height <= 0 || width <= 0)
	{
		return false;
	}

	IndexType fgSize = 0;
	IndexType bgSize = 0;

	for (IndexType hId = 0; hId < height; ++ hId)
	{
		for (IndexType wId = 0; wId < width; ++ wId)
		{
			if ( 1 ==initLabels(hId,wId))
			{
				++ fgSize; 

			}else if ( 0== initLabels(hId,wId) )
			{
				++ bgSize;
			}
		}
	}

	if (fgSize > 0 && bgSize > 0)
	{
		return true;
	}else
	{
		return false;
	}
}

void StopMotionUI::undoStrokeDraw()
{
	//if (m_isDealLeftImg) // the strokes for left image only
    if (!m_isRightMask) // the strokes for left image only
	{
		if( m_leftmaskImgStack.size() > 1)
		{
			m_leftmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_lmaskImage = m_leftmaskImgStack.top();

			//assign the label information

			IndexType width = m_lmaskImage.width();
			IndexType height = m_lmaskImage.height();

			m_markLeftLabels.resize(height,width);

			QRgb bgColor;  //();
			QRgb fgColor;

			bgColor = qRgb(bgMask[0], bgMask[1],bgMask[2]);
			fgColor = qRgb(fgMask[0], fgMask[1],fgMask[2]);

			for (IndexType hIdx = 0; hIdx < height; ++ hIdx)
			{
				for (IndexType wIdx = 0; wIdx < width; ++ wIdx)
				{
					QRgb curColor = m_lmaskImage.pixel(wIdx,hIdx);

					if (curColor == bgColor)
					{
						m_markLeftLabels(hIdx,wIdx) = GC_BGD;
					}
					else if (curColor == fgColor)
					{
						m_markLeftLabels(hIdx,wIdx) = GC_FGD;

					}else
					{
						m_markLeftLabels(hIdx,wIdx) = GC_PR_BGD;
					}
				}
			}

		}else
		{
			if (m_leftmaskImgStack.empty())
			{
				Loggger<<"Strack is empty!.\n";
				return;
			}

			m_leftmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_lmaskImage = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
				QImage::Format_ARGB32);
			//do not assign 
		}

		ui.inputVideo->setImage(&m_curShowLeftImg);	

		ui.inputVideo->setMaskImage(m_lmaskImage);

		ui.inputVideo->updateDisplayImage();

	}else // for right image
	{
		//the highest element was already used.
		if( m_rightmaskImgStack.size() > 1)
		{
		   m_rightmaskImgStack.pop();

		   QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
		   ui.strokeLevel->setText(strSize_str);

		   m_rmaskImage = m_rightmaskImgStack.top();

		   //assign the label information

		   IndexType width = m_rmaskImage.width();
		   IndexType height = m_rmaskImage.height();

		   m_markLabels.resize(height,width);

		   QRgb bgColor;  //();
		   QRgb fgColor;

		   bgColor = qRgb(bgMask[0], bgMask[1],bgMask[2]);
		   fgColor = qRgb(fgMask[0], fgMask[1],fgMask[2]);

		   for (IndexType hIdx = 0; hIdx < height; ++ hIdx)
		   {
			   for (IndexType wIdx = 0; wIdx < width; ++ wIdx)
			   {
				   QRgb curColor = m_rmaskImage.pixel(wIdx,hIdx);

				   if (curColor == bgColor)
				   {
					   m_markLabels(hIdx,wIdx) = GC_BGD;
				   }
				   else if (curColor == fgColor)
				   {
					   m_markLabels(hIdx,wIdx) = GC_FGD;

				   }else
				   {
					   m_markLabels(hIdx,wIdx) = GC_PR_BGD;
				   }
			   }
		   }

		}else
		{
			if (m_rightmaskImgStack.empty())
			{
				Loggger<<"Strack is empty!.\n";
				return;
			}

			m_rightmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
				QImage::Format_ARGB32);
			//do not assign 
		}

		ui.outputVideo->setImage(&m_curShowImg);	
	
		ui.outputVideo->setMaskImage(m_rmaskImage);

		ui.outputVideo->updateDisplayImage();
	}// end for if

}

void StopMotionUI::okDeformation()
{
	m_isOKDeformation = true;
	m_isDealLeftImg = false;

	MatrixXXi handMarkers, stillPixel;
	//handMarkerGenerate(roiR,handMarkers); // RGB method
	m_sMotion->setOkDeformation(m_isOKDeformation);

	//m_sMotion->handMarkerRGBYUV(m_rightROI,handMarkers); // combine RGB and YUV

	m_sMotion->itaHandMarkers(m_rightROI,handMarkers);// do nothing

	//m_sMotion->getinitTrainData(m_rightROI,handMarkers, stillPixel); //with unknown labels

	m_diffLabels = handMarkers;

	assignInitialLabels();

	MatrixXXi roiMarkHand;

	if (m_markLabels.cols() > 0)
	{
		roiMarkHand = m_markLabels.block(m_sltRect[0].y,m_sltRect[0].x,m_sltRect[0].height,m_sltRect[0].width);

		m_sMotion->setInteractiveStatue(
			m_sMotion->initImageLabelWithInteractive(roiMarkHand));
	}

	MatrixXXi outPutMark;

	Mat outPutImg;

	if (!checkTrainData(m_initLabels))
	{
		Loggger<<" The train data are wrong!.\n";

		return;
	}

	m_sMotion->handRemoval(m_dfLeftROI,m_rightROI,m_initLabels,outPutMark);

	//checkStillPixels(outPutMark,stillPixel);

	m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,outPutMark,outPutImg);// default R->L.

	// using background pixel to replace the bgMask regions.

// 	Mat* bg = SingleFrameSet::get_instance().getBackGround().body;
// 
// 	Mat* curF = SingleFrameSet::get_instance()[m_showPairIdx[1] ].body;
// 
// 	Mat bgROI;
// 	(*bg)(m_sltRect[0]).copyTo(bgROI);
// 
// 	m_sMotion->imageGenerWithBackg(m_dfLeftROI,m_rightROI,outPutMark,bgROI,outPutImg);// with background pixels
	
	m_dfLeftROI.release();

	Loggger<<"Combine process is OK.\n";

}

// using the interactive digital photomontage method to fuse two rois

void StopMotionUI::fuseTwoROIs()
{

	if (m_isLeftBg || m_isRightBg)
	{
		completionWithBG();
		
		return;
	}

	// input:  m_dfLeftROI, m_rightROI, few strokes;
	// output: 

	IndexType height = m_dfLeftROI.rows;
	IndexType width = m_dfLeftROI.cols;


	if (m_sltRect.size() <=0 )
	{
	    Loggger<<"please select the ROIs again!'n";
		return;
	}
	if (height <= 5 || width <= 5)
	{
		Loggger<<"Using the original ROI.\n";

		m_dfLeftROI = m_leftROI;// the left image equals to a default image.
	}


	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	m_isRightMask = isMaskFromRight();

	if (m_isRightMask)
	{
		imgData.push_back(m_rightROI);
		imgData.push_back(m_dfLeftROI);
     	getMaskFromImage(m_rmaskImage,maskFromStk);

	}else
	{
		imgData.push_back(m_dfLeftROI);
		imgData.push_back(m_rightROI);
	    getMaskFromImage(m_lmaskImage,maskFromStk);
	}

	MatrixXXi initLabels;
	initLabels.resize(height,width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height,imgData,maskFromStk,initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	_gcutLabels.clear();
	_gcutLabels.push_back(resSeg);//for the final p smooth.

	m_cutLabels = resSeg;

	//for local deformation
	//step1: using Canny to find the feature points along the boundary of the graph cut for two images respectively;
	//step2: matching the feature points by dynamic program
	//step3: the matching points as the control handle to deform the local region area.

	//directly using matlab codes

// 	Loggger<<"start to local piecewise homogeneous deform.\n";
// 
	Mat defImg,outPutImg;

	//bool isAutomatic = false;

	bool isAutomatic = true;

	if (isAutomatic)
	{

		if (m_isRightMask)
		{
		    m_sMotion->setInputImg(m_dfLeftROI,m_rightROI,resSeg);

			m_sMotion->localDeformation(m_dfLeftROI,m_rightROI,resSeg,defImg);

			if (!defImg.empty())
			{
				assert(defImg.size() == m_dfLeftROI.size());

				m_sMotion->imageGenerate(defImg,m_rightROI,resSeg,outPutImg);

			}else
			{
				m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,resSeg,outPutImg);
			}

			//for bg completion


		}else
		{
			 m_sMotion->setInputImg(m_rightROI,m_dfLeftROI,resSeg);

			m_sMotion->localDeformation(m_rightROI,m_dfLeftROI,resSeg,defImg);

			if (!defImg.empty())
			{
				assert(defImg.size() == m_dfLeftROI.size());

				m_sMotion->imageGenerate(defImg,m_leftROI,resSeg,outPutImg);

			}else
			{
				m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,resSeg,outPutImg);
			}

		}

		displayMiddleResults(outPutImg);

			//for bg completion
		Mat bg;

		(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

		vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;

		outPutImg.copyTo((bg)(roi[0])); 

	    bg.copyTo(m_gcutImg);//for display, global image

	}else//with the help of interaction
	{
		//interaction for matching
		vector<CvPoint2D32f> fsrMatchingPs,ftgMatchingPs;
		vector<Point2f> ptsList;

		//1-find the seam 	
		//m_sMotion->findSeamPosList(resSeg,ptsList);
		//m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,resSeg,outPutImg);	

	    MatrixXXi expLabels;
		m_sMotion->findSeamExpand(resSeg,ptsList,expLabels);

		//should return a new labels information!

		if (m_isRightMask)
		{
	       m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,expLabels,outPutImg);
		}else
		{
	       m_sMotion->imageGenerate(m_rightROI, m_dfLeftROI,expLabels,outPutImg);
		}

		m_seamPts = ptsList;
		m_totSeamPsLeft = ptsList.size();
		m_totSeamPsright = ptsList.size();

		ui.leftSeamIdx->setRange(0,m_totSeamPsLeft - 1);
		ui.rightSeamIdx->setRange(0,m_totSeamPsright - 1);

		//for visualization

		Mat bg;
		(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

		vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;

		outPutImg.copyTo(m_gcLeftROI);//for deformation,is a bug! can not do again!

		outPutImg.copyTo((bg)(roi[0])); 

		bg.copyTo(m_gcutImg);//for display, global image


		//draw the seam on bg
		Scalar color(255,0,0);
		Point2f tlp = m_sltRect[0].tl();

		for (IndexType i = 0; i < ptsList.size(); ++ i)
		{
			Point2f curP = ptsList[i];
			curP += tlp;
			circle(bg,curP,1,color);
		}

		if (m_isRightMask)
		{
		  updateRight(bg);

		}else
		{
		  updateLeft(bg);
		}


		//2-detect the edges cross the seam

		//active slider!

		//matching points or edges?
		//bool isMatchingPs = true;

		//selection is ok?

		//if using edge¡H
		// disable above setting and active the edges setting
		//vector<Point> lMatchEdges,rMatchEdges;

		//combine the matching information

// 		Mat dOutImg;
// 		m_sMotion->deformRoiWithMathcing(m_dfLeftROI,m_rightROI,ptsList,fsrMatchingPs,ftgMatchingPs,dOutImg);

	}


}

void StopMotionUI::newFuseTwoROIs()
{
	//for refine the results using background frame
	if (m_isLeftBg || m_isRightBg)
	{
		completionWithBG();

		return;

	}	//end for refine

	handRemoveByGCut();

	// end for graph cut processing.

	vector<Point2f> ptsList;
	MatrixXXi expLabels;

	if (m_cutLabels.rows() > 0)
	{
	  m_sMotion->findSeamExpand(m_cutLabels,ptsList,expLabels);

	  m_seamPts = ptsList;
	  m_totSeamPsLeft = ptsList.size();
	  m_totSeamPsright = ptsList.size();

	  ui.leftSeamIdx->setRange(0,m_totSeamPsLeft - 1);
	  ui.rightSeamIdx->setRange(0,m_totSeamPsright - 1);

	}else
	{
		return;
	}
	
	//find the initial matching information//
	vector<CvPoint2D32f> lmatchPs,rmatchPs;
	lmatchPs.clear();
	rmatchPs.clear();

	Mat outPutImg;

	if (m_isRightMask)//default
	{
       m_sMotion->setInputImg(m_dfLeftROI,m_rightROI,expLabels);
	   //find initial matching points
	   m_sMotion->findInitialMatchingAlongSeam(ptsList,lmatchPs,rmatchPs);

	   m_lMPixels = lmatchPs;
	   m_rMPixels = rmatchPs;

	   	// completion
	   m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,expLabels,outPutImg);

	}else
	{
	   m_sMotion->setInputImg(m_rightROI,m_dfLeftROI,expLabels);
	   m_sMotion->findInitialMatchingAlongSeam(ptsList,lmatchPs,rmatchPs);

	   m_lMPixels = lmatchPs;
	   m_rMPixels = rmatchPs;

	   // completion
	   m_sMotion->imageGenerate(m_rightROI, m_dfLeftROI,expLabels,outPutImg);
	}

	//visualize the matching
	//update current image after graph cut
	updateCurrentImage(ptsList, outPutImg,lmatchPs,rmatchPs);

}

void StopMotionUI::updateCurrentImage(vector<Point2f>& ptsList, Mat& outPutImg,
									  vector<CvPoint2D32f>& lmatchPs,vector<CvPoint2D32f>& rmatchPs)
{
	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;

	outPutImg.copyTo(m_gcLeftROI);//for deformation,is a bug! can not do again!

	outPutImg.copyTo((bg)(roi[0])); 

	bg.copyTo(m_gcutImg);//for display, global image

	//draw the seam on bg
	Scalar color(255,0,0);
	Point2f tlp = m_sltRect[0].tl();

	for (IndexType i = 0; i < ptsList.size(); ++ i)
	{
		Point2f curP = ptsList[i];
		curP += tlp;
		circle(bg,curP,.1,color);
	}

	//draw matching lines
	RNG rng(12345);

	for (IndexType i = 0; i < lmatchPs.size(); ++ i)
	{
		Point2f curS = lmatchPs[i];
		Point2f curT = rmatchPs[i];

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		line(bg,(curS + tlp),(curT + tlp),color,3);

		//circle(bg,(curS + oriPs),2,color);//start points
	}

	if (m_isRightMask)
	{
		updateRight(bg);

	}else
	{
		updateLeft(bg);
	}

}

//
void StopMotionUI::handRemoveByGCut()
{

	//start to graph cut processing
	// input:  m_dfLeftROI, m_rightROI, few strokes;
	// output: labels (0 is the original pixel, 1 represent a pixel from another frame)

	IndexType height = m_dfLeftROI.rows;
	IndexType width = m_dfLeftROI.cols;

	if (m_sltRect.size() <=0 )
	{
		Loggger<<"please select the ROIs again!'n";
		return;
	}
	if (height <= 5 || width <= 5)
	{
		Loggger<<"Using the original ROI.\n";

		m_dfLeftROI = m_leftROI;// the left image equals to a default image.
	}


	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	m_isRightMask = isMaskFromRight();

	if (m_isRightMask)
	{
		imgData.push_back(m_rightROI);
		imgData.push_back(m_dfLeftROI);
		getMaskFromImage(m_rmaskImage,maskFromStk);

	}else
	{
		imgData.push_back(m_dfLeftROI);
		imgData.push_back(m_rightROI);
		getMaskFromImage(m_lmaskImage,maskFromStk);
	}

	MatrixXXi initLabels;
	initLabels.resize(height,width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height,imgData,maskFromStk,initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	_gcutLabels.clear();
	_gcutLabels.push_back(resSeg);//for the final p smooth.

	m_cutLabels = resSeg;

}
//
void StopMotionUI::updateLeftWindowImage(Mat& warpedROI)
{
	Mat* curF = SingleFrameSet::get_instance()[m_showPairIdx[0]].body;

	vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[0]].sltRect;

   //m_initCurFrame = *curF;

   curF->copyTo(m_initCurFrame);

   warpedROI.copyTo((*curF)(roi[0]));

   mat2Qimage(*curF,m_curShowLeftImg);

   ui.inputVideo->setImage(&m_curShowLeftImg);	

   if (m_lmaskImage.width() > 0)
   {
	   ui.inputVideo->setMaskImage(m_lmaskImage);
   }

   ui.inputVideo->updateDisplayImage();


}

void StopMotionUI::pSmooth()
{

	possionImages(m_dfLeftROI,m_rightROI);//(left,right)

	//(bg,left,right)
	Mat bgROI;
	Mat bg;

   (*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

   bg(m_sltRect[0]).copyTo(bgROI);

   finPossionImages(bgROI, m_gMatchingImg,m_rightROI);

}

void StopMotionUI::possionImages(Mat& leftImg, Mat& rightImg)
{

	// initialization

	IndexType width = leftImg.cols;
	IndexType heigh = leftImg.rows;

	ImageAbs lImg(width,heigh,leftImg);
	ImageAbs rImg(width,heigh,rightImg);

	_inputImage.push_back(&rImg);
	_inputImage.push_back(&lImg);

	// add a background image 

	_blendedImage = ImageAbs(width,heigh);
	_blendTemp = new ImageAbs(width,heigh);

	_blender = new Adrien::GlobalBlender(width,heigh,&_inputImage, _blendedImage.mutableData(), 1000., stdout);

 
	possionCompute();

}


void StopMotionUI::finPossionImages(Mat& bgImg, Mat& leftImg, Mat& rightImg)
{
	// initialization

	IndexType width = leftImg.cols;
	IndexType heigh = leftImg.rows;

	ImageAbs lImg(width,heigh,leftImg);
	ImageAbs rImg(width,heigh,rightImg);
	ImageAbs bImg(width,heigh,bgImg);

	_inputImage.push_back(&rImg); //0
	_inputImage.push_back(&lImg); //1
	_inputImage.push_back(&bImg); //2

	// add a background image 

	_blendedImage = ImageAbs(width,heigh);
	_blendTemp = new ImageAbs(width,heigh);

	_blender = new Adrien::GlobalBlender(width,heigh,&_inputImage, _blendedImage.mutableData(), 1000., stdout);


	possionCompute();

}


void StopMotionUI::possionCompute()
{
	if (!_blender)
	{
		Loggger<<"Blender is empty! \n";
		return;
	}

	//changeLabelsType(); //only for two images

	changeLabelsTypeFinal(); // for multi images


	double et;

	et = 100;

	if (_blender && _labels)
	{
		 _blender->pinPixel (50, 50);
		 _blender->takeComputeData(false, _labels, et);
		 _blender->compute();

		 unsigned char* res = _blender->getResult();

		 saveSmoothImage(res);

	}else
	{
		Loggger<<"Poisson smooth is error!.\n";
	}

}

void StopMotionUI::stopPSmooth()
{
	if (_blender)
	{
		_blender->myStop();
	}
}


void StopMotionUI::changeLabelsType()
{
	if(m_dialCut)
	{
       MatrixXXi& resSeg = m_dialCut->getLabels();

	   IndexType nW = resSeg.cols();
	   IndexType nH = resSeg.rows();

	   if (!_labels)
	   {
		   delete _labels;
		   _labels = NULL;
	   }

	   _labels = new ushort[nW * nH];

	   memset(_labels,0, nW * nH * sizeof(ushort) );

	   for (IndexType i = 0; i < nH; ++ i)
	   {
		   for (IndexType j = 0; j < nW; ++ j)
		   {
			   _labels[ i * nW + j] = resSeg(i,j);
		   }
	   }

	}else
	{
		Loggger<<"Composite images are error!\n";
		return;
	}

}


void StopMotionUI::changeLabelsTypeFinal()
{
	if(m_dialCut)
	{
		MatrixXXi resSeg; // = m_dialCut->getLabels();

		assert(_gcutLabels.size() > 0);

		resSeg = _gcutLabels[0];

		IndexType nW = resSeg.cols();

		IndexType nH = resSeg.rows();

		for (IndexType i = 1; i < _gcutLabels.size(); ++ i)
		{
			MatrixXXi curLab = _gcutLabels[i];

			for (IndexType i = 0; i < nH; ++ i)
			{
				for (IndexType j = 0; j < nW; ++ j)
				{
					if (curLab(i,j))
					{
                        resSeg(i,j) = i + 1; // right - left - background;
					}
				}
			}
		}


		if (!_labels)
		{
			delete _labels;
			_labels = NULL;
		}

		_labels = new ushort[nW * nH];

		memset(_labels,0, nW * nH * sizeof(ushort) );

		for (IndexType i = 0; i < nH; ++ i)
		{
			for (IndexType j = 0; j < nW; ++ j)
			{
				_labels[ i * nW + j] = resSeg(i,j);
			}
		}

	}else
	{
		Loggger<<"Composite images are error!\n";
		return;
	}

}


void StopMotionUI::saveSmoothImage(unsigned char* res)
{

	Mat smImage;

	IndexType nW = m_leftROI.cols;
	IndexType nH = m_leftROI.rows;

	smImage.create(nH,nW,m_leftROI.type());

	for (IndexType i = 0; i < nH; ++ i)
	{
		for (IndexType j = 0; j < nW; ++ j)
		{
			IndexType curIdx =  i * nW + j;

			IndexType curRes = 3 * curIdx;

			Vec3b _rgb;

			_rgb[0] = res[curRes];
			_rgb[1] = res[curRes + 1];
			_rgb[2] = res[curRes + 2];

			smImage.at<Vec3b>(i,j) = _rgb;
		}
	}

	Mat resTemp;
	cvtColor(smImage,resTemp,CV_RGB2BGR);
	imshow("Smooth image",resTemp);

	char comName[1024];

	sprintf(comName,".\\resCombine\\dcut-pSmooth-%.2d.jpg",m_curPairIdx);

	imwrite(comName, resTemp);

}

void StopMotionUI::autoSelectionPair()
{
...
	//visualization of a short range of optical flow firstly
	Loggger<<"Start to auto-selection!.\n";

	m_autoSelect = new AutoSelect(m_curFrameIdx, m_totFrames);
	m_autoSelect->setBand(m_band);

	//m_autoSelect->visOptialFlow(m_curFrameIdx,m_curFrameIdx + 3);
	//m_autoSelect->visOptialFlow(440,440 + 100);

	vector<IndexType> labels;

	//m_autoSelect->labelingOneProcess(m_curFrameIdx,m_curFrameIdx + 30,labels);
	//m_autoSelect->labelingOneProcess(248,248 + 30,labels);
	//m_autoSelect->labelingOneProcess(500,500 + 50,labels); // for ransac

	// see the area of foreground pixels

	if (m_bgId <=0 || m_srtId <= 0 || m_lenSeq <= 0)
	{
		Loggger<<"The input for auto-seletion is error!.\n";
		return;
	}
	//m_autoSelect->areaForeground(80,200,1000);// backg-start-length

	m_autoSelect->areaForegWithRect(m_bgId,m_srtId,m_lenSeq,m_sltRect[0]);//cub-background -80
	//m_autoSelect->areaForegWithRect(80,250,200,m_sltRect[0]);
	//test optical flow

	//m_autoSelect->areaFlowVis(80,180,200);// flow bigger than threshold
	//phone-30,180,200

	// with rectangle
	//m_autoSelect->areaFlowVisRect(80,250,100,m_sltRect[0]);

	Loggger<<"End for auto-selection process.\n";
};

void StopMotionUI::drawEditImg(Mat& oriImg,Point& minPs, int height,int width,MatrixXX& quadCoor,MatrixXX& textCoor)
{

// 	if (m_isRightMask)
// 	{
	    ui.outputVideo->updateDeformImage(oriImg,minPs,height,width,quadCoor,textCoor);
// 	}else
// 	{
// 	    ui.outputVideo->updateDeformImage(oriImg,minPs,height,width,quadCoor,textCoor);
// 	}

	m_canvas->getTextureCanvas()->drawDeformedImg(oriImg,height, width,quadCoor,textCoor);


}

void StopMotionUI::updateRight(Mat& freImg)
{
	//QImage temp;

	mat2Qimage(freImg,m_curShowImg);

	ui.outputVideo->setImage(&m_curShowImg);

	ui.outputVideo->updateDisplayImage();

}

void StopMotionUI::updateLeft(Mat& gcutImg)
{
	mat2Qimage(gcutImg,m_curShowLeftImg);

	ui.inputVideo->setImage(&m_curShowLeftImg);

	ui.inputVideo->updateDisplayImage();
}

void StopMotionUI::feaIdxChangesLeft(int curId)
{
	if (curId > 0 && curId < m_totSeamPsLeft)
	{

		m_lfeaIdx = curId;
		ui.lfeaIdx->setText(QString::number(curId) );

		Point curCoor = m_seamPts[curId];

		curCoor += m_sltRect[0].tl();

	   Scalar color(255,0,0);

	   Mat temp;
	   m_gcutImg.copyTo(temp);

	   circle(temp,curCoor,5,color);

	   updateRight(temp);
	}
}

void StopMotionUI::feaIdxChangesRight(int curId)
{
	if (curId > 0 && curId <m_totSeamPsright)
	{
		m_rfeaIdx = curId;

		ui.rfeaIdx->setText(QString::number(curId) );
		Point curCoor = m_seamPts[curId];

		curCoor += m_sltRect[0].tl();

		Scalar color(255,0,0);

		Mat temp;

		m_gcutImg.copyTo(temp);

		circle(temp,curCoor,5,color);

		updateRight(temp);
	}
}

void StopMotionUI::addMatching()
{
	m_lMatchingPs.push_back(m_lfeaIdx);
	m_rMatchingPs.push_back(m_rfeaIdx);

	ui.leftSeamIdx->setEnabled(true);
	ui.rightSeamIdx->setEnabled(true);

	ui.lfeaselect->setChecked(false);

	ui.rfeaselect->setChecked(false);

}

void StopMotionUI::deformImgInteraction()
{
	//combine two matching information (automatic and interaction ways)

	//for the slide interaction method 
	if (!m_seamPts.empty() && m_lMatchingPs.size() > 0 && (m_lMatchingPs.size() == m_rMatchingPs.size()))
	{
		for (IndexType i = 0; i < m_lMatchingPs.size(); ++ i)
		{
			m_lMPixels.push_back(m_seamPts[m_lMatchingPs[i]] );
			m_rMPixels.push_back(m_seamPts[m_rMatchingPs[i]] );
		}
	}

	if (m_lMPixels.empty() || m_rMPixels.empty()|| m_sMotion == NULL)
	{
		Loggger              <<"please select the matching points.\n";
		return;
	}

	Mat interInf, stateImg;

	m_gcLeftROI.copyTo(interInf);

	if (m_isRightMask)
	{
		m_dfLeftROI.copyTo(stateImg);
	}else
	{
		m_rightROI.copyTo(stateImg);
	}

	vector<CvPoint2D32f> lmtemp,rmtemp;
	lmtemp = m_lMPixels;
	rmtemp = m_rMPixels;

	Scalar color(255,0,0);//red-source
	Scalar color2(255,255,255);//white-target

	for (IndexType i= 0; i < m_lMPixels.size(); ++ i)
	{
		Point2f tp = m_lMPixels[i];
		Point2f tp2 = m_rMPixels[i];

	    circle(interInf,tp,5,color);//source
		circle(interInf,tp2,5,color2);//target

		circle(stateImg,tp,5,color);//source
		circle(stateImg,tp2,5,color2);//target
	}

	Mat tinter,stimgtemp;
	cvtColor(interInf,tinter,CV_RGB2BGR);
	cvtColor(stateImg,stimgtemp,CV_RGB2BGR);

	char comName[1024];
	char corName[1024];

	sprintf(comName,".\\resCombine\\1029-defCorres.jpg");
	sprintf(corName,".\\resCombine\\1029-oriCorres.jpg");

	imwrite(comName,tinter);
	imwrite(corName,stimgtemp);



	Mat dOutImg;

	//bool isLines = true;
	bool isLines = false;

	m_sMotion->setInputImg(m_gcLeftROI,m_rightROI,m_cutLabels);

	if (isLines)
	{
		if (m_isRightMask)
		{
			m_sMotion->deformLinesPoints(m_dfLeftROI,m_rightROI,m_seamPts,
				m_lMPixels,m_rMPixels,dOutImg);
		}else
		{
			m_sMotion->deformLinesPoints(m_rightROI,m_dfLeftROI,m_seamPts,
				m_lMPixels,m_rMPixels,dOutImg);
		}

	}else
	{
		if (m_isRightMask)
		{
			m_sMotion->deformRoiWithMathcing(m_dfLeftROI,m_rightROI,m_seamPts,
				m_lMPixels,m_rMPixels,dOutImg);
		}else
		{
			m_sMotion->deformRoiWithMathcing(m_rightROI,m_dfLeftROI,m_seamPts,
				m_lMPixels,m_rMPixels,dOutImg);
		}
	}

	
	Mat dOutTemp;
	dOutImg.copyTo(dOutTemp);

	for (IndexType i= 0; i < m_lMPixels.size(); ++ i)
	{
		Point2f tp = lmtemp[i];
		Point2f tp2 = rmtemp[i];

		circle(dOutTemp,tp,5,color);//source
		circle(dOutTemp,tp2,5,color2);//target
	}

	displayMiddleResults(dOutTemp);

	//let the dOutImg as the leftROI, for smooth processing;
	dOutImg.copyTo(m_gMatchingImg);

	//
	Mat outPutImg;

 	if (m_isRightMask)
 	{
 		m_sMotion->imageGenerate(dOutImg,m_rightROI,m_cutLabels,outPutImg);
 	}else
 	{
 		m_sMotion->imageGenerate(dOutImg,m_dfLeftROI,m_cutLabels,outPutImg);
 	}
//
//
//	//do not directly to complete the image by the two images.using graph cut again!
// 	if (m_isRightMask)
// 	{
// 		gcAfterDeformation(dOutImg,m_rightROI,outPutImg,true);
// 
// 	}else
// 	{
// 		gcAfterDeformation(dOutImg,m_leftROI,outPutImg,false);
// 	}

	outPutImg.copyTo(m_gcutImg(m_sltRect[0]));


// 	m_lMatchingPs.clear();
// 	m_rMatchingPs.clear();
// 	m_lMPixels.clear();
// 	m_rMPixels.clear();


}

void StopMotionUI::gcAfterDeformation(Mat& leftImg, Mat& rightImg, Mat& outImg, bool isRightMask)
{
	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	imgData.push_back(m_rightROI);
	imgData.push_back(m_dfLeftROI);

	if (isRightMask)
	{
	   getMaskFromImage(m_rmaskImage,maskFromStk);
	}else
	{
	   getMaskFromImage(m_lmaskImage,maskFromStk);
	}

	IndexType width = leftImg.cols;
	IndexType height = leftImg.rows;

	MatrixXXi initLabels;
	initLabels.resize(height,width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height,imgData,maskFromStk,initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	m_sMotion->imageGenerate(leftImg,rightImg,resSeg,outImg);


}

void StopMotionUI::selectLeftFeatures()
{
	bool isLeftCheck = ui.lfeaselect->isChecked();

	if (isLeftCheck)
	{
		ui.leftSeamIdx->setEnabled(false);
	}

	bool isRightCheck = ui.rfeaselect->isChecked();

	if (isLeftCheck && isRightCheck)
	{
		addMatching();
	}
}


void StopMotionUI::selectRighFeatures()
{
	bool isLeftCheck = ui.lfeaselect->isChecked();

	bool isRightCheck = ui.rfeaselect->isChecked();

	if (isRightCheck)
	{
		ui.rightSeamIdx->setEnabled(false);
	}

	if (isLeftCheck && isRightCheck)
	{
		addMatching();
	}
}

void StopMotionUI::findClosestPointOnSeam(QPoint& curP, Point& cloPs)
{
	IndexType nSize = m_seamPts.size();
	assert(nSize > 0);

	vector<ScalarType> dis;
	vector<IndexType> pIdx;
	dis.clear();
	pIdx.clear();

	for (IndexType i = 0; i < nSize; ++ i)
	{
		ScalarType dx = curP.x() - (m_seamPts[i].x);
		ScalarType dy = curP.y() - (m_seamPts[i].y);

		ScalarType idis = sqrt(dx * dx + dy * dy);
		dis.push_back(idis);
		pIdx.push_back(i);
	}

	bubleSort(dis,pIdx,nSize);

	cloPs = m_seamPts[pIdx[0]];// select the closest point for mathcing 
	
}

void StopMotionUI::bubleSort(vector<ScalarType>& oriData,vector<IndexType>& labels,IndexType lSize)
{
	ScalarType temp;
	IndexType labelTemp = 0;
	bool flag=false;
	for (int i=0;i<lSize;i++)
	{
		flag=true;
		for (int j=0;j<lSize-i-1;j++)
		{
			if(oriData[j]>oriData[j+1])
			{
				temp=oriData[j];
				labelTemp = labels[j];

				oriData[j]=oriData[j+1];
				labels[j] = labels[j+1];

				oriData[j+1]=temp;
				labels[j+1] = labelTemp;

				flag = false;
			}
		}
		if(flag) break;
	}
}

void StopMotionUI::setLeftWindowasBG()
{
	if (!m_isRightMask)
	{
		m_isRightBg = true;
		//m_isDealLeftImg = true;

		while (!m_leftmaskImgStack.empty())
		{
			m_leftmaskImgStack.pop();
		}

		m_lmaskImage = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
			QImage::Format_ARGB32);

		ui.inputVideo->setMaskImage(m_lmaskImage);

		updateRightAsBG();

	}else
	{
		m_isLeftBg = true;
		//m_isDealLeftImg = true;

		while (!m_rightmaskImgStack.empty())
		{
			m_rightmaskImgStack.pop();
		}

		m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
			QImage::Format_ARGB32);

		ui.outputVideo->setMaskImage(m_rmaskImage);
    
		updateLeftAsBG();
	}



}

void StopMotionUI::setRightWindowasBG()
{
	m_isRightBg = true;
	//m_isDealLeftImg = true;

	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	m_lmaskImage = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
		QImage::Format_ARGB32);

	ui.inputVideo->setMaskImage(m_lmaskImage);

	updateRightAsBG();
}


void StopMotionUI::updateLeftAsBG()
{
    //clear the maskimage
	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	//set a new image
	m_lmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
		QImage::Format_ARGB32);

	ui.inputVideo->setMaskImage(m_lmaskImage);

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	mat2Qimage(bg,m_curShowLeftImg);

	ui.inputVideo->setImage(&m_curShowLeftImg);

	ui.inputVideo->updateDisplayImage();
}


void StopMotionUI::updateRightAsBG()
{
	//clear the maskimage
	while (!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.pop();
	}

	//set a new image
	m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
		QImage::Format_ARGB32);

	ui.outputVideo->setMaskImage(m_rmaskImage);

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	mat2Qimage(bg,m_curShowImg);

	ui.outputVideo->setImage(&m_curShowImg);

	ui.outputVideo->updateDisplayImage();
}

void StopMotionUI::completionWithBG()
{
	if (m_sltRect.size() <= 0)
	{
		Loggger<<"ROIs are empty,please select it again!.\n";
		return;
	}

	IndexType nSkip = 20;

	Rect exR;

	exRect(m_sltRect[0],exR,nSkip);
	
	IndexType width = exR.width;
	IndexType height = exR.height;

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	Mat leftRoi,rightRoi;

	bg(exR).copyTo(leftRoi);

	if (!m_gcutImg.empty())
	{
	   m_gcutImg(exR).copyTo(rightRoi);

	}else
	{
		Loggger<<"None for the first graph cut process!.\n";

		return;
        //source from the right image!
	}


	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	if (m_isRightMask)
	{
		imgData.push_back(rightRoi);//right
		imgData.push_back(leftRoi);//left
		getMaskFromImage(m_rmaskImage,maskFromStk);
	}else
	{
		imgData.push_back(leftRoi);//left
		imgData.push_back(rightRoi);//right
	    getMaskFromImage(m_lmaskImage,maskFromStk);
	}

	MatrixXXi initLabels;

	initLabels.resize(height,width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height,imgData,maskFromStk,initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	_gcutLabels.push_back(resSeg);//for the final p smooth.

	m_cutLabels = resSeg;

	Mat outPutImg;

    m_sMotion->imageGenerate(leftRoi,rightRoi,resSeg,outPutImg);

}

void StopMotionUI::exRect(Rect& oriR, Rect& exR, IndexType nSkip)
{
	exR = oriR;
}

void StopMotionUI::fullScreen()
{
	if (m_isFullS)
	{
		this->setWindowFlags(Qt::SubWindow);
		this->showNormal();
		m_isFullS = false;
	}else
	{
		this->setWindowFlags(Qt::Window);
		this->showFullScreen();
		m_isFullS = true;
	}
}

void StopMotionUI::displayMiddleResults(Mat& mRes)
{
    Mat temp;	          
	cvtColor(mRes,temp,CV_RGB2BGR);

	imwrite(".\\tempImages\\afterLocDef.jpg",temp);

	m_curMiddleImg.load(".\\tempImages\\afterLocDef.jpg");


	QImage lmask = QImage(m_curMiddleImg.width(), m_curMiddleImg.height(), 
		QImage::Format_ARGB32);

	ui.middleResults->setMaskImage(lmask);

	ui.middleResults->setImage(&m_curMiddleImg);

	ui.middleResults->updateDisplayImage();

}


void StopMotionUI::showMiddleCombine(Mat& left,Mat& right)
{

	Mat transAB;

	translucentImages(left,right,0.5, transAB);

// 	m_curMiddleImg = QImage((const unsigned char*)(transAB.data),
// 		transAB.cols,transAB.rows,QImage::Format_RGB888);

	mat2Qimage(transAB,m_curMiddleImg);

	//mat2Qimage(left,m_curMiddleImg);

// 	QImage mask;
// 	mask = QImage(m_curMiddleImg.width(), m_curMiddleImg.height(), 
// 		QImage::Format_ARGB32);


	Mat temp;
	cvtColor(transAB,temp,CV_RGB2BGR);
	imshow("combineImg",temp);


// 	ui.middleResults->clearSltRegion();
// 
// 	ui.middleResults->setRectZero();
// 
// 	ui.middleResults->setMaskImage(mask);

	ui.middleResults->setImage(&m_curMiddleImg);

	ui.middleResults->showMiddleImage();

}

bool StopMotionUI::isMaskFromRight()
{
	bool isRightMask = true;

	IndexType nLMask = m_leftmaskImgStack.size();
	IndexType nRMask = m_rightmaskImgStack.size();

	if (nLMask > nRMask)
	{
		isRightMask = false;
	}
	//
	return isRightMask;
}

void StopMotionUI::detectFGObjects()
{

	if (!m_isVideo)
	{
		Loggger<<"The input is a video, can not do this!.\n";

		return;
	}
	IndexType sFrame, eFrame;

	sFrame = 150;
	eFrame = 400;

	Mat curFrame, fgMask;

	BackgroundSubtractorMOG2 pMOG2;

	char fname[1024];

	for (IndexType i = sFrame; i < eFrame && i < m_totFrames; ++ i)
	{
		m_video.set(CV_CAP_PROP_POS_FRAMES, i );

		m_video>>curFrame;

		pMOG2(curFrame,fgMask);

		sprintf(fname,".\\resCombine\\1102-bg-subtraction-%.2d.jpg",i);

		imwrite(fname,fgMask);

		//updateRight(fgMask);
	}

}

//new interface for global registration

void StopMotionUI::showROIs(Mat& lROI,Mat& rROI)
{

	imwrite("left_ROI.jpg",lROI);

	m_lroi.load("left_ROI.jpg");

	QImage lmask = QImage(m_lroi.width(), m_lroi.height(), 
		QImage::Format_ARGB32);

	ui.middleResults->setMaskImage(lmask);

	ui.middleResults->setImage(&m_lroi,true);	

	ui.middleResults->updateDisplayImage();


	imwrite("right_ROI.jpg",rROI);

	m_rroi.load("right_ROI.jpg");

	QImage rmask = QImage(m_rroi.width(), m_rroi.height(), 
		QImage::Format_ARGB32);

	ui.middleResultsB->setMaskImage(rmask);

	ui.middleResultsB->setImage(&m_rroi,true);	

	ui.middleResultsB->updateDisplayImage();

}

void StopMotionUI::showOriROIs()
{
	//draw global matching points

	Mat ltemp,rtemp;

	m_lROIshow.copyTo(ltemp);
	m_rROIshow.copyTo(rtemp);

    RNG rng(12345);

	for (IndexType i = 0; i < m_gloMatPsLeft.size(); ++ i)
	{
		QPoint curP = m_gloMatPsLeft[i];
        Point2f temp(curP.x(),curP.y());
		Scalar color = Scalar( 0, 0, 255 );
		circle(ltemp,temp,4,color);
	}

	imwrite(".\\tempImages\\left_ROI.jpg",ltemp);

	m_lroi.load(".\\tempImages\\left_ROI.jpg");

	QImage lmask = QImage(m_lroi.width(), m_lroi.height(), 
		QImage::Format_ARGB32);

	ui.middleResults->setMaskImage(lmask);

	ui.middleResults->setImage(&m_lroi,true);	

	ui.middleResults->updateDisplayImage();


	//right
	for (IndexType i = 0; i < m_gloMatPsRight.size(); ++ i)
	{
		QPoint curP = m_gloMatPsRight[i];
		Point2f temp(curP.x(),curP.y());
		Scalar color = Scalar( 0, 255, 0 );
		circle(rtemp,temp,4,color);
	}


	imwrite(".\\tempImages\\right_ROI.jpg",rtemp);

	m_rroi.load(".\\tempImages\\right_ROI.jpg");

	QImage rmask = QImage(m_rroi.width(), m_rroi.height(), 
		QImage::Format_ARGB32);

	ui.middleResultsB->setMaskImage(rmask);

	ui.middleResultsB->setImage(&m_rroi,true);	

	ui.middleResultsB->updateDisplayImage();

}

void StopMotionUI::deleteGloMatchingPsLeft(const QPoint& pos)
{

	//find the nearest point located on the  m_gloMatPsLeft;
	
	IndexType nsize = m_gloMatPsLeft.size();

	vector<ScalarType> dis;
	vector<IndexType> pIdx;
	dis.clear();
	pIdx.clear();

	for (IndexType i = 0; i < nsize; ++ i)
	{
		ScalarType dx = pos.x() - (m_gloMatPsLeft[i].x());
		ScalarType dy = pos.y() - (m_gloMatPsLeft[i].y());

		ScalarType idis = sqrt(dx * dx + dy * dy);
		dis.push_back(idis);
		pIdx.push_back(i);
	}

	bubleSort(dis,pIdx,nsize);

	vector<QPoint>::iterator lit = m_gloMatPsLeft.begin() + pIdx[0];
	m_gloMatPsLeft.erase(lit);

	showOriROIs();
}

void StopMotionUI::deleteGloMatchingPsRight(const QPoint& pos)
{
	//find the nearest point located on the  m_gloMatPsLeft;

	IndexType nsize = m_gloMatPsRight.size();

	vector<ScalarType> dis;
	vector<IndexType> pIdx;
	dis.clear();
	pIdx.clear();

	for (IndexType i = 0; i < nsize; ++ i)
	{
		ScalarType dx = pos.x() - (m_gloMatPsRight[i].x());
		ScalarType dy = pos.y() - (m_gloMatPsRight[i].y());

		ScalarType idis = sqrt(dx * dx + dy * dy);
		dis.push_back(idis);
		pIdx.push_back(i);
	}

	bubleSort(dis,pIdx,nsize);

	vector<QPoint>::iterator lit = m_gloMatPsRight.begin() + pIdx[0];

	m_gloMatPsRight.erase(lit);

	showOriROIs();
}

bool StopMotionUI::globalDeformation(Mat& roiL, Mat& roiR,Mat& outImg)
{
  
	IndexType lmSize = m_gloMatPsLeft.size();
	IndexType rmSize = m_gloMatPsRight.size();

	if ( lmSize == 0 || lmSize != rmSize)
	{
		return false;
	}

	vector<CvPoint2D32f> lMatchingPs, rMatchingPs;

	for (IndexType i = 0; i < lmSize; ++ i)
	{
		CvPoint2D32f curPL;
		curPL.x = m_gloMatPsLeft[i].x();
		curPL.y = m_gloMatPsLeft[i].y();

		CvPoint2D32f curPR;
		curPR.x = m_gloMatPsRight[i].x();
		curPR.y = m_gloMatPsRight[i].y();

		lMatchingPs.push_back(curPL);

		rMatchingPs.push_back(curPR);
	}

	//m_sMotion->gloDeformRoi(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	//global transformation while preserving lines
	m_sMotion->gloDeformRoiwithLines(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	imwrite("gloDefRes.jpg",outImg);

	return true;
}
