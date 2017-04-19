#include "dlg_StopMotionRefine.h"
#include "dlg_canvas.h"

#include "opencv2/video/video.hpp"
#include "opencv2/video/background_segm.hpp"


StopMotionRUI::StopMotionRUI() : m_video(SingleFrameSet::get_instance().getTotVideo() )
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

	m_isLocalDef = false;

	m_psmoothN = 100;

	m_NextStep = Step_ROIS;

	m_iniPos4Smooth.setX(50);
	m_iniPos4Smooth.setY(50);

}

StopMotionRUI::~StopMotionRUI()
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

void StopMotionRUI::initConnect()
{
	connect(m_timeCtl,SIGNAL(timeout()), this, SLOT(nextFrame()));

	connect(ui.playAndPause, SIGNAL(clicked()),this, SLOT( playPause() ) );


	connect(ui.StopPlay, SIGNAL(clicked()), this, SLOT(playStop() ) );

	connect(ui.previousFrame, SIGNAL(clicked()), this, SLOT(preFrame()) );

	connect(ui.nextFrame, SIGNAL(clicked()), this, SLOT(nextFrame() ) );

	connect(ui.isSelectKeyFrameA, SIGNAL( clicked() ), this, SLOT( selectKeyFrameA() ) );

	connect(ui.isSelectKeyFrameB, SIGNAL( clicked() ), this, SLOT( selectKeyFrameB() ) );


	connect(ui.matchingDeform,SIGNAL(clicked()), this, SLOT(deformImgInteraction() ));

	//connect(ui.isBackground, SIGNAL( clicked() ), this, SLOT( selectBackground() ) );

	connect(ui.isBackground, SIGNAL( valueChanged(int) ), this, SLOT( selectBackground(int) ) );

	connect(ui.savePairs, SIGNAL(clicked()) ,this, SLOT(savePairs() ) );

	connect(ui.inVideoView, SIGNAL(valueChanged(int) ), this, SLOT( changeCurFrameIdx(int) ) );

	connect(ui.inVideoView, SIGNAL(valueChanged(int) ), this, SLOT( showTransForSelection(int) ) );

	connect(ui.pairIndex, SIGNAL(valueChanged(int) ), this, SLOT( showPairIdx(int) ) );

	connect(ui.pSmoothIter, SIGNAL(valueChanged(int)), this, SLOT(pSmoothIter(int) ) );

	connect(ui.frameIdx, SIGNAL(valueChanged(int) ), this, SLOT( assignFrameIdx(int) ) );

	//new ui for pair selection 
	//connect(ui.LeftCandIdx, SIGNAL(valueChanged(int) ), this, SLOT( assignLeftFrame(int) ) );

	//connect(ui.RightCandId,SIGNAL(valueChanged(int) ), this, SLOT( assignRightFrame(int) ) );

	//using candidate frame to draw
	connect(ui.LeftCandIdx, SIGNAL(valueChanged(int) ), this, SLOT( changeCurFrameIdx4PairLeft(int) ) );

	connect(ui.RightCandId,SIGNAL(valueChanged(int) ), this, SLOT( changeCurFrameIdx4PairRight(int) ) );


	connect(ui.outputVideo, SIGNAL(changeRect()), this, SLOT(getROIsFromSign() ) );

	//for video for-automatic selection part
	connect(ui.inputVideo, SIGNAL(changeRect()), this, SLOT(getROIsFromSign() ) );

	connect(ui.outputVideo, SIGNAL(deleteRect()), this, SLOT(setDeletePress() ) );

	//using global mask
	// 	connect(ui.outputVideo, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromImages()) );
	// 
	// 	connect(ui.inputVideo, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromLeftImages()) );

	//using ROI's mask

	connect(ui.middleResults, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromLeftROI()));

	connect(ui.middleResultsB,SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromRightROI()) );

	//end mask obtaining

	connect(ui.CombineOperation, SIGNAL(clicked()), this, SLOT(combinePairImages() ));


	connect(ui.MergingImages, SIGNAL(clicked()), this, SLOT(newFuseTwoROIs()) );

	//connect(ui.UndoStroke, SIGNAL(clicked()), this, SLOT(undoStrokeDraw()) );//for global images

	connect(ui.UndoStroke, SIGNAL(clicked()), this, SLOT(undoStrokeROI()) );//for ROIs images

	connect(ui.outputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showCoorRGB(const QPoint&, const QRgb& )) );

	connect(ui.outputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showLabels(const QPoint&, const QRgb& )) );

	//connect(ui.outputVideo, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	//connect(ui.inputVideo, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	//for rois
	connect(ui.middleResults, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	connect(ui.middleResultsB, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)) );

	connect(ui.inputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showCoorRGB(const QPoint&, const QRgb& )) );

	connect(ui.inputVideo, SIGNAL(showCoorAndRGB (const QPoint&, const QRgb&)), this, SLOT(showLabels(const QPoint&, const QRgb& )) );

	//for delete the matching pairs--old version global 
	//connect(ui.outputVideo, SIGNAL(propaCoor(const QPoint& )), this, SLOT(obtainCoor(const QPoint& ) ));

	//connect(ui.inputVideo, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoor(const QPoint& ) ));

	//only local using this point to delete a line
	connect(ui.middleResults, SIGNAL(propaCoor(const QPoint& )), this, SLOT(obtainCoor(const QPoint& ) ));

	connect(ui.middleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoor(const QPoint& ) ));


	//using this point for poisson smooth
	connect(ui.middleResults, SIGNAL(propaCoor(const QPoint& )), this, SLOT(obtainInitPos4Smooth(const QPoint& ) ));

	connect(ui.middleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainInitPos4Smooth(const QPoint& ) ));

	//delete which one?
	// 	connect(ui.outputVideo, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));
	// 
	// 	connect(ui.inputVideo, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));

	//delete the pairs in the ROIs
	connect(ui.middleResults, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));

	connect(ui.middleResultsB, SIGNAL(delCurNearLine()), this, SLOT(delCurLine() ));

	//for global matching points--assign a point in the image
	connect(ui.middleResults, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorLeft(const QPoint& ) ));

	connect(ui.middleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorRight(const QPoint& ) ));

	//pick points for local matching

	connect(ui.middleResults, SIGNAL(propaCoor(const QPoint&)), this, SLOT(drawLocalMatchingPsLeft(const QPoint&) ));

	connect(ui.middleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(drawLocalMatchingPsRight(const QPoint& ) ));

	//delete global matching points
	connect(ui.middleResults, SIGNAL(delGloCoor(const QPoint&)), this, SLOT(deleteGloMatchingPsLeft(const QPoint&)) );

	connect(ui.middleResultsB, SIGNAL(delGloCoor(const QPoint&)), this, SLOT(deleteGloMatchingPsRight(const QPoint&)));

	//update texture image

	connect(ui.afterConbine, SIGNAL(clicked()), this, SLOT(afterCombine() ) );

	connect(ui.selectROI, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.delGlobalMatchingPs, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.selectBackG, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.selectForeG, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.addMatching, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.ImgTranslation, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.gloPick, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	//poisson smooth initPixSmooth

	connect(ui.initPixSmooth, SIGNAL(clicked()), this, SLOT(getStatusTool() ) );

	connect(ui.gloPick, SIGNAL(clicked()), this, SLOT(showArrows() ) );
	//for automatic selection of pair 

	//poisson smooth
	connect(ui.pSmooth, SIGNAL(clicked()), this, SLOT(pSmooth()) );

	connect(ui.sPSmooth, SIGNAL(clicked()), this, SLOT(stopPSmooth()) );

	//using background image for completion
	//connect(ui.setLeftBG,SIGNAL(clicked()), this, SLOT(setLeftWindowasBG()) );

	connect(ui.setLeftBG,SIGNAL(clicked()), this, SLOT(showROIasBG()) );
	//12-14
	//new ui for system
	connect(ui.outputVideo, SIGNAL(meantimeTrans(float&,float&,ImageLabel&)),
		this, SLOT(transImagesSameTime(float&,float&,ImageLabel&) ));

	connect(ui.inputVideo, SIGNAL(meantimeTrans(float&,float&,ImageLabel&)), 
		this, SLOT(transImagesSameTime(float&,float&,ImageLabel&) ));

	connect(ui.middleResults, SIGNAL(meantimeTrans(float&,float&,ImageLabel&)),
		this, SLOT(transImagesSameTime(float&,float&,ImageLabel&) ));

	connect(ui.middleResultsB, SIGNAL(meantimeTrans(float&,float&,ImageLabel&)), 
		this, SLOT(transImagesSameTime(float&,float&,ImageLabel&) ));

	//zoom at the same time

	connect(ui.outputVideo, SIGNAL(bothZoom(float&,ImageLabel&)),
		this, SLOT(zoomSameTime(float&,ImageLabel&)) );

	connect(ui.inputVideo, SIGNAL(bothZoom(float&,ImageLabel&)),
		this, SLOT(zoomSameTime(float&,ImageLabel&)) );

	connect(ui.middleResults, SIGNAL(bothZoom(float&,ImageLabel&)),
		this, SLOT(zoomSameTime(float&,ImageLabel&)) );

	connect(ui.middleResultsB, SIGNAL(bothZoom(float&,ImageLabel&)),
		this, SLOT(zoomSameTime(float&,ImageLabel&)) );

	//gcut for background

	connect(ui.BgGCut, SIGNAL(clicked()), this, SLOT(completionWithBG()) );

	//user study 2
	connect(ui.NextOperation,SIGNAL(clicked()), this, SLOT(videoNextClick()) );

	//for step by step
	connect(ui.imageNextOperationGlob, SIGNAL(clicked()), this, SLOT(nextOperationClick()) );

	connect(ui.UndoStrokeBGOnly,SIGNAL(clicked()), this, SLOT(undo4BGGcut()));

	//auto-selection from video
	connect(ui.autoSelect, SIGNAL(clicked()), this, SLOT(autoSelectionPair() ) );
	connect(ui.bgSub, SIGNAL(clicked()), this, SLOT(detectFGObjects()) );
	connect(ui.savePairsauto, SIGNAL(clicked()) ,this, SLOT(savePairs() ) );
	connect(ui.startId, SIGNAL(valueChanged(int) ), this, SLOT(setstartId(int)) );
	connect(ui.rangeFrame, SIGNAL(valueChanged(int) ), this, SLOT(setLen(int)) );
	connect(ui.band, SIGNAL(valueChanged(int) ), this, SLOT(setBand(int)) );
	connect(ui.backgId, SIGNAL(valueChanged(int) ), this, SLOT(setBackgId(int)) );

	connect(ui.videoSelectROI, SIGNAL(clicked()), this, SLOT(setSelectMode() ));
	connect(ui.VideoImgTranslation, SIGNAL(clicked()), this, SLOT(setTransModel4Video() ));
}

void StopMotionRUI::getStatusTool()
{
 	bool isSelect = ui.selectROI->isChecked();
 	
 	//ui.selectROI->setToolTip("ROI selection");
 
 	bool isGloDel = ui.delGlobalMatchingPs->isChecked();
 
 	bool isBack = ui.selectBackG->isChecked();
 
 	bool isFore = ui.selectForeG->isChecked();
 
 	//for adding matching information
 	bool isAdd = ui.addMatching->isChecked();
 
 	bool isTrans = ui.ImgTranslation->isChecked();
 
 
 	bool isgloClick = ui.gloPick->isChecked();
 
 
 	bool isPick4Smooth = ui.initPixSmooth->isChecked();
 
 
 
 	//update again
 	ui.outputVideo->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);
 
 	ui.inputVideo->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);
 
 	ui.middleResults->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);
 
 	ui.middleResultsB->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);
 
 	ui.middleResultsC->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);
 
 	//draw area curves
 	ui.showAreaCurve->setStatusToolNew2(isSelect,isgloClick,isGloDel,isBack,isFore,isAdd,isTrans,isPick4Smooth);

}


void StopMotionRUI::setSelectMode()
{
	ui.inputVideo->setVideoSelectMode();
}

void StopMotionRUI::setTransModel4Video()
{
    ui.showAreaCurve->setVideoTransModel();
}

void StopMotionRUI::showArrows()
{
	if (ui.middleResults->isVisible())
	{
		
		int mSize = m_gloMatPsLeft.size();

		QString strSize_str = QString("%1 Matching size").arg(mSize);

		ui.MatchingSizeL->setText(strSize_str);

		ui.MatchingSizeL->show();


		int rSize = m_gloMatPsRight.size();

		QString rstrSize_str = QString("%1 Matching size").arg(rSize);

		ui.MatchingSizeR->setText(rstrSize_str);

		ui.MatchingSizeR->show();

		//ui.LArrow->show();
		//ui.RArrow->show();
	}

}
void StopMotionRUI::playPause()
{
 	m_timeCtl->start();
 
 	ui.previousFrame->setEnabled(false);
 
 	ui.nextFrame->setEnabled(false);
 
 	//ui.selectPairStatue->setEnabled(false);
 
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

void StopMotionRUI::preFrame()
{
	m_curFrameIdx --;
	if (m_curFrameIdx >= 0)
	{
		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

		m_video>>m_curImgData;

		//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );

		ui.isBackground->setValue(m_curFrameIdx);

		ui.LeftCandIdx->setValue(m_curFrameIdx);

		ui.RightCandId->setValue(m_curFrameIdx);

		ui.frameIdx->setValue(m_curFrameIdx);

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

void StopMotionRUI::nextFrame()
{
    //assert(m_curFrameIdx <= m_totFrames);

	if (m_curFrameIdx >= m_totFrames)
	{
		Loggger<<"End position.\n";
		return;
	}

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

		ui.isBackground->setValue(m_curFrameIdx);

		ui.LeftCandIdx->setValue(m_curFrameIdx);

		ui.RightCandId->setValue(m_curFrameIdx);

		ui.frameIdx->setValue(m_curFrameIdx);
		//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );

	}
	else
	{
		Loggger<<"The Final!.\n";
		m_timeCtl->stop();
	}
}


void StopMotionRUI::showPairIdx(int idx)
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

	reset4NextPair();

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

		QMessageBox::information(this, "Information", "Out of size!",QMessageBox::Ok);

		return;
	}
}


void StopMotionRUI::reset4NextPair()
{
	m_dfLeftROI.release();
	m_gcNHandROI.release();
	m_gMatchingImg.release();

	while(!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.pop();
	}

	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	//

	ui.middleResults->setFocus();
	ui.middleResults->resetTransZoom();

	ui.middleResultsB->setFocus();
	ui.middleResultsB->resetTransZoom();

	ui.middleResultsC->setFocus();
	ui.middleResultsC->resetTransZoom();

	m_isLeftBg = false;
	m_isRightBg = false;

	m_cutLabels.resize(0,0);

	//global matching
	ui.MatchingSizeL->hide();
	ui.MatchingSizeR->hide();

	ui.LArrow->hide();
	ui.RArrow->hide();


}

//automatic selection
 void StopMotionRUI::autoSelectionPair()
 {
	 //visualization of a short range of optical flow firstly
	 Loggger<<"Start to auto-selection!.\n";

// 	 m_bgId = 25;
// 	 m_srtId = 100;
// 	 m_lenSeq = 100;
// 	 m_band = 15;

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
		 Loggger<<"The input for auto-selection is error!.\n";
		 return;
	 }
	 //m_autoSelect->areaForeground(80,200,1000);// backg-start-length

	 double time_start = (double)clock();

	 //m_autoSelect->areaForegWithRect(m_bgId,m_srtId,m_lenSeq,m_sltRect[0]);//cub-background -80
	 IndexType nShots = m_autoSelect->areaForegWithRectSta(m_bgId,m_srtId,m_lenSeq,m_sltRect[0]);

	 //m_autoSelect->areaForegWithRect(80,250,200,m_sltRect[0]);
	 //test optical flow

	 //m_autoSelect->areaFlowVis(80,180,200);// flow bigger than threshold
	 //phone-30,180,200

	 // with rectangle
	 //m_autoSelect->areaFlowVisRect(80,250,100,m_sltRect[0]);

	 double time_end = (double)clock();

	 double runTime = (time_end-time_start)/(60000.*60);

	 
	 char optical_name[1024];

	 sprintf(optical_name,"D:\\project-cityu\\Projects\\Projects\\STOP MOTION\\Videos-sample\\sig-examples\\%d-autoSelectionTimeRecord.txt",1);

	 FILE *out_timeRecord = fopen(optical_name,"w");

	 fprintf(out_timeRecord, "Badminton time is %f s.\n",runTime);
	 fprintf(out_timeRecord, "Background frame idx = %d; start from = %d; the length = %d.\n",m_bgId,m_srtId,m_lenSeq);
	 fprintf(out_timeRecord, "The total shot selected from this video is %d.\n",nShots);

	 fprintf(out_timeRecord,"\n");

	 fclose(out_timeRecord);

	 Loggger<<"End for auto-selection process.\n";
 }


void StopMotionRUI::pSmoothIter(int nit)
{
    if (nit < 1 || nit > 1001)
    {
		nit = 100;
    }
    
	m_psmoothN = nit;
}

void StopMotionRUI::assignFrameIdx(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_curFrameIdx = fId;
	}
}


void StopMotionRUI::setBackgId(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_bgId = fId;
	}

	m_backGroundIdx = m_bgId;

	wirteBackground();
}

void StopMotionRUI::setstartId(int fId)
{
	if ( fId > 0 && fId < m_totFrames)
	{
		m_srtId = fId;
	}

}

void StopMotionRUI::setLen(int fId)
{
	if ( fId > 0 && fId < (m_totFrames - m_bgId) )
	{
		m_lenSeq = fId;
	}

}

void StopMotionRUI::setBand(int _band)
{
	if (_band <=2 || _band > 1000)
	{
		return;
	}

	m_band = _band;
}

void StopMotionRUI::combinePairImages() //the algorithm interface- stop motion class
{
	
 	if (/*!m_isVideo && */!m_isInitImSeq) //read an image sequence;
 	{
		m_isInitImSeq = initPairFromImageSeq();
	}

	m_isRightBg = false;
	m_isLeftBg = false;

	m_lMatchingPs.clear();
	m_rMatchingPs.clear();

	m_lMPixels.clear();
	m_rMPixels.clear();

	m_seamPts.clear();

	initInteracrionLabels();

	clearMasks();

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

		if (activePair < 0 || activePair >= m_totFrames )
		{
			Loggger<<"Out of size.\n";
			return;
		}

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

	//
	//only selecting the 'Selection' tool
// 		ui.gloPick->setDisabled(true);
// 		ui.delGlobalMatchingPs->setDisabled(true);
// 		ui.selectBackG->setDisabled(true);
// 		ui.selectForeG->setDisabled(true);
// 		ui.addMatching->setDisabled(true);

		//tools
		ui.groupBox_3->setVisible(true);
		ui.selectROI->setVisible(true);
		//default as checked
		ui.selectROI->setChecked(true);
		getStatusTool();
		//
		ui.ImgTranslation->setVisible(true);
		ui.gloPick->setVisible(false);
		ui.delGlobalMatchingPs->setVisible(false);
		ui.selectBackG->setVisible(false);
		ui.selectForeG->setVisible(false);
		ui.addMatching->setVisible(false);

		ui.initPixSmooth->setVisible(false);

		ui.subWinMiddle->setVisible(false);
		ui.subWinLeft->setVisible(false);
		ui.subWinRight->setVisible(false);
		ui.UndoStrokeBGOnly->setVisible(false);

		ui.StepRemindeImg->setVisible(true);
		ui.StepRemindeImg->setText("Step-1: Please select the ROI in right image!");

		//actions
		ui.afterConbine->setVisible(false);
		ui.MergingImages->setVisible(false);
		ui.matchingDeform->setVisible(false);
		ui.setLeftBG->setVisible(false);
		ui.BgGCut->setVisible(false);
		ui.pSmooth->setVisible(false);
		ui.UndoStroke->setVisible(false);
		ui.imageNextOperationGlob->setText("Next");
		//ui.imageNextOperationGlob->setVisible(true);

		//clear the selected points for psmooth
		m_iniSeq4SmPts.clear();

		m_NextStep = Step_ROIS;

}

void StopMotionRUI::clearMasks()
{
  while(!m_rightmaskImgStack.empty())
  {
	  m_rightmaskImgStack.pop();
  }

  while (!m_leftmaskImgStack.empty())
  {
	  m_leftmaskImgStack.pop();
  }

  m_isRightMask = true;

}

void StopMotionRUI::selectKeyFrameA()
{

	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_keyFrame[0] = m_curFrameIdx;

		drawKeyFrame(m_keyFrame[0]);

		if (!ui.isSelectKeyFrameB->isChecked() )
		{
            ui.isSelectKeyFrameB->setEnabled(false); // make sure the pair do not selected the same frame
		}
		   
        ui.curLIdx->setText(QString::number(m_curFrameIdx) );
	}

// 	bool isACheck = ui.isSelectKeyFrameA->isChecked();
// 
// 	bool isBCheck = ui.isSelectKeyFrameB->isChecked();
// 
// 	if (isACheck && isBCheck)
// 	{
// 		addPairs();
// 	}

}

void StopMotionRUI::selectKeyFrameB()
{

	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_keyFrame[1] = m_curFrameIdx;

		if (!ui.isSelectKeyFrameA->isChecked() )
		{
			ui.isSelectKeyFrameA->setEnabled(false); // make sure the pair do not selected the same frame
		}

		ui.curRIdx->setText(QString::number(m_curFrameIdx) );
	}

// 	bool isACheck = ui.isSelectKeyFrameA->isChecked();
// 
// 	bool isBCheck = ui.isSelectKeyFrameB->isChecked();
// 
// 	if (isACheck && isBCheck)
// 	{
// 		addPairs();
// 	}
}

void StopMotionRUI::selectBackground(int bgIdx)
{

	//new ui
	if ( m_curFrameIdx >=0 && m_curFrameIdx <= m_totFrames)
	{
		m_backGroundIdx = bgIdx;

		m_curFrameIdx = bgIdx;


		//set the progress bar
		ui.inVideoView->setValue(m_curFrameIdx);

		//ui.curBGIdx->setText(QString::number(m_curFrameIdx) );// redunant

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

		m_video>>m_curImgData;

		Mat tempImg;
		cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
		m_curShowImg = QImage((const unsigned char*)(tempImg.data),
							tempImg.cols,tempImg.rows,QImage::Format_RGB888);
		ui.inputVideo->setImage(&m_curShowImg);		
		ui.inputVideo->updateDisplayImage();
	}

}


void StopMotionRUI::drawViewBG()
{
	Mat imgTemp, mTemp;

	SingleFrame* cufF = new SingleFrame;

	m_video.set(CV_CAP_PROP_POS_FRAMES, m_backGroundIdx);

	m_video>>imgTemp;

	QImage tempQImg;

	mat2Qimage(imgTemp,tempQImg);

	ui.TempImg->setImage(&tempQImg);	

	ui.TempImg->updateDisplayImage();
}

void StopMotionRUI::addPairs()
{
// 	assert(ui.isSelectKeyFrameA->isChecked() &&
// 		   ui.isSelectKeyFrameB->isChecked() );

// 	bool isLCheck = ui.isSelectKeyFrameA->isChecked();
// 	bool isRCheck = ui.isSelectKeyFrameB->isChecked();
// 
// 	if (!isRCheck || !isLCheck)
// 	{
//         QMessageBox::information(this, "Information", "Both images are checked?",QMessageBox::Ok);
// 		return;
// 	}

	if (m_keyFrame[0] >0 && m_keyFrame[0]< m_totFrames
			&& m_keyFrame[1] >=0 && m_keyFrame[1]<= m_totFrames)
	{
			IndexType pairSize = m_keyPairs.size();
			IndexType pIdx = pairSize + 1;
			IndexType paKeysIdx = frame_frame_to_key(m_keyFrame[0], m_keyFrame[1]);
			m_keyPairs.push_back(std::make_pair(pIdx, paKeysIdx) );

			QString strSize_str = QString("%1 Pairs selected.").arg(m_keyPairs.size());
			//ui.PairSize->setText(strSize_str);
			ui.bgIdx2->setText(strSize_str);

			m_keyFrame[0] = m_keyFrame[1];

           ui.LeftCandIdx->setValue(m_keyFrame[1]);

		   //m_curFrameIdx = m_keyFrame[1];
	}

	//drawKeyFrame(m_keyFrame[0]);

// 	m_keyFrame[0] = m_keyFrame[1] = 0;
// 
// 	ui.curLIdx->setText("");
// 	ui.curRIdx->setText("");
// 
// 	ui.isSelectKeyFrameA->setChecked(false);
// 
// 	ui.isSelectKeyFrameB->setChecked(false);

		//progressBar();
}


void StopMotionRUI::savePairs()
{
	writeImages();
}

void StopMotionRUI::drawKeyFrame(IndexType fIdx)
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
		//assert(fIdx >= 0 && fIdx <= m_totFrames);

		if (fIdx < 0 || fIdx >= m_totFrames)
		{
			Loggger<<"Out of size.\n";
			return;
		}

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

void StopMotionRUI::drawKeyFrame(IndexType fIdx, bool isRight)
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
					m_rmaskImage.fill(1);
				}

			}else
			{
				while (!m_rightmaskImgStack.empty())
				{
					m_rightmaskImgStack.pop();
				}

				m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
					QImage::Format_ARGB32);
				m_rmaskImage.fill(1);
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
					m_lmaskImage.fill(1);
				}

			}else
			{
				while (!m_leftmaskImgStack.empty())
				{
					m_leftmaskImgStack.pop();
				}

				m_lmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
					QImage::Format_ARGB32);
				m_lmaskImage.fill(1);
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
		//assert(fIdx >= 0 && fIdx <= m_totFrames);

		if (fIdx < 0 || fIdx >= m_totFrames)
		{
			Loggger<<"Out of size.\n";
			return;
		}

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


void StopMotionRUI::showTrans4Selection()
{
	if (m_keyFrame[0] < 0 || m_keyFrame[0] >= m_totFrames ||
		m_keyFrame[1] <0 || m_keyFrame[1] >= m_totFrames)
	{
		Loggger<<"Out of size.\n";
		return;
	}

// 	if (ui.isSelectKeyFrameA->isChecked() )
// 	{
		Mat aKey,bKey,transAB;

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_keyFrame[0] );
		m_video>>aKey;

		m_video.set(CV_CAP_PROP_POS_FRAMES, m_keyFrame[1] );
		m_video>>bKey;

		translucentImages(aKey,bKey,0.5, transAB);

		Mat temp;

		cvtColor(transAB,temp,CV_BGR2RGB);

		m_curShowImg = QImage((const unsigned char*)(temp.data),
			temp.cols,temp.rows,QImage::Format_RGB888);

		ui.outputVideo->setImage(&m_curShowImg);		
		ui.outputVideo->updateDisplayImage();
	//}
}
void StopMotionRUI::showTransForSelection(int curFrame)
{
// 	assert(m_keyFrame[0] >=0 && m_keyFrame[0] < m_totFrames);
// 	assert(curFrame >=0 && curFrame < m_totFrames);


	if (m_keyFrame[0] < 0 || m_keyFrame[0] >= m_totFrames ||
		curFrame <0 || curFrame >= m_totFrames)
	{
		Loggger<<"Out of size.\n";
		return;
	}

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

void StopMotionRUI::writeImages()
{
	IndexType pSize = m_keyPairs.size();
	//assert(pSize > 0);

	if (pSize <= 0)
	{
		Loggger<<"The select images are empty!.\n";
		return;
	}

	//filesClear();

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

	QMessageBox::information(this, "Information", "Done for saving images!",QMessageBox::Ok);
}

void StopMotionRUI::wirteBackground()
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

void StopMotionRUI::displayAnImage(bool isRight, cv::Mat& showImg)
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

void StopMotionRUI::disPlayPureImage(bool isRight, QImage& showImg)
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



void StopMotionRUI::playStop()
{
	m_timeCtl->stop();

	m_isPlay = false;

	ui.previousFrame->setEnabled(true);

	ui.nextFrame->setEnabled(true);

	//ui.selectPairStatue->setEnabled(true);//show the number of the pairs

	ui.playAndPause->setEnabled(true);

	ui.isSelectKeyFrameA->setEnabled(true);

	ui.isSelectKeyFrameB->setEnabled(true);
}


void StopMotionRUI::changeCurFrameIdx4PairLeft(int cufFrmIdx)
{
 	if (Step_Pair == m_vNextStep)
 	{
		if (cufFrmIdx >= 0 && cufFrmIdx < m_totFrames)
		{
			m_curFrameIdx = cufFrmIdx;

			m_keyFrame[0] = cufFrmIdx;

			//update the right at the same time
			ui.RightCandId->setValue(cufFrmIdx);

			//set the progress bar
			ui.inVideoView->setValue(m_curFrameIdx);

			//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );// redunant

			m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

			m_video>>m_curImgData;

			Mat tempImg;
			cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
			m_curShowImg = QImage((const unsigned char*)(tempImg.data),
				tempImg.cols,tempImg.rows,QImage::Format_RGB888);
			ui.inputVideo->setImage(&m_curShowImg);		
			ui.inputVideo->updateDisplayImage();

			showTrans4Selection();

		}
 	}//else
// 	{
//       QMessageBox::information(this, "Information", "Please check the left button first!",QMessageBox::Ok);
// 	}

}


void StopMotionRUI::changeCurFrameIdx4PairRight(int cufFrmIdx)
{
 	if (Step_Pair == m_vNextStep)
 	{
		if (cufFrmIdx >= 0 && cufFrmIdx < m_totFrames)
		{
			m_curFrameIdx = cufFrmIdx;

			m_keyFrame[1] = cufFrmIdx;

			//set the progress bar
			ui.inVideoView->setValue(m_curFrameIdx);

			//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );// redunant

			m_video.set(CV_CAP_PROP_POS_FRAMES, m_curFrameIdx );

			m_video>>m_curImgData;

			Mat tempImg;
			cvtColor(m_curImgData,tempImg,CV_BGR2RGB);
			m_curShowImg = QImage((const unsigned char*)(tempImg.data),
				tempImg.cols,tempImg.rows,QImage::Format_RGB888);
			ui.inputVideo->setImage(&m_curShowImg);		
			ui.inputVideo->updateDisplayImage();

			showTrans4Selection();

		}
 	}
// 	{
// 		QMessageBox::information(this, "Information", "Please check the right button first!",QMessageBox::Ok);
// 	}
}
void StopMotionRUI::changeCurFrameIdx(int curFrmIdx)
{
	ui.previousFrame->setEnabled(false);

	ui.nextFrame->setEnabled(false);

	ui.playAndPause->setEnabled(false);

	ui.isSelectKeyFrameA->setEnabled(false);

	ui.isSelectKeyFrameB->setEnabled(false);


	if (curFrmIdx >= 0 && curFrmIdx < m_totFrames)
	{
		m_curFrameIdx = curFrmIdx;

		//set the progress bar
        ui.inVideoView->setValue(m_curFrameIdx);

		//ui.inImgIdx->setText(QString::number(m_curFrameIdx) );// redunant
		//update the idx 
		if (m_vNextStep == Step_BG)
		{
	    	ui.isBackground->setValue(curFrmIdx);
		}

		if (!ui.RightCandId->hasFocus())
		{
		  ui.LeftCandIdx->setValue(curFrmIdx);	
		}

		ui.RightCandId->setValue(curFrmIdx);
		ui.frameIdx->setValue(curFrmIdx);

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


// void StopMotionRUI::assignLeftFrame(int curF)
// {
// 
// }
// 
// void StopMotionRUI::assignRightFrame(int cufF)
// {
// 
// }

void StopMotionRUI::initBaseInfo()
{
	if (!m_isVideo) return;

	m_rate = m_video.get(CV_CAP_PROP_FPS);

	assert(m_rate>1);

	m_delay = 1000/m_rate;

	m_totFrames = m_video.get(CV_CAP_PROP_FRAME_COUNT);

	ui.inVideoView->setRange(0, m_totFrames - 1);


}

void StopMotionRUI::showVideoButtons()
{

	ui.MatchingSizeL->hide();

	ui.MatchingSizeR->hide();

	ui.RArrow->hide();

	ui.LArrow->hide();

	ui.UndoStrokeBGOnly->hide();

	ui.initPixSmooth->hide();

	ui.pSmooth->hide();

	ui.BgGCut->hide();

	ui.matchingDeform->hide();

	ui.MergingImages->hide();
// 	if (m_isVideo)
// 	{
		ui.groupBox->hide();
		ui.groupBox_3->hide();

		ui.sPSmooth->hide();
		ui.strokeLevel->hide();

		//ui.pairIndex->hide();

		ui.coorMouse->hide();
		ui.labels->hide();
		
		ui.pSmoothIter->hide();

		ui.MatchingSize->hide();

		ui.UndoStroke->hide();

		//hide the pairs selection parts
		ui.bgIdx2->setText("Select BG image!");
		ui.StepReminde->setText("Step-1: Please select the background image!");
		ui.isSelectKeyFrameA->hide();
		ui.isSelectKeyFrameB->hide();
		ui.PairSize->hide();
		ui.curLIdx->hide();
		ui.curRIdx->hide();
		ui.savePairs->hide();

		ui.afterConbine->hide();
		ui.imageNextOperationGlob->hide();
		ui.StepRemindeImg->hide();
		//ui.horizontalLayout_2->hide();

		ui.FirstFrame->hide();
		ui.SecondFrame->hide();
		ui.LeftCandIdx->hide();
		ui.RightCandId->hide();

		ui.subWinLeft->hide();
		ui.subWinRight->hide();
		ui.subWinMiddle->hide();

		ui.showAreaCurve->hide();
		ui.VideoImgTranslation->hide();
// 	}else
// 	{
		if (m_isVideo)
		{
			ui.CombineOperation->hide();
			ui.pairIndex->hide();
		}else
		{
			 ui.videoPBox->hide();
			 ui.groupBox_2->hide();
		}



	//}
}

bool StopMotionRUI::initPairFromImageSeq()
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

	//progressBar();

	return true;
}

// this is a old version function, just for test, the real function corresponding combine is the after()
void StopMotionRUI::setROIs()
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

}

void StopMotionRUI::getROIsFromSign()
{
	m_sltRect.clear();

	//for step by step

	if (m_isVideo)
	{
      ui.inputVideo->getRect(m_sltRect);
	}else
	{

	  ui.outputVideo->getRect(m_sltRect);

	  //assert(m_showPairIdx[0] >=0 && m_showPairIdx[1] <m_totFrames);

	  if (m_showPairIdx[0] < 0 || m_showPairIdx[1] >= m_totFrames)
	  {
		  Loggger<<"Out of size.\n";
		  return;
	  }

	  SingleFrameSet& set_ = SingleFrameSet::get_instance();

	  set_[m_showPairIdx[0] ].setSecRect(m_sltRect);

	  set_[m_showPairIdx[1] ].setSecRect(m_sltRect);

	  //ui.inputVideo->updateDisplayImage();
	  drawKeyFrame(m_showPairIdx[0],false);
	  drawKeyFrame(m_showPairIdx[1],true);
	}

	if (m_isVideo)
	{
		return;
	}

	//show the ROIs;
	if (m_sltRect.size() >0)
	{
		ui.middleResults->setVisible(true);

		ui.middleResultsB->setVisible(true);

		ui.middleResultsC->setVisible(true);

		ui.subWinLeft->setVisible(true);
		ui.subWinRight->setVisible(true);
		ui.subWinMiddle->setVisible(true);

		ui.StepRemindeImg->setText("");

   	    initalROIsShow();

// 		ui.gloPick->setDisabled(false);
// 		ui.delGlobalMatchingPs->setDisabled(false);

// 		ui.gloPick->setVisible(true);
// 		ui.delGlobalMatchingPs->setVisible(true);
// 
// 		//operation
// 
// 		ui.afterConbine->setVisible(true);
 		ui.imageNextOperationGlob->setVisible(true);
// 
// 		ui.StepRemindeImg->setText("Please select the matching points!");

	}else
	{
		//hide
		ui.middleResults->hide();

		ui.middleResultsB->hide();

		ui.middleResultsC->hide();

		ui.subWinLeft->setVisible(false);

		ui.subWinRight->setVisible(false);

		ui.subWinMiddle->setVisible(false);

		ui.imageNextOperationGlob->setVisible(false);

		ui.StepRemindeImg->setText("Step-1: Please select the ROI in right image!");

	}

}


void StopMotionRUI::getMaskFromImages()
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


void StopMotionRUI::getMaskFromImage(QImage& _maskIMg, MatrixXXi& _labels)
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

bool StopMotionRUI::getMaskFromImageandCheck(QImage& _maskIMg, MatrixXXi& _labels)
{
	if (m_sltRect.size() <=0)
	{
		Loggger<<"Please select the ROIs!.\n";
		return false;
	}

	bool isValid = false;

	IndexType strRectX = m_sltRect[0].x;
	IndexType strRectY = m_sltRect[0].y;

	IndexType width = m_sltRect[0].width;

	IndexType height = m_sltRect[0].height;

	//assert(width>0 && height>0);


	if (width <= 0 || height <= 0)
	{
		Loggger<<"ROIs are empty.\n";
		return false;
	}

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
				isValid = true;
			}
			else if (curColor == fgColor)
			{
				_labels(hIdx,wIdx) = GC_FGD;
				isValid = true;
			}else
			{
				_labels(hIdx,wIdx) = GC_BGD;
			}
		}
	}

	return isValid;
}


bool StopMotionRUI::getLabesFromMaskROI(QImage& _maskIMg, MatrixXXi& _labels)
{
	if (m_sltRect.size() <=0)
	{
		Loggger<<"Please select the ROIs!.\n";
		return false;
	}

	bool isValid = false;

	IndexType width = _maskIMg.width();

	IndexType height = _maskIMg.height();

	//assert(width>0 && height>0);


	if (width <= 0 || height <= 0)
	{
		Loggger<<"ROIs are empty.\n";
		return false;
	}

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
			QRgb curColor = _maskIMg.pixel(wIdx , hIdx);

			if (curColor == bgColor)
			{
				_labels(hIdx,wIdx) = GC_FGD;
				isValid = true;
			}
			else if (curColor == fgColor)
			{
				_labels(hIdx,wIdx) = GC_FGD;
				isValid = true;
			}else
			{
				_labels(hIdx,wIdx) = GC_BGD;
			}
		}
	}

	return isValid;
}


void StopMotionRUI::getMaskFromLeftImages()
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

	// make left as the 0-label
	m_isRightMask = isMaskFromRight();
}


//mask from ROIs

void StopMotionRUI::getMaskFromLeftROI()
{
  m_lmaskImage = ui.middleResults->getMaskImage(); 

  IndexType width = m_lmaskImage.width();
  IndexType height = m_lmaskImage.height();

//   IndexType oriW = m_curShowImg.width();
//   IndexType oriH = m_curShowImg.height();
// 
//   assert(width == oriW && height == oriH);

  m_leftmaskImgStack.push(m_lmaskImage);

//   QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
//   ui.strokeLevel->setText(strSize_str);

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

  // make left as the 0-label
  m_isRightMask = isMaskFromRight();

}

void StopMotionRUI::getMaskFromRightROI()
{
	m_rmaskImage = ui.middleResultsB->getMaskImage(); 

	//assert()
	IndexType width = m_rmaskImage.width();
	IndexType height = m_rmaskImage.height();

// 	IndexType oriW = m_curShowImg.width();
// 	IndexType oriH = m_curShowImg.height();

	//assert(width == oriW && height == oriH);

	m_rightmaskImgStack.push(m_rmaskImage);

// 	QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
// 	ui.strokeLevel->setText(strSize_str);

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


void StopMotionRUI::initalROIsShow() 
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
    displayMiddleResults(comImg);
	//showmiddle image

// 	cvtColor(comImg,tComImg,CV_RGB2BGR);
// 	imshow("Combine Image",tComImg);

	//show ROIs --
	cvtColor(roiL,tempL,CV_RGB2BGR);
	//imshow("A-ROI",tempL);
	cvtColor(roiR,tempR,CV_RGB2BGR);
	//imshow("B-ROI",tempR);

	tempL.copyTo(m_lROIshow);
	tempR.copyTo(m_rROIshow);

	showROIs(tempL,tempR);

}

void StopMotionRUI::afterCombine()//registration button
{

	if (m_sltRect.size() <=0 )
	{
		Loggger<<"please select the ROIs again!\n";
		return;
	}

	//11-20
	//add a global transformation for the two rois
	Mat gloDefLeft;
	bool isGloD = globalDeformation(m_leftROI,m_rightROI,gloDefLeft); //roiL->roiR

	//all of the ROIs--629
	//using piecewise homography method to align two images first.

	QString strSize_str = QString("%1").arg(m_lMPixels.size());
	ui.MatchingSize->setText(strSize_str);

	Mat outImg;

	//gloDefLeft.copyTo(outImg);

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
 
 	//updateLeftWindowImage(outImg);

	Mat showDImage;
	cvtColor(outImg,showDImage,CV_RGB2BGR);

	showLeftROI(showDImage);

	//after homography transformation
	Mat comImg,tComImg;
	translucentImages(outImg,m_rightROI,0.7,comImg);

	displayMiddleResults(comImg);

	outImg.copyTo(m_dfLeftROI);

    QMessageBox::information(this, "Information", "Done for matching!",QMessageBox::Ok);

	//hide the matching information
	ui.MatchingSizeL->hide();
	ui.MatchingSizeR->hide();

	ui.LArrow->hide();
	ui.RArrow->hide();

	m_NextStep = Step_GMAT;
	//ui.gloPick->setDisabled(true);
	//ui.delGlobalMatchingPs->setDisabled(true);

	//ui.selectBackG->setDisabled(false);
	//ui.selectForeG->setDisabled(false);
	//ui.addMatching->setDisabled(false);
}


void StopMotionRUI::tryDeformROIOnlyMethod()
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

}


void StopMotionRUI::initStopMotionClass()
{
		m_sMotion->setRect(m_sltRect);
		m_sMotion->setGraphCutWeight(m_gWeight);
		m_sMotion->setTresChange(m_thresDiff);
		m_sMotion->setCurPairIdx(m_curPairIdx);
		m_isLeftBg = false;
}

void StopMotionRUI::mat2Qimage(const Mat& srcMat, QImage& desQImage)
{
	IndexType nChannel=srcMat.channels();

	if (nChannel==3)
	{			
		Mat srcTemp = Mat(srcMat.rows,srcMat.cols,srcMat.type());
		srcMat.copyTo(srcTemp);
		desQImage=QImage(srcTemp.cols,srcTemp.rows,QImage::Format_RGB888);
		memcpy(desQImage.bits(),srcTemp.data,srcTemp.cols*srcTemp.rows*3*sizeof(unsigned char));	
	}
	else if (nChannel==4||nChannel==1)
	{
		desQImage = QImage((const unsigned char*)srcMat.data,srcMat.cols,srcMat.rows,srcMat.step,QImage::Format_ARGB32);
	}
}

void StopMotionRUI::qImage2Mat(const QImage& srcQImage,Mat& desMat)
{
	cv::Mat matQ=cv::Mat(srcQImage.height(), srcQImage.width(), CV_8UC4, (uchar*)srcQImage.bits(), srcQImage.bytesPerLine()); 
	desMat=Mat(matQ.rows,matQ.cols,CV_8UC3);
	int from_to[] = { 0,0, 1,1, 2,2 };
	cv::mixChannels(&matQ,1,&desMat,1,from_to,3);
}

void StopMotionRUI::handDectect()
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

void StopMotionRUI::simpleHandClassify(const Mat* srImg, Mat* tgImg)
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

void StopMotionRUI::handMarkerGenerate(const Mat& srImg, MatrixXXi& handMK)
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


void StopMotionRUI::markersGenDiff(const Mat& srImg, const Mat& tgImg/*, MatrixXXi& diffMK*/)
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

		}
	}

	cvtColor(initMark,temp,CV_RGB2BGR);
	imshow("Init-Markers",temp);
}


bool StopMotionRUI::isHandPixel(const Vec3b& oriColor)
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

bool StopMotionRUI::isHandPixelNormal(const Vec3b& oriColor)
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


bool StopMotionRUI::isHandPixelNormal2(const Vec3b& oriColor)
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

void StopMotionRUI::showBackGround()
{
	SingleFrame& bg = SingleFrameSet::get_instance().getBackGround();

	Mat* body_ = bg.body;

	Mat temp;

	cvtColor(*body_, temp,CV_RGB2BGR);

	imshow("BackGround", temp);  
}

void StopMotionRUI::showCoorRGB(const QPoint& pos, const QRgb& rgb_)
{

	QString idx_str = QString("Current Position X = [%1], Y = [%2]").arg(pos.x()).arg(pos.y() );
    ui.coorMouse->setText(idx_str);

	QString rgb_str = QString("Current RGB R = [%1], G = [%2], B = [%3]").arg(qRed(rgb_) ).arg(qGreen(rgb_) ).arg(qBlue(rgb_) );
	ui.rgbMouse->setText(rgb_str);
}

void StopMotionRUI::obtainCoor(const QPoint& pos)
{
	if (ui.selectForeG->isChecked())
	{
		if (m_lMPixels.size() > 0)//only save the selected point if we exist a matching points.
		{
			if (ui.selectForeG->isChecked())
			{
				m_delPCoor.setX(pos.x());
				m_delPCoor.setY(pos.y());

				//findNearestLine();//global
				findNearestLineROI(); //local roi
			}
		}
	}

}


void StopMotionRUI::obtainInitPos4Smooth(const QPoint& pos)
{
	if (ui.initPixSmooth->isChecked())
	{
	   m_iniPos4Smooth = pos;

	   m_iniSeq4SmPts.push_back(pos);
	   //update the windows:
       drawSelePoissPts(); 
	}

}

void StopMotionRUI::obtainCoorLeft(const QPoint& pos)
{
	if(ui.gloPick->isChecked())
	{

		int lSize = m_gloMatPsLeft.size();

		int rSize = m_gloMatPsRight.size();


		if ((lSize - rSize) > 0)
		{
			ui.LArrow->show();
			ui.RArrow->hide();
			return;
		}

		//equals and small cases.

		m_selectPointLeft.setX(pos.x());
		m_selectPointLeft.setY(pos.y());

		m_gloMatPsLeft.push_back(m_selectPointLeft);

		int mSize = m_gloMatPsLeft.size();

		QString strSize_str = QString("%1 Matching size").arg(mSize);

		ui.MatchingSizeL->setText(strSize_str);

		if (m_gloMatPsLeft.size() > m_gloMatPsRight.size())//equal case 
		{
		  ui.LArrow->show();
		  ui.RArrow->hide();
		}

		if (m_gloMatPsLeft.size() == m_gloMatPsRight.size())// small case 
		{
			ui.LArrow->hide();
			ui.RArrow->hide();
		}

		showOriROIs();

	}


}

void StopMotionRUI::obtainCoorRight(const QPoint& pos)
{

	if(ui.gloPick->isChecked())
	{

		int lSize = m_gloMatPsLeft.size();

		int rSize = m_gloMatPsRight.size();

		if ((rSize - lSize) > 0)
		{
			ui.LArrow->hide();
			ui.RArrow->show();
			return;
		}

		m_selectPointRight.setX(pos.x());
		m_selectPointRight.setY(pos.y());

		m_gloMatPsRight.push_back(m_selectPointRight);

		//draw image again

		int mSize = m_gloMatPsRight.size();

		QString strSize_str = QString("%1 Matching size").arg(mSize);
		ui.MatchingSizeR->setText(strSize_str);

		if (m_gloMatPsLeft.size() < m_gloMatPsRight.size()) //equal case
		{
			ui.LArrow->hide();
			ui.RArrow->show();
		}

		if (m_gloMatPsLeft.size() == m_gloMatPsRight.size()) // small case
		{
			ui.LArrow->hide();
			ui.RArrow->hide();
		}

		showOriROIs();
	}

}

void StopMotionRUI::delCurLine()
{

	if(m_delLineIdx >= 0 && m_delLineIdx < m_rMPixels.size())
	{
	    vector<CvPoint2D32f>::iterator lit = m_lMPixels.begin() + m_delLineIdx;
		m_lMPixels.erase(lit);

		vector<CvPoint2D32f>::iterator rit = m_rMPixels.begin() + m_delLineIdx;
		m_rMPixels.erase(rit);

		QString strSize_str = QString("%1").arg(m_lMPixels.size());
		ui.MatchingSize->setText(strSize_str);

		//update image of the window

		//updateCurrentImage(m_seamPts, m_gcNHandROI,m_lMPixels,m_rMPixels);
		Mat tmimg;
		m_gcNHandROI.copyTo(tmimg);
		updateCurrentROIAfterGCutNoMask(m_seamPts,tmimg,m_lMPixels,m_rMPixels);

		//back to the original version
		if ( !m_gMatchingImg.empty() )
		{
			displayMiddleResults(m_gMatchingImg);
		}


	}else
	{
	    Loggger<<"The matching size is none.\n";
		return;
	}


}


void StopMotionRUI::drawLocalMatchingPsLeft(const QPoint& pos)
{
	if (ui.addMatching->isChecked())
	{
		m_locMatPsLeft.clear();
		m_locMatPsLeft.push_back(pos);

		Mat tmimg;
		m_gcNHandROI.copyTo(tmimg);
		
		updateCurrentROIAfterGCutAndPs(m_seamPts,tmimg,m_lMPixels,m_rMPixels,m_locMatPsLeft);
	}
}

void StopMotionRUI::drawLocalMatchingPsRight(const QPoint& pos)
{
	if (ui.addMatching->isChecked())
	{
		m_locMatPsRight.clear();

		m_locMatPsRight.push_back(pos);

		Mat tmimg;
		m_gcNHandROI.copyTo(tmimg);

		updateCurrentROIAfterGCutAndPs(m_seamPts,tmimg,m_lMPixels,m_rMPixels,m_locMatPsRight);
	}
}

void StopMotionRUI::findNearestLineROI()
{
	//directly delete the lines, do not active the line firstlty.

	if (m_lMPixels.size() <= 0)
	{
		return;
	}

	if (m_delPCoor.x() <= 0 || m_delPCoor.y() <= 0 ||
		(m_lMPixels.size() != m_rMPixels.size()))
	{
		Loggger<<"the Matching points are error!\n";
		return;
	}

	vector<ScalarType> p2LineDis;
	p2LineDis.clear();

	for (IndexType i = 0; i < m_lMPixels.size(); ++ i)
	{	
		ScalarType dis = p2LineSegDis(m_delPCoor, m_lMPixels[i],m_rMPixels[i]);
		p2LineDis.push_back(dis);
	}

	IndexType idx = min_element(p2LineDis.begin(),p2LineDis.end()) - p2LineDis.begin();

	m_delLineIdx = idx;

    delCurLine();

// 	if (idx >= 0)
// 	{
// 		QPoint sP,eP;
// 
// 		sP.setX(m_rMPixels[idx].x);
// 		sP.setY(m_rMPixels[idx].y );
// 
// 		eP.setX(m_lMPixels[idx].x);
// 		eP.setY(m_lMPixels[idx].y);
// 
// 		if(m_isRightMask)
// 		{
// 			ui.middleResultsB->setDelLine(sP,eP);
// 		}else
// 		{
// 			ui.middleResults->setDelLine(sP,eP);
// 		}
// 
// 	}

}


void StopMotionRUI::findNearestLine()
{
	if (m_lMPixels.size() <= 0)
	{
		return;
	}

// 	assert(m_delPCoor.x() > 0 && m_delPCoor.y() > 0 
// 		&& (m_lMPixels.size() == m_rMPixels.size()));

	if (m_delPCoor.x() <= 0 || m_delPCoor.y() <= 0 ||
		(m_lMPixels.size() != m_rMPixels.size()))
	{
		Loggger<<"the Matching points are error!\n";
		return;
	}

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

ScalarType StopMotionRUI::p2LineSegDis(QPoint& oP, CvPoint2D32f& lsp, CvPoint2D32f& lep)
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

	 //assert(c2 > 1e-3 || c2 <-1e-3);

	 if (c2 <= 1e-3 && c2 >= -1e-3)
	 {
		 Loggger<<"0 is a denominator.\n";

		 return 1e4; 
	 }

	 ScalarType b = c1/c2;

	 Eigen::Vector2f pb = q1 + b* vL;

	 return (p-pb).norm();
}

void StopMotionRUI::getMatchingPs(vector<QPoint>& mPs)
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

		 bool isSOk = findClosestPointOnSeam(sP,scvp);
		 bool isTOk = findClosestPointOnSeam(tP,tcvp);

		 if (!isTOk || !isSOk)
		 {
             QMessageBox::information(this, "Information", "Please do it later!",QMessageBox::Ok);
			 return;
		 }

		 ScalarType dis= sqrt( (scvp.x-tcvp.x)* (scvp.x-tcvp.x) + 
			                   (scvp.y-tcvp.y)*(scvp.y-tcvp.y) );
		 if ( dis < 2.)
		 {
			 QMessageBox::information(this, "Information", "It's too short!",QMessageBox::Ok);
			 continue;
		 }

		 m_lMPixels.push_back(scvp);
		 m_rMPixels.push_back(tcvp);
	}

	QString strSize_str = QString("%1").arg(m_lMPixels.size());
	ui.MatchingSize->setText(strSize_str);

	//show the current pairs
	//updateCurrentImage(m_seamPts, m_gcNHandROI,m_lMPixels,m_rMPixels);//show global images

	Mat tmimg;
	m_gcNHandROI.copyTo(tmimg);
	//updateCurrentROIAfterGCut(m_seamPts,tmimg,m_lMPixels,m_rMPixels);
	updateCurrentROIAfterGCutNoMask(m_seamPts,tmimg,m_lMPixels,m_rMPixels);
}

void StopMotionRUI::showLabels(const QPoint& pos, const QRgb& rgb_)
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


void StopMotionRUI::testGraphCut(Mat& inputIMg)
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

void StopMotionRUI::initTwoRegion(Mat& inputImg, MatrixXXi& handMark)
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

void StopMotionRUI::assignInitialLabels()
{
	// combine the labels from the m_diff and m_mark

	//'mark' label is a global marker;
	//'diff' label is a ROI marker

	//assert(m_markLabels.rows() > 0 && m_markLabels.cols() > 0);

	IndexType height = m_diffLabels.rows();

	IndexType width = m_diffLabels.cols();

	//assert(height > 0  && width > 0);

	if(height <= 0 || width <= 0)
	{
		Loggger<<"Mask Image is empty!.\n";
		return;
	}

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

void StopMotionRUI::translucentImages(Mat& srIMg, Mat& tgImg, 
									 ScalarType alpha, Mat& resImg)
{
	//assert(alpha >= 0. && alpha <= 1.);

	if(alpha < 0. || alpha > 1.0)
	{
		alpha = 0.5;
	}

	ScalarType beta = 1. - alpha;

	addWeighted(srIMg,alpha,tgImg,beta,0.0, resImg);

}

void StopMotionRUI::checkStillPixels(MatrixXXi& outPutMark, MatrixXXi& stillPixels)
{
//    assert(outPutMark.rows() == stillPixels.rows() );
//    assert(outPutMark.cols() == stillPixels.cols() );

   if ( (outPutMark.rows() != stillPixels.rows()) || (outPutMark.cols() != stillPixels.cols()) )
   {
	   Loggger<<"Check Error.\n";
	   return;
   }

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

void StopMotionRUI::initInteracrionLabels()
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

bool StopMotionRUI::checkTrainData(MatrixXXi& initLabels)
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

void StopMotionRUI::undoStrokeDraw()
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
			m_lmaskImage.fill(1);
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
			m_rmaskImage.fill(1);

			//do not assign 
		}

		ui.outputVideo->setImage(&m_curShowImg);	
	
		ui.outputVideo->setMaskImage(m_rmaskImage);

		ui.outputVideo->updateDisplayImage();
	}// end for if

}



void StopMotionRUI::undoStrokeROI()
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

			getLabesFromMaskROI(m_lmaskImage,m_markLeftLabels);


		}else
		{
			if (m_leftmaskImgStack.empty())
			{
				//Loggger<<"Strack is empty!.\n";
				QMessageBox::information(this, "Information","Stack is empty!",QMessageBox::Ok);
				return;
			}

			m_leftmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_lmaskImage = QImage(m_lroi.width(), m_lroi.height(), 
				QImage::Format_ARGB32);

			m_lmaskImage.fill(1);

		}


		Mat temp;
		cvtColor(m_dfLeftROI,temp,CV_RGB2BGR);

		showLeftROIwihtMask(temp,m_lmaskImage);

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

			getLabesFromMaskROI(m_rmaskImage,m_markLabels);

		}else
		{
			if (m_rightmaskImgStack.empty())
			{
				//Loggger<<"Stack is empty!.\n";
                QMessageBox::information(this, "Information","Stack is empty!",QMessageBox::Ok);
				return;
			}

			m_rightmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_rmaskImage = QImage(m_rroi.width(), m_rroi.height(), 
				QImage::Format_ARGB32);

			m_rmaskImage.fill(1);

			//do not assign 
		}

		//for right ROI


		Mat temp;
		cvtColor(m_rightROI,temp,CV_RGB2BGR);

		showRightROIwihtMask(temp,m_rmaskImage);

	}// end for if

}


void StopMotionRUI::undo4BGGcut()
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

			getLabesFromMaskROI(m_lmaskImage,m_markLeftLabels);


		}else
		{
			if (m_leftmaskImgStack.empty())
			{
				//Loggger<<"Strack is empty!.\n";
				QMessageBox::information(this, "Information","Stack is empty!",QMessageBox::Ok);
				//return;
			}

			m_leftmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_lmaskImage = QImage(m_lroi.width(), m_lroi.height(), 
				QImage::Format_ARGB32);

			m_lmaskImage.fill(1);

		}

		Mat temp;

		if (!m_gMatchingImg.empty())
		{
			cvtColor(m_gMatchingImg,temp,CV_RGB2BGR);

		}else if (!m_gcNHandROI.empty())
		{
			cvtColor(m_gcNHandROI,temp,CV_RGB2BGR);

		}else
		{
			Loggger<<"None for the first graph cut process!.\n";
			return;
			//source from the right image!
		}

		showLeftROIwihtMask(temp,m_lmaskImage);

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

			getLabesFromMaskROI(m_rmaskImage,m_markLabels);

		}else
		{
			if (m_rightmaskImgStack.empty())
			{
				//Loggger<<"Stack is empty!.\n";
				QMessageBox::information(this, "Information","Stack is empty!",QMessageBox::Ok);
				//return;
			}

			m_rightmaskImgStack.pop();

			QString strSize_str = QString("%1 / %2").arg(m_leftmaskImgStack.size()).arg(m_rightmaskImgStack.size());
			ui.strokeLevel->setText(strSize_str);

			m_rmaskImage = QImage(m_rroi.width(), m_rroi.height(), 
				QImage::Format_ARGB32);

			m_rmaskImage.fill(1);

			//do not assign 
		}

		//for right ROI

		Mat temp;

		if (!m_gMatchingImg.empty())
		{
		    cvtColor(m_gMatchingImg,temp,CV_RGB2BGR);
		}else if (!m_gcNHandROI.empty())
		{
			cvtColor(m_gcNHandROI,temp,CV_RGB2BGR);
		}else
		{
			Loggger<<"None for the first graph cut process!.\n";
			return;
			//source from the right image!
		}

		showRightROIwihtMask(temp,m_rmaskImage);

	}// end for if
}

void StopMotionRUI::okDeformation()
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

void StopMotionRUI::fuseTwoROIs()
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
     	//getMaskFromImage(m_rmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_rmaskImage,maskFromStk))
		{
           Loggger<<"Please brush the regions in ROI.\n";
		   return;
		}

	}else
	{
		imgData.push_back(m_dfLeftROI);
		imgData.push_back(m_rightROI);
	    //getMaskFromImage(m_lmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_lmaskImage,maskFromStk))
		{
			Loggger<<"Please brush the regions in ROI.\n";
			return;
		}
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

		//ui.leftSeamIdx->setRange(0,m_totSeamPsLeft - 1);
		//ui.rightSeamIdx->setRange(0,m_totSeamPsright - 1);

		//for visualization

		Mat bg;
		(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

		vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;

		outPutImg.copyTo(m_gcNHandROI);//for deformation,is a bug! can not do again!

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

	}


}

void StopMotionRUI::newFuseTwoROIs()
{
	//Loggger<<"start for image generate.\n";

	if (m_sltRect.size() <=0 )
	{
		Loggger<<"please select the ROIs again!\n";
		QMessageBox::information(this, "Information", "please select the ROIs again!",QMessageBox::Ok);
		return;
	}

	if (m_isLeftBg || m_isRightBg)
	{
		QMessageBox::information(this, "Information", "Please select the BG exchange!",QMessageBox::Ok);
		return;
	}

	if (m_leftmaskImgStack.empty() && m_rightmaskImgStack.empty())
	{
        QMessageBox::information(this, "Information", "Please brush the regions in ROI!",QMessageBox::Ok);
		return;
	}

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

	}else
	{
	    QMessageBox::information(this, "Information", "Source image detected errors!",QMessageBox::Ok);
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
		//m_sMotion->findInitialMatchingAlongSeam(ptsList,lmatchPs,rmatchPs);
		//m_lMPixels = lmatchPs;
		//m_rMPixels = rmatchPs;

	   	// completion
		m_sMotion->imageGenerate(m_dfLeftROI,m_rightROI,expLabels,outPutImg);

	}else
	{
		m_sMotion->setInputImg(m_rightROI,m_dfLeftROI,expLabels);
		//m_sMotion->findInitialMatchingAlongSeam(ptsList,lmatchPs,rmatchPs);
		//m_lMPixels = lmatchPs;
		//m_rMPixels = rmatchPs;

		// completion
		m_sMotion->imageGenerate(m_rightROI, m_dfLeftROI,expLabels,outPutImg);
	}		


	//shows the size of matching points by automatic way.

	QString strSize_str = QString("%1").arg(m_lMPixels.size());
    ui.MatchingSize->setText(strSize_str);


	//Loggger<<"End for image generate.\n";

	//visualize the matching
	//update current image after graph cut
	//updateCurrentImage(ptsList, outPutImg,lmatchPs,rmatchPs);//global images

	outPutImg.copyTo(m_gcNHandROI);

	updateCurrentROIAfterGCut(ptsList, outPutImg,lmatchPs,rmatchPs);//only for roi

	displayMiddleResults(outPutImg);

	//ui.selectForeG->setDisabled(false);
	//ui.addMatching->setDisabled(false);

	QMessageBox::information(this, "Information", "Done for Hand removal!",QMessageBox::Ok);

	m_NextStep = Step_HandR;
}

void StopMotionRUI::updateCurrentImage(vector<Point2f>& ptsList, Mat& outPutImg,
									  vector<CvPoint2D32f>& lmatchPs,vector<CvPoint2D32f>& rmatchPs)
{
	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	vector<Rect> roi = SingleFrameSet::get_instance()[m_showPairIdx[1] ].sltRect;

	if (roi.size() <= 0)
	{
		Loggger<<"ROIs are empty!\n";
		return;
	}

	outPutImg.copyTo(m_gcNHandROI);//for deformation,is a bug! can not do again!

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

void StopMotionRUI::updateCurrentROIAfterGCut(vector<Point2f>& ptsList, Mat& outPutImg, 
											  vector<CvPoint2D32f>& lmatchPs,
											  vector<CvPoint2D32f>& rmatchPs)
{
	//draw the seam on bg
	Scalar color(255,0,0);
	//Point2f tlp = m_sltRect[0].tl();

	for (IndexType i = 0; i < ptsList.size(); ++ i)
	{
		Point2f curP = ptsList[i];
		//curP += tlp;
		circle(outPutImg,curP,.1,color);
	}

	//draw matching lines
	RNG rng(12345);

	for (IndexType i = 0; i < lmatchPs.size(); ++ i)
	{
		Point2f curS = lmatchPs[i];
		Point2f curT = rmatchPs[i];

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		line(outPutImg,(curS),(curT),color,3);

		//circle(bg,(curS + oriPs),2,color);//start points
	}

	if (m_isRightMask)
	{
		Mat tempR;
		
		cvtColor(outPutImg,tempR,CV_RGB2BGR);

		//showRightROI(tempR);

		//using mask images
		showRightROIwihtMask(tempR,m_rmaskImage);

	}else
	{
		Mat tempL;
		cvtColor(outPutImg,tempL,CV_RGB2BGR);
		//showLeftROI(tempL);
		showLeftROIwihtMask(tempL,m_lmaskImage);
	}

}

void StopMotionRUI::updateCurrentROIAfterGCutNoMask(vector<Point2f>& ptsList, Mat& outPutImg, 
											  vector<CvPoint2D32f>& lmatchPs,
											  vector<CvPoint2D32f>& rmatchPs)
{
	//draw the seam on bg
	Scalar color(255,0,0);
	//Point2f tlp = m_sltRect[0].tl();

	for (IndexType i = 0; i < ptsList.size(); ++ i)
	{
		Point2f curP = ptsList[i];
		//curP += tlp;
		circle(outPutImg,curP,.1,color);
	}

	//draw matching lines
	RNG rng(12345);

	for (IndexType i = 0; i < lmatchPs.size(); ++ i)
	{
		Point2f curS = lmatchPs[i];
		Point2f curT = rmatchPs[i];

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

        arrowedLine(outPutImg,(curS),(curT),color,3,8,0,0.4);

	}

	if (m_isRightMask)
	{
		Mat tempR;

		cvtColor(outPutImg,tempR,CV_RGB2BGR);

		showRightROI(tempR);

		//using mask images
		//showRightROIwihtMask(tempR,m_rmaskImage);

	}else
	{
		Mat tempL;
		cvtColor(outPutImg,tempL,CV_RGB2BGR);
		showLeftROI(tempL);
		//showLeftROIwihtMask(tempL,m_lmaskImage);
	}

}

void StopMotionRUI::updateCurrentROIAfterGCutAndPs(vector<Point2f>& ptsList, Mat& outPutImg, 
												vector<CvPoint2D32f>& lmatchPs,
												vector<CvPoint2D32f>& rmatchPs,
												vector<QPoint>& locMatPs)
{
	//draw the seam on bg
	Scalar color(255,0,0);
	//Point2f tlp = m_sltRect[0].tl();

	for (IndexType i = 0; i < ptsList.size(); ++ i)
	{
		Point2f curP = ptsList[i];
		//curP += tlp;
		circle(outPutImg,curP,.1,color);
	}

	//draw points

	for (IndexType i = 0; i < locMatPs.size(); i++)
	{
		Point2f curM;
		curM.x = locMatPs[i].x();
		curM.y = locMatPs[i].y();

		circle(outPutImg,curM,4,color);
	}


	//draw matching lines
	RNG rng(12345);

	for (IndexType i = 0; i < lmatchPs.size(); ++ i)
	{
		Point2f curS = lmatchPs[i];
		Point2f curT = rmatchPs[i];

		Scalar colorMatchingSizeL = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		//line(outPutImg,(curS),(curT),color,3);

		arrowedLine(outPutImg,(curS),(curT),color,3,8,0,0.4);
		//circle(bg,(curS + oriPs),2,color);//start points
	}

	if (m_isRightMask)
	{
		Mat tempR;

		cvtColor(outPutImg,tempR,CV_RGB2BGR);

		showRightROI(tempR);

	}else
	{
		Mat tempL;
		cvtColor(outPutImg,tempL,CV_RGB2BGR);
		showLeftROI(tempL);
	}
}
//
void StopMotionRUI::handRemoveByGCut()
{

	//start to graph cut processing
	// input:  m_dfLeftROI, m_rightROI, few strokes;
	// output: labels (0 is the original pixel, 1 represent a pixel from another frame)

	IndexType height = m_dfLeftROI.rows;
	IndexType width = m_dfLeftROI.cols;

	if (m_sltRect.size() <=0 )
	{
		Loggger<<"please select the ROIs again!\n";
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
		//getMaskFromImage(m_rmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_rmaskImage,maskFromStk))
		{
			//Loggger<<"Please brush the regions in ROI.\n";

	        QMessageBox::information(this, "Information", "Please brush the regions in ROI!",QMessageBox::Ok);
			return;
		}

	}else
	{
		imgData.push_back(m_dfLeftROI);
		imgData.push_back(m_rightROI);
		//getMaskFromImage(m_lmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_lmaskImage,maskFromStk))
		{
			QMessageBox::information(this, "Information", "Please brush the regions in ROI!",QMessageBox::Ok);
			return;
		}
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
void StopMotionRUI::updateLeftWindowImage(Mat& warpedROI)
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

void StopMotionRUI::pSmooth()
{

	//after graph cut process

	if (m_cutLabels.rows() <= 0)//not work 
	{
		Loggger<<"Please removal the hands first!.\n";
		return;
	}

	if (m_sltRect.size() <= 0)
	{
		Loggger<<"Please select again.\n";
		return;
	}


	//only show the combine reuslts on the right window
   showcomImageRight();


//  	Mat bgROI;
//  	Mat bg;
//  
//     (*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);
//  
//     bg(m_sltRect[0]).copyTo(bgROI);
//  
//     if(m_isLocalDef)
//     {
//  	  if(m_isRightMask)
//  	  {
//           finPossionImages(bgROI, m_gMatchingImg, m_rightROI);
//  	  }else
//  	  {
//           //finPossionImages(bgROI, m_gMatchingImg, m_leftROI);
//  		 finPossionImages(bgROI, m_rightROI, m_gMatchingImg);
//  	  }
//  
//     }else
//     {
//  	   if (m_isRightMask)
//  	   {
//             finPossionImages(bgROI, m_dfLeftROI, m_rightROI);
//  	   }else
//  	   {
//  		   finPossionImages(bgROI, m_rightROI,m_dfLeftROI);
//  	   }
//  
//     }

   // the final poisson smooth with background image

   //finalPSmooth(); //background + composite 


//for the second image smooth
   if (m_iniSeq4SmPts.size()> 1)
   {
      secondSmoothImage();
   }

   
   m_NextStep = Step_Smooth;

   QMessageBox::information(this, "Information", "Done for Smoothing! Finished this pose!",QMessageBox::Ok);

   //
   ui.pSmooth->setVisible(false);

   ui.StepRemindeImg->setText("Please turn to the next pair!");

}

void StopMotionRUI::possionImages(Mat& leftImg, Mat& rightImg)
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


void StopMotionRUI::finPossionImagesIdx(Mat& bgImg, Mat& leftImg, Mat& rightImg,IndexType selPtsIdx)
{
	// initialization

	_inputImage.clear();

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

	//possionCompute();

	possionCompute(selPtsIdx);
}

void StopMotionRUI::finPossionImages(Mat& bgImg, Mat& leftImg, Mat& rightImg)
{
	// initialization

	_inputImage.clear();

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

	//possionCompute();

	possionCompute(0);
}


// void StopMotionRUI::finPossionImages(Mat& bgImg, Mat& leftImg, Mat& rightImg)
// {
// 	// initialization
// 
// 	_inputImage.clear();
// 
// 	IndexType width = leftImg.cols;
// 	IndexType heigh = leftImg.rows;
// 
// 	ImageAbs lImg(width,heigh,leftImg);
// 	ImageAbs rImg(width,heigh,rightImg);
// 	ImageAbs bImg(width,heigh,bgImg);
// 
// 	_inputImage.push_back(&rImg); //0
// 	_inputImage.push_back(&lImg); //1
// 	_inputImage.push_back(&bImg); //2
// 
// 	// add a background image 
// 
// 	_blendedImage = ImageAbs(width,heigh);
// 	_blendTemp = new ImageAbs(width,heigh);
// 
// 	_blender = new Adrien::GlobalBlender(width,heigh,&_inputImage, _blendedImage.mutableData(), 1000., stdout);
// 
// 	//possionCompute();
// 
// 	possionCompute(0);
// }


void StopMotionRUI::drawSelePoissPts()
{
	Mat curTemp;

	if (!m_bgBrushROI.empty())
	{
		m_bgBrushROI.copyTo(curTemp);
	}else
	{
		return;
	}

// 	if(!m_gMatchingImg.empty())
// 	{
// 		m_gMatchingImg.copyTo(curTemp);
// 	}else
// 	{
// 		m_dfLeftROI.copySize(curTemp);
// 	}

	//draw select points;


	Scalar color(255,0,0);

	for (IndexType i = 0; i < m_iniSeq4SmPts.size(); ++ i)
	{
		m_iniPos4Smooth = m_iniSeq4SmPts[i];
     	Point iniPts(m_iniPos4Smooth.x(),m_iniPos4Smooth.y());
        circle(curTemp,iniPts,4,color);
	}

	if (m_isRightMask)
	{
		Mat tempR;

		cvtColor(curTemp,tempR,CV_RGB2BGR);

		showRightROI(tempR);

		//using mask images
		//showRightROIwihtMask(tempR,m_rmaskImage);

	}else
	{
		Mat tempL;
		cvtColor(curTemp,tempL,CV_RGB2BGR);
		showLeftROI(tempL);
		//showLeftROIwihtMask(tempL,m_lmaskImage);
	}

}

void StopMotionRUI::possionCompute()
{
	if (!_blender)
	{
		Loggger<<"Blender is empty! \n";
		return;
	}

	changeLabelsTypeFinal(); // for multi images


	double et;

	et = 100;

	int initx = m_iniPos4Smooth.x();
	int inity = m_iniPos4Smooth.y();


	if (_blender && _labels)
	{
		 _blender->pinPixel (initx, inity);
		 _blender->takeComputeData(false, _labels, et);
		 //_blender->compute();
		  _blender->computeIt(400);

		 Loggger<<"Ok for smooth!.\n";

		 unsigned char* res = _blender->getResult();

		 saveSmoothImage(res);

	}else
	{
		Loggger<<"Poisson smooth  for multi-images is error!.\n";
	}

}


void StopMotionRUI::possionCompute(IndexType PtsIdx)
{

	if (!_blender)
	{
		Loggger<<"Blender is empty! \n";
		return;
	}

	changeLabelsTypeFinal(); // for multi images


	double et;

	et = 100;

	if (m_iniSeq4SmPts.size()> 0 && PtsIdx < m_iniSeq4SmPts.size())
	{
		m_iniPos4Smooth = m_iniSeq4SmPts[PtsIdx];
	}else
	{
		m_iniPos4Smooth.setX(10);
		m_iniPos4Smooth.setY(10);
	}


	int initx = m_iniPos4Smooth.x();

	int inity = m_iniPos4Smooth.y();


	if (_blender && _labels)
	{
		_blender->pinPixel (initx, inity);
		_blender->takeComputeData(false, _labels, et);
		//_blender->compute();
		_blender->computeIt(400);

		Loggger<<"Ok for smooth!.\n";

		unsigned char* res = _blender->getResult();

		saveSmoothImage(res);

	}else
	{
		Loggger<<"Poisson smooth  for multi-images is error!.\n";
	}

}

void StopMotionRUI::stopPSmooth()
{
	if (_blender)
	{
		_blender->myStop();
	}
}


void StopMotionRUI::changeLabelsType()
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


void StopMotionRUI::changeLabelsTypeFinal()
{
	if(m_dialCut)
	{
		MatrixXXi resSeg; // = m_dialCut->getLabels();

		//assert(_gcutLabels.size() > 0);

		if (_gcutLabels.size() <= 0)
		{
			Loggger<<"Mask image is error.\n";
			return;
		}

		resSeg = _gcutLabels[0];

		IndexType nW = resSeg.cols();

		IndexType nH = resSeg.rows();

		Mat inMask;
		inMask.create(nH,nW,CV_8UC3);
		inMask.setTo(0);


		for (IndexType i = 1; i < _gcutLabels.size(); ++ i)
		{
			MatrixXXi curLab = _gcutLabels[i];

			for (IndexType k = 0; k < nH; ++ k)
			{
				for (IndexType j = 0; j < nW; ++ j)
				{
					if (curLab(k,j))
					{
                        resSeg(k,j) = i + 1; // right - left - background;
					}
					if (resSeg(k,j) == 1)
					{
						inMask.at<Vec3b>(k,j) = bgMask;
					}else if(resSeg(k,j) == 2)
					{
						inMask.at<Vec3b>(k,j) = fgMask;
					}

				}
			}
		}

		Mat oMask;
		cvtColor(inMask,oMask,CV_RGB2BGR);
		imshow("inMask4Pois",oMask);

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

void StopMotionRUI::finalPSmooth()
{
	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	//using a bigger rectangle for blending,but do not use a global image 

	if (m_sltRect.size() <= 0)
	{
		Loggger<<"roi is empty.\n";
		return;
	}

	Rect curRoi = m_sltRect[0];

	IndexType sWidth = bg.cols;
	IndexType sHeight = bg.rows;

	Rect eRect;
	IndexType eLen = 50;
	expanRect(curRoi,eRect,sWidth,sHeight,eLen);

	IndexType width = eRect.width;
	IndexType heigh = eRect.height;

	Mat bgERoi,curERoi;	
	bg(eRect).copyTo(bgERoi);
	m_pMltSmoothRes(eRect).copyTo(curERoi);

	ImageAbs lImg(width, heigh,bgERoi);
	ImageAbs rImg(width, heigh,curERoi);

	if (!_inputImage.empty())
	{
		_inputImage.clear();
	}

	_inputImage.push_back(&lImg);
	_inputImage.push_back(&rImg);

	_blendedImage = ImageAbs(width,heigh);

	if (_blendTemp)
	{
		delete _blendTemp;
		_blendTemp = NULL;
	}

	_blendTemp = new ImageAbs(width,heigh);

	if (_blender)
	{
		delete _blender;
		_blender = NULL;
	}

	_blender = new Adrien::GlobalBlender(width,heigh,&_inputImage,
		                                 _blendedImage.mutableData(), 1000., stdout);


	//pSmoothFinalCompute();

	Mat fOutimage;
	pSmoothFinalCompute(fOutimage,eLen,curRoi);

	fOutimage.copyTo(m_pMltSmoothRes(eRect));

	//draw the results on the left window!
	updateRight(m_pMltSmoothRes);
	

}


void StopMotionRUI::secondSmoothImage()
{
	Mat bgROI;
	Mat bg;

	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	bg(m_sltRect[0]).copyTo(bgROI);


	if(m_isLocalDef)
	{
		if(m_isRightMask)
		{
			finPossionImagesIdx(bgROI, m_gMatchingImg, m_psMiddleImage,1);
		}else
		{
			finPossionImagesIdx(bgROI, m_gMatchingImg, m_psMiddleImage,1);
		}

	}else
	{
		if (m_isRightMask)
		{
			finPossionImagesIdx(bgROI, m_dfLeftROI, m_psMiddleImage,1);
		}else
		{
			finPossionImagesIdx(bgROI, m_rightROI,m_psMiddleImage,1);
		}

	}

}

void StopMotionRUI::pSmoothFinalCompute(Mat& outImg,IndexType elen, Rect& rct)
{
	if (!_blender)
	{
		Loggger<<"Blender is empty! \n";
		return;
	}

	//changeLabelsType(); //only for two images

	//assignLabels4BgRoi();

	assignLabels4EBgRoi(elen, rct);

	double et;

	et = 100;

	if (_blender && _labels)
	{
		_blender->pinPixel (20, 20);
		_blender->takeComputeData(false, _labels, et);
		//_blender->compute();

		_blender->computeIt(m_psmoothN);

		//Loggger<<"Ok for Final smooth!.\n";

		unsigned char* res = _blender->getResult();

		//saveFinalSmoothImage(res);

		saveFinalSmoothImage(res,outImg);

	}else
	{
		Loggger<<"Poisson smooth  for final background images is error!.\n";
	}
}

void StopMotionRUI::pSmoothFinalCompute()
{
	if (!_blender)
	{
		Loggger<<"Blender is empty! \n";
		return;
	}

	//changeLabelsType(); //only for two images

	assignLabels4BgRoi();

	double et;

	et = 100;

	if (_blender && _labels)
	{
		_blender->pinPixel (10, 10);
		_blender->takeComputeData(false, _labels, et);
		//_blender->compute();

		_blender->computeIt(m_psmoothN);

		//Loggger<<"Ok for Final smooth!.\n";

		unsigned char* res = _blender->getResult();

		saveFinalSmoothImage(res);

	}else
	{
		Loggger<<"Poisson smooth  for final background images is error!.\n";
	}

}

void StopMotionRUI::assignLabels4EBgRoi(IndexType eLen, Rect& oriR)
{
	IndexType nH = _blender->getHeight();
	IndexType nW = _blender->getWidth();

	//assign labels for input
	MatrixXXi resSeg;

	resSeg.setZero(nH,nW);

	//show input
	Mat inMask;
	inMask.create(nH,nW,CV_8UC3);
	inMask.setTo(0);

	Rect roi = m_sltRect[0];

// 	IndexType xS = roi.x;
// 
// 	IndexType yS = roi.y;


	IndexType roiH = oriR.height;

	IndexType roiW = oriR.width;

	for (IndexType i = eLen; i < roiH + eLen; ++ i)
	{
		for (IndexType j = eLen; j < roiW + eLen; ++ j)
		{
			resSeg( i, j) = 1;
			inMask.at<Vec3b>(i, j) = fgMask;
		}
	}

	imshow("Final Smooth IN mask",inMask);


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
}

void StopMotionRUI::assignLabels4BgRoi()
{

	IndexType nH = m_pMltSmoothRes.rows;
	IndexType nW = m_pMltSmoothRes.cols;

	//assign labels for input
	MatrixXXi resSeg;

	resSeg.setZero(nH,nW);

	//show input
// 	Mat inMask;
// 	inMask.create(nH,nW,CV_8UC3);
// 	inMask.setTo(0);

	Rect roi = m_sltRect[0];

	IndexType xS = roi.x;

	IndexType yS = roi.y;

	IndexType roiH = roi.height;

	IndexType roiW = roi.width;

	for (IndexType i = 0; i < roiH; ++ i)
	{
		for (IndexType j = 0; j < roiW; ++ j)
		{
			resSeg( i + yS, j + xS) = 1;
			//inMask.at<Vec3b>(i + yS, j + xS) = fgMask;
		}
	}

	//imshow("Final Smooth IN mask",inMask);


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

}


void StopMotionRUI::saveSmoothImage(unsigned char* res)
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

	smImage.copyTo(m_psMiddleImage);// for next psmooth

	displayMiddleResults(smImage);

	//copy to background image
	Mat bg;

	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	smImage.copyTo(bg(m_sltRect[0]));

	bg.copyTo(m_pMltSmoothRes);

	Mat resTemp;

	cvtColor(bg,resTemp,CV_RGB2BGR);

	displayAnImage(true, bg);

// 	imshow("Smooth image",resTemp);

	char comName[1024];

	sprintf(comName,".\\resCombine\\roi-pSmooth-%.2d.jpg",m_curPairIdx);

	imwrite(comName, resTemp);




}


void StopMotionRUI::saveFinalSmoothImage(unsigned char* res)
{
	Mat smImage;

	IndexType nW = m_pMltSmoothRes.cols;
	IndexType nH = m_pMltSmoothRes.rows;

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

	//displayMiddleResults(smImage);
	//display in right window¡I

	updateRight(smImage);

	Mat resTemp;

	cvtColor(smImage,resTemp,CV_RGB2BGR);

    imshow("Final Smooth Image",resTemp);

	char comName[1024];

	sprintf(comName,".\\resCombine\\final-pSmooth-%.2d.jpg",m_curPairIdx);

	imwrite(comName, resTemp);

}


void StopMotionRUI::saveFinalSmoothImage(unsigned char* res,Mat& outImg)
{
	Mat smImage;

	IndexType nW = _blender->getWidth();

	IndexType nH = _blender->getHeight();

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

	//displayMiddleResults(smImage);
	//display in right window¡I

	//updateRight(smImage);

	smImage.copyTo(outImg);

	Mat resTemp;

	cvtColor(smImage,resTemp,CV_RGB2BGR);

	imshow("Final Smooth Image",resTemp);

	char comName[1024];

	sprintf(comName,".\\resCombine\\final-pSmooth-xroi%.2d.jpg",m_curPairIdx);

	imwrite(comName, resTemp);
}

void StopMotionRUI::drawEditImg(Mat& oriImg,Point& minPs, int height,int width,MatrixXX& quadCoor,MatrixXX& textCoor)
{


	//draw meshes
    ui.outputVideo->updateDeformImage(oriImg,minPs,height,width,quadCoor,textCoor);

	//draw deformed images
	m_canvas->getTextureCanvas()->drawDeformedImg(oriImg,height, width,quadCoor,textCoor);


}

void StopMotionRUI::updateRight(Mat& freImg)
{
	//QImage temp;

	mat2Qimage(freImg,m_curShowImg);

	ui.outputVideo->setImage(&m_curShowImg);

	ui.outputVideo->updateDisplayImage();

}

void StopMotionRUI::updateLeft(Mat& gcutImg)
{
	mat2Qimage(gcutImg,m_curShowLeftImg);

	ui.inputVideo->setImage(&m_curShowLeftImg);

	ui.inputVideo->updateDisplayImage();
}

void StopMotionRUI::feaIdxChangesLeft(int curId)
{
	if (curId > 0 && curId < m_totSeamPsLeft)
	{

		m_lfeaIdx = curId;
		//ui.lfeaIdx->setText(QString::number(curId) );

		Point curCoor = m_seamPts[curId];

		curCoor += m_sltRect[0].tl();

	   Scalar color(255,0,0);

	   Mat temp;
	   m_gcutImg.copyTo(temp);

	   circle(temp,curCoor,5,color);

	   updateRight(temp);
	}
}

void StopMotionRUI::feaIdxChangesRight(int curId)
{
	if (curId > 0 && curId <m_totSeamPsright)
	{
		m_rfeaIdx = curId;

		//ui.rfeaIdx->setText(QString::number(curId) );
		Point curCoor = m_seamPts[curId];

		curCoor += m_sltRect[0].tl();

		Scalar color(255,0,0);

		Mat temp;

		m_gcutImg.copyTo(temp);

		circle(temp,curCoor,5,color);

		updateRight(temp);
	}
}

void StopMotionRUI::addMatching()
{
// 	m_lMatchingPs.push_back(m_lfeaIdx);
// 	m_rMatchingPs.push_back(m_rfeaIdx);
// 
// 	ui.leftSeamIdx->setEnabled(true);
// 	ui.rightSeamIdx->setEnabled(true);
// 
// 	ui.lfeaselect->setChecked(false);
// 
// 	ui.rfeaselect->setChecked(false);

}

//local deformation button
void StopMotionRUI::deformImgInteraction()
{

	m_NextStep = Step_LocalD;
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
		//Loggger<<"please select the matching points.\n";
		QMessageBox::information(this, "Information","Done for local matching!",QMessageBox::Ok);
		m_gcNHandROI.copyTo(m_gMatchingImg);// temp results for bg graph cut processing
		return;
	}


	// void invalid input

	if (m_dfLeftROI.empty() || m_rightROI.empty())
	{
		Loggger<<"Input for deformation is error.\n";
		QMessageBox::information(this, "Information","Input for deformation is error!",QMessageBox::Ok);
		return;
	}

// 	Mat interInf, stateImg;
// 
// 	m_gcNHandROI.copyTo(interInf);
// 
// 	if (m_isRightMask)
// 	{
// 		m_dfLeftROI.copyTo(stateImg);
// 	}else
// 	{
// 		m_rightROI.copyTo(stateImg);
// 	}

	vector<CvPoint2D32f> lmtemp,rmtemp;
	lmtemp.clear();
	rmtemp.clear();

	lmtemp = m_lMPixels;
	rmtemp = m_rMPixels;

 	Scalar color(255,0,0);//red-source
 	Scalar color2(255,255,255);//white-target

	//show matching points

	Mat dOutImg;

	//bool isLines = true;
	bool isLines = false;

	//m_sMotion->setInputImg(m_gcNHandROI,m_rightROI,m_cutLabels); // ? dleftROI correct!

	m_sMotion->setInputImg(m_dfLeftROI,m_rightROI,m_cutLabels);

	if (isLines)
	{
		if (m_isRightMask)
		{
			m_sMotion->deformLinesPoints(m_dfLeftROI,m_rightROI,m_seamPts,
				lmtemp,rmtemp,dOutImg);
		}else
		{
			m_sMotion->deformLinesPoints(m_rightROI,m_dfLeftROI,m_seamPts,
				lmtemp,rmtemp,dOutImg);
		}

	}else
	{
		if (m_isRightMask)
		{
			m_sMotion->deformRoiWithMathcing(m_dfLeftROI,m_rightROI,m_seamPts,
				lmtemp,rmtemp,dOutImg);
		}else
		{
			m_sMotion->deformRoiWithMathcing(m_rightROI,m_dfLeftROI,m_seamPts,
				lmtemp,rmtemp,dOutImg);
		}
	}

	
// 	Mat dOutTemp;
// 	dOutImg.copyTo(dOutTemp);
// 
// 	for (IndexType i= 0; i < m_lMPixels.size(); ++ i)
// 	{
// 		Point2f tp = lmtemp[i];
// 		Point2f tp2 = rmtemp[i];
// 
// 		circle(dOutTemp,tp,5,color);//source
// 		circle(dOutTemp,tp2,5,color2);//target
// 	}
	//displayMiddleResults(dOutTemp);

	//let the dOutImg as the leftROI, for smooth processing;
	if (!m_gMatchingImg.empty())
	{
	   m_gMatchingImg.release();
	}

	//dOutImg.copyTo(m_gMatchingImg);//

	m_isLocalDef = true;

	//
	Mat outPutImg;

 	if (m_isRightMask)
 	{
 		m_sMotion->imageGenerate(dOutImg,m_rightROI,m_cutLabels,outPutImg);
 	}else
 	{
 		m_sMotion->imageGenerate(dOutImg,m_dfLeftROI,m_cutLabels,outPutImg);
 	}

	displayMiddleResults(outPutImg);//middleC

	//only show the results,but do not update the input images, only if the matching points are changed.
// 	Mat ldTemp;
// 	outPutImg.copyTo(ldTemp);
//  updateCurrentROIAfterGCut(m_seamPts, ldTemp,m_lMPixels,m_lMPixels);//only for roi


//	//do not directly to complete the image by the two images.using graph cut again!
// 	if (m_isRightMask)
// 	{
// 		gcAfterDeformation(dOutImg,m_rightROI,outPutImg,true);
// 
// 	}else
// 	{
// 		gcAfterDeformation(dOutImg,m_leftROI,outPutImg,false);
// 	}

	//outPutImg.copyTo(m_gcutImg(m_sltRect[0]));

	outPutImg.copyTo(m_gMatchingImg);// temp results for bg graph cut processing

	QMessageBox::information(this, "Information", "Done for local matching!",QMessageBox::Ok);

}

void StopMotionRUI::gcAfterDeformation(Mat& leftImg, Mat& rightImg, Mat& outImg, bool isRightMask)
{
	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	imgData.push_back(m_rightROI);
	imgData.push_back(m_dfLeftROI);

	if (isRightMask)
	{
	   //getMaskFromImage(m_rmaskImage,maskFromStk);

	   if (!getLabesFromMaskROI(m_rmaskImage,maskFromStk))
	   {
		   Loggger<<"Please brush the regions in ROI.\n";
		   return;
	   }

	}else
	{
	   //getMaskFromImage(m_lmaskImage,maskFromStk);

	   if (!getLabesFromMaskROI(m_lmaskImage,maskFromStk))
	   {
		   Loggger<<"Please brush the regions in ROI.\n";
		   return;
	   }
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

void StopMotionRUI::selectLeftFeatures()
{
// 	bool isLeftCheck = ui.lfeaselect->isChecked();
// 
// 	if (isLeftCheck)
// 	{
// 		ui.leftSeamIdx->setEnabled(false);
// 	}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
// 
// 	bool isRightCheck = ui.rfeaselect->isChecked();
// 
// 	if (isLeftCheck && isRightCheck)
// 	{
// 		addMatching();
// 	}
}


void StopMotionRUI::selectRighFeatures()
{
// 	bool isLeftCheck = ui.lfeaselect->isChecked();
// 
// 	bool isRightCheck = ui.rfeaselect->isChecked();
// 
// 	if (isRightCheck)
// 	{
// 		ui.rightSeamIdx->setEnabled(false);
// 	}
// 
// 	if (isLeftCheck && isRightCheck)
// 	{
// 		addMatching();
// 	}
}

bool StopMotionRUI::findClosestPointOnSeam(QPoint& curP, Point& cloPs)
{
	IndexType nSize = m_seamPts.size();

	//assert(nSize > 0);

	if (nSize <= 0)
	{
		Loggger<<"finding closed points makes error.\n";
		return false;
	}

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
	
	return true;
}

void StopMotionRUI::bubleSort(vector<ScalarType>& oriData,vector<IndexType>& labels,IndexType lSize)
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

void StopMotionRUI::showROIasBG()
{
	if (m_sltRect.size() <= 0)
	{
		Loggger<<"ROIs empty!.\n";
		return;
	}

   //clear the local matching points after local deformation

   m_lMPixels.clear();
   m_rMPixels.clear();

	Mat bgROI,bgROITemp;
	Mat bg;

	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	bg(m_sltRect[0]).copyTo(bgROI);

	cvtColor(bgROI,bgROITemp,CV_RGB2BGR);

	if (!m_isRightMask)
	{
		m_isRightBg = true;
		//m_isDealLeftImg = true;

		while (!m_leftmaskImgStack.empty())
		{
			m_leftmaskImgStack.pop();
		}
		
		showRightROI(bgROITemp);

	}else
	{
		m_isLeftBg = true;
		//m_isDealLeftImg = true;

		while (!m_rightmaskImgStack.empty())
		{
			m_rightmaskImgStack.pop();
		}

		showLeftROI(bgROITemp);
	}

	//QMessageBox::information(this, "Information", "Done for Background replace!",QMessageBox::Ok);
}



void StopMotionRUI::updateROI4BGGCut()
{
	Mat curImg;

	if (!m_gMatchingImg.empty())
	{
		m_gMatchingImg.copyTo(curImg);

	}else if (!m_gcNHandROI.empty())
	{
		m_gcNHandROI.copyTo(curImg);

	}else
	{
		Loggger<<"None for the first graph cut process!.\n";

		return;
		//source from the right image!
	}

	if (m_isRightMask)
	{
		Mat tempR;

		cvtColor(curImg,tempR,CV_RGB2BGR);
		showRightROI(tempR);
		//using mask images
		//showRightROIwihtMask(tempR,m_rmaskImage);

	}else
	{
		Mat tempL;
		cvtColor(curImg,tempL,CV_RGB2BGR);
		showLeftROI(tempL);
		//showLeftROIwihtMask(tempL,m_lmaskImage);
	}

}

void StopMotionRUI::setLeftWindowasBG()
{
	if (m_cutLabels.rows() <= 0)
	{
		Loggger<<"Please completed the ROI first!.\n";
		return;
	}

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
		m_lmaskImage.fill(1);

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
		m_rmaskImage.fill(1);

		ui.outputVideo->setMaskImage(m_rmaskImage);
    
		updateLeftAsBG();
	}



}

void StopMotionRUI::setRightWindowasBG()
{
	m_isRightBg = true;
	//m_isDealLeftImg = true;

	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	m_lmaskImage = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
		QImage::Format_ARGB32);
	m_lmaskImage.fill(1);

	ui.inputVideo->setMaskImage(m_lmaskImage);

	updateRightAsBG();
}


void StopMotionRUI::updateLeftAsBG()
{
    //clear the maskimage
	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	//set a new image
	m_lmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
		QImage::Format_ARGB32);
	m_lmaskImage.fill(1);

	ui.inputVideo->setMaskImage(m_lmaskImage);

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	mat2Qimage(bg,m_curShowLeftImg);

	ui.inputVideo->setImage(&m_curShowLeftImg);

	ui.inputVideo->updateDisplayImage();
}


void StopMotionRUI::updateRightAsBG()
{
	//clear the maskimage
	while (!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.pop();
	}

	//set a new image
	m_rmaskImage = QImage(m_curShowImg.width(), m_curShowImg.height(), 
		QImage::Format_ARGB32);
	m_rmaskImage.fill(1);

	ui.outputVideo->setMaskImage(m_rmaskImage);

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	mat2Qimage(bg,m_curShowImg);

	ui.outputVideo->setImage(&m_curShowImg);

	ui.outputVideo->updateDisplayImage();
}

void StopMotionRUI::completionWithBG()
{
	if (m_sltRect.size() <= 0)
	{
		Loggger<<"ROIs are empty,please select it again!.\n";
		return;
	}

	if (!m_isLeftBg & !m_isRightBg)
	{
		return;
	}	

	//end for refine
// 	IndexType nSkip = 20;
// 
// 	exRect(m_sltRect[0],exR,nSkip);

	Rect exR;	
	exR = m_sltRect[0];

	IndexType width = exR.width;
	IndexType height = exR.height;

	Mat bg;
	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);

	Mat leftRoi,rightRoi;

	bg(exR).copyTo(leftRoi);

	if (!m_gMatchingImg.empty())
	{
	   m_gMatchingImg.copyTo(rightRoi);

	}else if (!m_gcNHandROI.empty())
	{
		m_gcNHandROI.copyTo(rightRoi);

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
		//getMaskFromImage(m_rmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_rmaskImage,maskFromStk))
		{
			Loggger<<"Please brush the regions in ROI.\n";
			return;
		}

	}else
	{
		imgData.push_back(leftRoi);//left
		imgData.push_back(rightRoi);//right
	    //getMaskFromImage(m_lmaskImage,maskFromStk);

		if (!getLabesFromMaskROI(m_lmaskImage,maskFromStk))
		{
			Loggger<<"Please brush the regions in ROI.\n";
			return;
		}

	}

	MatrixXXi initLabels;

	initLabels.resize(height,width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height,imgData,maskFromStk,initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	if (_gcutLabels.size() < 2)
	{
	    _gcutLabels.push_back(resSeg);//for the final p smooth.
	}else
	{
        _gcutLabels[1] = resSeg;
	}

	m_cutLabels = resSeg;

	Mat outPutImg;

    m_sMotion->imageGenerate(leftRoi,rightRoi,resSeg,outPutImg);

	outPutImg.copyTo(m_bgBrushROI);

	displayMiddleResults(outPutImg);

	//update the ROI images

	if (m_isRightMask)
	{
		Mat tempR;

		cvtColor(outPutImg,tempR,CV_RGB2BGR);

		//using mask images
		showRightROIwihtMask(tempR,m_rmaskImage);

	}else
	{
		Mat tempL;
		cvtColor(outPutImg,tempL,CV_RGB2BGR);
		//showLeftROI(tempL);
		showLeftROIwihtMask(tempL,m_lmaskImage);
	}


	m_NextStep = Step_BGGCut;

	QMessageBox::information(this, "Information", "Done for BG replacement!",QMessageBox::Ok);

}



void StopMotionRUI::exRect(Rect& oriR, Rect& exR, IndexType nSkip)
{
	exR = oriR;
}

void StopMotionRUI::fullScreen()
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

void StopMotionRUI::displayMiddleResults(Mat& mRes)
{
    Mat temp;	          
	cvtColor(mRes,temp,CV_RGB2BGR);

	imwrite(".\\tempImages\\afterLocDef.jpg",temp);

	m_curMiddleImg.load(".\\tempImages\\afterLocDef.jpg");


	QImage lmask = QImage(m_curMiddleImg.width(), m_curMiddleImg.height(), 
		QImage::Format_ARGB32);

	lmask.fill(1);

	ui.middleResultsC->setMaskImage(lmask);

	ui.middleResultsC->setImage(&m_curMiddleImg);

	ui.middleResultsC->updateDisplayImage();

}


void StopMotionRUI::showMiddleCombine(Mat& left,Mat& right)
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

bool StopMotionRUI::isMaskFromRight()
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

void StopMotionRUI::detectFGObjects()
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

void StopMotionRUI::showROIs(Mat& lROI,Mat& rROI)
{
	showLeftROI(lROI);
	showRightROI(rROI);


// 	imwrite(".\\tempImages\\ori_left_ROI.jpg",lROI);
// 
// 	m_lroi.load(".\\tempImages\\ori_left_ROI.jpg");
// 
// 
// 	QImage lmask = QImage(m_lroi.width(), m_lroi.height(), 
// 		QImage::Format_ARGB32);
// 
// 	lmask.fill(1);
// 
// 	//lmask = m_lroi.copy();
// 
// 	ui.middleResults->setMaskImage(lmask);
// 
// 	ui.middleResults->setImage(&m_lroi,true);	
// 
// 	ui.middleResults->updateDisplayImage();
// 
// 
// 	imwrite(".\\tempImages\\ori_right_ROI.jpg",rROI);
// 
// 	m_rroi.load(".\\tempImages\\ori_right_ROI.jpg");
// 
// 	QImage rmask = QImage(m_rroi.width(), m_rroi.height(), 
// 		QImage::Format_ARGB32);
// 
// 	rmask.fill(1);
// 	//rmask = m_rroi.copy();
// 
// 	ui.middleResultsB->setMaskImage(rmask);
// 
// 	ui.middleResultsB->setImage(&m_rroi,true);	
// 
// 	ui.middleResultsB->updateDisplayImage();

}


void StopMotionRUI::showLeftROI(Mat& lroi)
{
	imwrite(".\\tempImages\\ori_left_ROI.jpg",lroi);

	m_lroi.load(".\\tempImages\\ori_left_ROI.jpg");


	QImage lmask = QImage(m_lroi.width(), m_lroi.height(), 
		QImage::Format_ARGB32);

	lmask.fill(1);

	//lmask = m_lroi.copy();

	ui.middleResults->setMaskImage(lmask);

	ui.middleResults->setImage(&m_lroi,true);	

	ui.middleResults->updateDisplayImage();
}

void StopMotionRUI::showRightROI(Mat& rroi)
{
	imwrite(".\\tempImages\\ori_right_ROI.jpg",rroi);

	m_rroi.load(".\\tempImages\\ori_right_ROI.jpg");

	QImage rmask = QImage(m_rroi.width(), m_rroi.height(), 
		QImage::Format_ARGB32);

	rmask.fill(1);
	//rmask = m_rroi.copy();

	ui.middleResultsB->setMaskImage(rmask);

	ui.middleResultsB->setImage(&m_rroi,true);	

	ui.middleResultsB->updateDisplayImage();

}

void StopMotionRUI::showLeftROIwihtMask(Mat& lroi, QImage& mask)
{
	imwrite(".\\tempImages\\ori_left_ROI.jpg",lroi);

	m_lroi.load(".\\tempImages\\ori_left_ROI.jpg");


	if (mask.height() != lroi.rows)
	{
		mask = QImage(m_lroi.width(), m_lroi.height(), 
			QImage::Format_ARGB32);
	    mask.fill(1);
	}


	ui.middleResults->setMaskImage(mask);

	ui.middleResults->setImage(&m_lroi,true);	

	ui.middleResults->updateDisplayImage();
}


void StopMotionRUI::showRightROIwihtMask(Mat& rroi, QImage& mask)
{
	imwrite(".\\tempImages\\ori_right_ROI.jpg",rroi);

	m_rroi.load(".\\tempImages\\ori_right_ROI.jpg");


	if (mask.height() != rroi.rows)
	{
		mask = QImage(m_rroi.width(), m_rroi.height(), 
			QImage::Format_ARGB32);
		mask.fill(1);
	}

	ui.middleResultsB->setMaskImage(mask);

	ui.middleResultsB->setImage(&m_rroi,true);	

	ui.middleResultsB->updateDisplayImage();

}

void StopMotionRUI::showOriROIs()
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

	lmask.fill(1);

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

	rmask.fill(1);

	ui.middleResultsB->setMaskImage(rmask);

	ui.middleResultsB->setImage(&m_rroi,true);	

	ui.middleResultsB->updateDisplayImage();

}

void StopMotionRUI::deleteGloMatchingPsLeft(const QPoint& pos)
{

	//find the nearest point located on the  m_gloMatPsLeft;
	
	IndexType nsize = m_gloMatPsLeft.size();

	if (nsize <= 0)
	{
		//Loggger<<"Matching size empty!.\n";
        QMessageBox::information(this, "Information", "Points Empty!",QMessageBox::Ok);
		return;
	}

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


	int mSize = m_gloMatPsLeft.size();

	QString strSize_str = QString("%1 Matching size").arg(mSize);

	ui.MatchingSizeL->setText(strSize_str);

	showOriROIs();
}

void StopMotionRUI::deleteGloMatchingPsRight(const QPoint& pos)
{
	//find the nearest point located on the  m_gloMatPsLeft;

	IndexType nsize = m_gloMatPsRight.size();

	if(nsize<= 0)
	{
		Loggger<<"Matching size empty!.\n";
		return;
	}

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

	int mSize = m_gloMatPsRight.size();

	QString strSize_str = QString("%1 Matching size").arg(mSize);

	ui.MatchingSizeR->setText(strSize_str);

	showOriROIs();

}

bool StopMotionRUI::globalDeformation(Mat& roiL, Mat& roiR,Mat& outImg)
{
  
	vector<CvPoint2D32f> lMatchingPs, rMatchingPs;

    //global matching
	//m_sMotion->getMatchingPoints(roiL,roiR,lMatchingPs,rMatchingPs);

	//interaction by user

	IndexType lmSize = m_gloMatPsLeft.size();

	IndexType rmSize = m_gloMatPsRight.size();

	if (lmSize != rmSize  )
	{
		QMessageBox::information(this, "Information", "Matching points are not equal!",QMessageBox::Ok);
		
		m_gloMatPsLeft.clear();
		m_gloMatPsRight.clear();
		return false;
	}


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

	IndexType lgSize = lMatchingPs.size();
	IndexType rgSize = rMatchingPs.size();

	if ( lgSize == 0 || lgSize != rgSize)
	{
		QMessageBox::information(this, "Information", "Matching points are not equal!",QMessageBox::Ok);
		return false;
	}


	//preserving lines
    //m_sMotion->gloDeformRoiwithLines(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	//without preserving lines
	m_sMotion->gloDeformRoi(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	//global transformation while preserving lines

	//imwrite("gloDefRes.jpg",outImg);

	//for undo operation
	//m_gloMatPsLeft.clear();

	//m_gloMatPsRight.clear();

	return true;
}

//new UI for system


void StopMotionRUI::transImagesSameTime(float& dx,float& dy,ImageLabel& _panter)
{

	if ( dx == 0 && dy == 0)
	{
		return;
	}

	if (&(_panter) == (ui.inputVideo))
	{
		ui.outputVideo->setTranslation(dx,dy);
	}

	if (&(_panter) == (ui.outputVideo))
	{
        ui.inputVideo->setTranslation(dx,dy);
	}

	if (&(_panter) == (ui.middleResults))
	{
		ui.middleResultsB->setTranslation(dx,dy);
	}

	if (&(_panter) == (ui.middleResultsB))
	{
		ui.middleResults->setTranslation(dx,dy);
	}
}


void StopMotionRUI::zoomSameTime(float& ration, ImageLabel& _panter)
{
	if ( ration <= 0.)
	{
		return;
	}

	if (&(_panter) == (ui.inputVideo))
	{
		ui.outputVideo->setZoom(ration);
	}

	if (&(_panter) == (ui.outputVideo))
	{
		ui.inputVideo->setZoom(ration);
	}

	if (&(_panter) == (ui.middleResults))
	{
		ui.middleResultsB->setZoom(ration);
	}

	if (&(_panter) == (ui.middleResultsB))
	{
		ui.middleResults->setZoom(ration);
	}

}

void StopMotionRUI::expanRect(Rect& curRoi, Rect& eRect, IndexType width, 
							  IndexType heigh, IndexType eLen)
{
	if (eLen <= 0)
	{
		Loggger<<"The length of expain is error!.\n";
		return;
	}

	IndexType curX = curRoi.x;
	IndexType curY = curRoi.y;

	IndexType nW = curRoi.width;
	IndexType nH = curRoi.height;

	if (curX - eLen > 0 )
	{
		eRect.x = curX - eLen;
	}else
	{
		eRect.x = curX;
	}


	if (curY - eLen > 0)
	{
		eRect.y = curY - eLen;
	}else
	{
		eRect.y = curY;
	}

	if (nW + eLen < width)
	{
		eRect.width = nW + 2*eLen;
	}else
	{
		eRect.width = nW;
	}

	if (nH + eLen < heigh)
	{
		eRect.height = nH + 2*eLen;
	}else
	{
		eRect.height = nH;
	}

}

void StopMotionRUI::filesClear()
{
	QString cur_dir= ".\\Pairs";

	QString dir = QFileDialog::getExistingDirectory(this,tr("Import image files"), cur_dir);

	if (dir.isEmpty())
		return ;

	QDir file_dir(dir);

	if ( !file_dir.exists() )
	{
		return;
	}
	file_dir.setFilter(QDir::Files);

	QFileInfoList file_list = file_dir.entryInfoList();

	if (file_list.size() <= 0) //empty
	{
		return;
	}

	for (IndexType file_idx = 0; file_idx < file_list.size(); file_idx++)
	{
		QFileInfo file_info = file_list.at(file_idx);

		file_list.removeAll(file_info);
	}

}

//record time of each operation
double startGlobalMatchTime = 0.;
double startHandsRemovalTime = 0.;
double startLocalMatchingTime = 0.;
double startBGReplaceTime = 0.;
double startSmoothTime = 0.;


double gmT = 0.;
double hrTime = 0.;
double lmTime = 0.;
double brTime = 0.;

void StopMotionRUI::nextOperationClick()
{

	//If no interaction happened, go to next step with the default operation.

	char optical_name[1024];

	sprintf(optical_name,".\\Pairs\\%.2d-HandsRemovalTime.txt",1);

	FILE *out_timeRecord = fopen(optical_name,"ab");

	fprintf(out_timeRecord, "Start to hands removal of the %2d pair.\n",m_curPairIdx);

	//global matching
	if ( Step_ROIS == m_NextStep)//already selected the ROI
	{
		startGlobalMatchTime = (double)clock();
		//tools

		ui.selectROI->setVisible(false);

		ui.gloPick->setVisible(true);

		//default
		ui.gloPick->setChecked(true);
		getStatusTool();
		showArrows();
		//show matching points
		
		//
		ui.delGlobalMatchingPs->setVisible(true);

		ui.StepRemindeImg->setText("Step-2: Please select the matching points!");

		//operation

		ui.afterConbine->setVisible(true);

		ui.imageNextOperationGlob->setVisible(true);


	}else if ( Step_GMAT == m_NextStep)// hands removal
	{
		startHandsRemovalTime = (double) clock();

		gmT = (startHandsRemovalTime - startGlobalMatchTime)/(1000.);

		//fprintf(out_timeRecord, "1: Global matching time %.2f s.\n",gmT);

		//fclose(out_timeRecord);

		//tools
		ui.gloPick->setVisible(false);

		ui.delGlobalMatchingPs->setVisible(false);

		ui.selectBackG->setVisible(true);

		//default
		ui.selectBackG->setChecked(true);
		getStatusTool();

		ui.UndoStroke->setVisible(true);

		ui.StepRemindeImg->setText("Step-3: Draw strokes on hands region in Right image!");
		//operation
		ui.MergingImages->setVisible(true);

		ui.afterConbine->setVisible(false);

		// mouse set as normal
		ui.middleResults->setCursor(Qt::ArrowCursor);
		ui.middleResultsB->setCursor(Qt::ArrowCursor);

	}else if (Step_HandR == m_NextStep)//local matching
	{
		startLocalMatchingTime = (double) clock();

		hrTime = (startLocalMatchingTime - startHandsRemovalTime)/(1000.);

		//fprintf(out_timeRecord, "2: Hands removal time %.2f s.\n",hrTime);

		//tools
		ui.selectBackG->setVisible(false);

		ui.UndoStroke->setVisible(false);

		ui.selectForeG->setVisible(true);

		ui.addMatching->setVisible(true);

		ui.MatchingSizeL->setVisible(false);

		ui.MatchingSizeR->setVisible(false);

		//default
		ui.ImgTranslation->setChecked(true);
		getStatusTool();

		

		ui.StepRemindeImg->setText("Step-4: Draw few lines along the red seam");
								   //("if you want to refine the results!");
		//operations

		ui.matchingDeform->setVisible(true);

		ui.MergingImages->setVisible(false);

	}else if(Step_LocalD == m_NextStep) //background replace
	{
		startBGReplaceTime = (double) clock();

		lmTime = (startBGReplaceTime- startLocalMatchingTime)/(1000.);

		//fprintf(out_timeRecord, "3: Local matching time %.2f s.\n",lmTime);

		//functions
		showROIasBG();

		updateROI4BGGCut();

		//tools
		ui.selectForeG->setVisible(false);

		ui.addMatching->setVisible(false);

		ui.matchingDeform->setVisible(false);

		ui.selectBackG->setVisible(true);

		ui.selectBackG->setChecked(true);
		//default
		getStatusTool();

		ui.StepRemindeImg->setText("Step-5:Brush the image on those Margin of the image!");

		//operation
		ui.BgGCut->setVisible(true);

		ui.UndoStrokeBGOnly->setVisible(true);


	}else if (Step_BGGCut == m_NextStep) //smooth operation
	{

		startSmoothTime = (double) clock();

		brTime = (startSmoothTime - startBGReplaceTime)/(1000.);

		fprintf(out_timeRecord, "1: Global matching %.2f s; 2: Hands removal %.2f s; 3: Local Matching %.2f s; 4: BG replace time %.2f s.\n",gmT,hrTime,lmTime,brTime);

		fclose(out_timeRecord);

		//tools
		ui.selectBackG->setVisible(false);

		ui.BgGCut->setVisible(false);

		ui.StepRemindeImg->setText("Step-6: Select one point, Smooth the boundary!");

		ui.initPixSmooth->setVisible(true);
		//default
		ui.initPixSmooth->setChecked(true);
		getStatusTool();

		//operation
		ui.pSmooth->setVisible(true);

		//ui.imageNextOperationGlob->setVisible(false);
		ui.UndoStrokeBGOnly->setVisible(false);

		ui.imageNextOperationGlob->setVisible(false);

 	}//else if(Step_Smooth == m_NextStep)
// 	{
// 		ui.imageNextOperationGlob->setText("Done!");
// 		QMessageBox::information(this, "Information", "Finished this pose!",QMessageBox::Ok);
// 		ui.middleResults->setCursor(Qt::CrossCursor);
// 		ui.middleResultsB->setCursor(Qt::CrossCursor);
// 	} 

}

void StopMotionRUI::videoNextClick()
{
	if (Step_BG == m_vNextStep)
	{
		//save background 
		//selectBackground();

		ui.isBackground->hide();

		ui.bgIdx2->setText("Selected Pairs!");
		ui.StepReminde->setText("Step-2: Please select the 'Pairs'!");

		ui.PairSize->show();

		ui.savePairs->show();
		//ui.NextOperation->show();

		ui.FirstFrame->hide();
		ui.SecondFrame->hide();
		ui.LeftCandIdx->show();
		ui.RightCandId->show();

		//show area curves
		ui.showAreaCurve->show();
		ui.VideoImgTranslation->show();
		showAreaCurves4Selection();

		//ui.curBGIdx->hide();

		ui.LeftCandIdx->setValue(m_backGroundIdx);

		ui.RightCandId->setValue(m_backGroundIdx);

		m_vNextStep = Step_Pair;

	}else if (Step_Pair == m_vNextStep)
	{
		//add pairs

		double curTime = (double)clock();

		m_pairSeTime.push_back(curTime);

		if (m_pairSeTime.size() == 2)
		{
			//write

		    double runTime = (m_pairSeTime[1] - m_pairSeTime[0])/(1000.);
			char optical_name[1024];

			sprintf(optical_name,"D:\\project-cityu\\Projects\\Projects\\STOP MOTION\\Videos-sample\\0904-videos\\%s-pairSelectionTime.txt",m_videofileName.c_str());

			FILE *out_timeRecord = fopen(optical_name,"ab");

			fprintf(out_timeRecord,"\n");

			fprintf(out_timeRecord, "The %.2d 'th pair selected time is %.2f s.\n",m_keyPairs.size(),runTime);

			fclose(out_timeRecord);

			/*m_pairSeTime.clear();*/
			m_pairSeTime.erase(m_pairSeTime.begin());
		}

		addPairs();

	}

}

void StopMotionRUI::showAreaCurves4Selection()
{
	QImage areaCurve;

	char curveName[1024];

	//sprintf(curveName,"D:\\project-cityu\\Projects\\Projects\\STOP MOTION\\Videos-sample\\0904-videos\\%s", m_videofileName.c_str());
	
	areaCurve.load(curveName);

	QImage lmask = QImage(areaCurve.width(), areaCurve.height(), 
		QImage::Format_ARGB32);

	lmask.fill(1);

	ui.showAreaCurve->setMaskImage(lmask);

	ui.showAreaCurve->setImage(&areaCurve,true);	

	ui.showAreaCurve->updateDisplayImage();
}

void StopMotionRUI::showcomImageRight()
{
	char comName[1024];

	Mat tempImg;

	sprintf(comName,".\\resCombine\\1012-combine-%.2d.jpg",m_curPairIdx);
 	tempImg = imread(comName);  

	Mat tt;

	cvtColor(tempImg,tt,CV_BGR2RGB);

	displayAnImage(1,tt); 
}
	

