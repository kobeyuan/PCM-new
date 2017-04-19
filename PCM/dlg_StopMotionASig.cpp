#include "dlg_StopMotionASig.h"
#include "dlg_canvas.h"

#include "opencv2/video/video.hpp"
#include "opencv2/video/background_segm.hpp"

StopMotionASIG::StopMotionASIG() : m_video(SingleFrameSet::get_instance().getTotVideo() )
{
	ui.setupUi(this);
	//atui.setupUi(this);
	m_isInitImSeq = false;

	m_gloMatPsLeft.clear();
	m_gloMatPsRight.clear();
	m_locMatPsLeft.clear();
	m_locMatPsRight.clear();

	m_curPairIdx = 1;

	m_isGloMatching = false;
	m_isLocMatching = false;
}

StopMotionASIG::~StopMotionASIG()
{

}

void StopMotionASIG::initConnect()
{
	//draw rectangle for selecting ROI-- left-right at the same status
	connect(ui.CLeftWin, SIGNAL(changeRect()), this, SLOT(getROIsFromRect() ));
	connect(ui.CRightWin, SIGNAL(changeRect()), this, SLOT(getROIsFromRect() ));


	//for pair idx changed
	connect(ui.CpairIndex, SIGNAL(valueChanged(int) ), this, SLOT( showPairIdx(int) ));
	connect(ui.CCombineOperation, SIGNAL(clicked()), this, SLOT(showCurPairs()));

	//for tools: selection, translation,brush, add matching points
	connect(ui.CselectROI, SIGNAL(clicked()), this, SLOT(getToolStatus())); //roi selection
	connect(ui.CgloPick, SIGNAL(clicked()), this, SLOT(getToolStatus())); //assign global matching points
	connect(ui.CselectBackG, SIGNAL(clicked()), this, SLOT(getToolStatus())); //brush 
	connect(ui.CaddMatching, SIGNAL(clicked()), this, SLOT(getToolStatus())); //assign local matching points
	connect(ui.CImgTranslation, SIGNAL(clicked()), this, SLOT(getToolStatus())); //for translation
	connect(ui.CBackGroundBrush, SIGNAL(clicked()), this, SLOT(getToolStatus()));
	//next operation
 
	connect(ui.CimageComROI, SIGNAL(clicked()), this,SLOT(preHandRemovalAction())); //after rois selection

	connect(ui.CimageNextOperationGlob, SIGNAL(clicked()), this, SLOT(preGlobalMatching()));

	connect(ui.C4LocalMatching, SIGNAL(clicked()), this, SLOT(preLocalMatching()) );

	connect(ui.C4BackgroundReplace, SIGNAL(clicked()), this, SLOT(preBackgroundReplace()));

	connect(ui.CNextKeyFrame, SIGNAL(clicked()), this, SLOT(preNextKeyFrame()));
	//undo operation
	connect(ui.CStrokeUndo, SIGNAL(clicked()), this, SLOT(undoHandRemoval()) );

	connect(ui.CGlobalPtsUndo, SIGNAL(clicked()), this, SLOT(undoGloMatching()));

	connect(ui.CLocalPairUndo, SIGNAL(clicked()), this, SLOT(undoLocalMatching()));

	connect(ui.CBackGroundReplace, SIGNAL(clicked()), this, SLOT(undoBackgroundReplace()));

	//back¡¢cancel operation

	connect(ui.CBack2Select, SIGNAL(clicked()), this, SLOT(back2Select()));
	connect(ui.CBack2Handremoval, SIGNAL(clicked()), this, SLOT(back2HandRemoval()));
	connect(ui.CBack2Global, SIGNAL(clicked()), this, SLOT(back2GlobalMatching()));
	connect(ui.CBack2LocalMatching, SIGNAL(clicked()), this, SLOT(back2LocalMatching()));

	//get mask for the brushes
	connect(ui.CmiddleResults, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromLeftROI()));

	connect(ui.CmiddleResultsB, SIGNAL(bgfgMarkers()), this, SLOT(getMaskFromRightROI()));


	//obtain the global matching points
	connect(ui.CmiddleResults, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorLeft(const QPoint&)));

	connect(ui.CmiddleResultsB, SIGNAL(propaCoor(const QPoint&)), this, SLOT(obtainCoorRight(const QPoint&)));

	//obtain the local matching points

	connect(ui.CmiddleResults, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)));

	connect(ui.CmiddleResultsB, SIGNAL(selectMatchingPs(vector<QPoint>&)), this, SLOT(getMatchingPs(vector<QPoint>&)));

	//zoom at the same time
	connect(ui.CLeftWin, SIGNAL(bothZoom(float&, ImageLabel&)),
		this, SLOT(zoomSameTime(float&, ImageLabel&)));

	connect(ui.CRightWin, SIGNAL(bothZoom(float&, ImageLabel&)),
		this, SLOT(zoomSameTime(float&, ImageLabel&)));

	connect(ui.CmiddleResults, SIGNAL(bothZoom(float&, ImageLabel&)),
		this, SLOT(zoomSameTime(float&, ImageLabel&)));

	connect(ui.CmiddleResultsB, SIGNAL(bothZoom(float&, ImageLabel&)),
		this, SLOT(zoomSameTime(float&, ImageLabel&)));


	//new ui for system
	connect(ui.CLeftWin, SIGNAL(meantimeTrans(float&, float&, ImageLabel&)),
		this, SLOT(transImagesSameTime(float&, float&, ImageLabel&)));

	connect(ui.CRightWin, SIGNAL(meantimeTrans(float&, float&, ImageLabel&)),
		this, SLOT(transImagesSameTime(float&, float&, ImageLabel&)));

	connect(ui.CmiddleResults, SIGNAL(meantimeTrans(float&, float&, ImageLabel&)),
		this, SLOT(transImagesSameTime(float&, float&, ImageLabel&)));

	connect(ui.CmiddleResultsB, SIGNAL(meantimeTrans(float&, float&, ImageLabel&)),
		this, SLOT(transImagesSameTime(float&, float&, ImageLabel&)));


}

void StopMotionASIG::initBaseInfo()
{

}

void StopMotionASIG::initImaLab()
{
	m_seqInList.push_back(ui.CImgSeqs1);
	m_seqInList.push_back(ui.CImgSeqs2);
	m_seqInList.push_back(ui.CImgSeqs3);
	m_seqInList.push_back(ui.CImgSeqs4);
	m_seqInList.push_back(ui.CImgSeqs5);
	m_seqInList.push_back(ui.CImgSeqs6);
	m_seqInList.push_back(ui.CImgSeqs7);
	m_seqInList.push_back(ui.CImgSeqs8);
	m_seqInList.push_back(ui.CImgSeqs9);
}
void StopMotionASIG::showSeqinList()
{
	int pSize = m_keyPairs.size();

	for (int i= 0; i < 9 && i < pSize; ++ i)
	{
		IndexType showIdA = get_aframe_from_key(m_keyPairs[i].second);
		SingleFrame& imgs = SingleFrameSet::get_instance()[showIdA];
		Mat imgMat = *imgs.body;

		displayListImg(imgMat, i);
	}
}

void StopMotionASIG::displayListImg(Mat& img, int listId)
{

	if (listId<0 || listId>=9)
	{
		return;
	}

	Mat temp;
	cvtColor(img, temp, CV_RGB2BGR);

	char filename[1024];
	sprintf(filename, ".\\tempImages\\imglist%d.jpg", listId);

	imwrite(filename,temp);

	m_combineSubQImg.load(filename);


	QImage commask = QImage(m_combineSubQImg.width(), m_combineSubQImg.height(),
		QImage::Format_ARGB32);

	commask.fill(1);

	m_seqInList[listId]->setVisible(true);

	m_seqInList[listId]->setMaskImage(commask);

	m_seqInList[listId]->setImage(&m_combineSubQImg);

	m_seqInList[listId]->updateDisplayImage();
}

//draw pair for completion

void StopMotionASIG::drawKeyPair(IndexType pIdx)
{
	if (!m_isInitImSeq)
	{
	  initPairFromImageSeq();
	}

// 	if (pIdx >= 0 && pIdx < m_keyPairs.size() )
// 	{
		// get the first image of the pair,means that the idx is the small one.
		m_curPairIdx = pIdx;

		IndexType showIdA = get_aframe_from_key(m_keyPairs[pIdx].second);

		IndexType showIdB = get_bframe_from_key(m_keyPairs[pIdx].second);

		m_showPairIdx[0] = showIdA;

		m_showPairIdx[1] = showIdB;

		if (showIdB < 0 || showIdB >= m_totFrames)
		{
			Loggger << "Out of size.\n";
			return;
		}

		SingleFrameSet& set_ = SingleFrameSet::get_instance();

		SingleFrame& aImage = set_[showIdA];
		SingleFrame& bImage = set_[showIdB];

		drawKeyFrame(showIdA, false);
		drawKeyFrame(showIdB, true);
	//}


}

void StopMotionASIG::drawKeyFrame(IndexType fIdx, bool isRight)
{
	//
	if (!m_isVideo)
	{
		//Loggger<<" Not a video.\n";

		SingleFrameSet& set_ = SingleFrameSet::get_instance();

		SingleFrame& curImage = set_[fIdx];

// 		Rect slt1;
// 		if (!curImage.sltRect.empty())
// 		{
// 			slt1 = curImage.sltRect[0]; //update, still have a rectangle
// 		}

		Mat imgMat = *set_[fIdx].body;

		if (isRight)
		{
		  m_maskRightImg = QImage(m_curShowRightImg.width(), m_curShowRightImg.height(), 
						QImage::Format_ARGB32);
		  m_maskRightImg.fill(1);

		  mat2Qimage(imgMat,m_curShowRightImg);
 
		  drawRightWin(m_maskRightImg, m_curShowRightImg);


		}else
		{

			mat2Qimage(imgMat,m_curShowLeftImg); 

 		    m_maskLeftImg = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(), 
 					QImage::Format_ARGB32);
 			m_maskLeftImg.fill(1);

			drawLeftWin(m_maskLeftImg, m_curShowLeftImg);

		}

	}
}


void StopMotionASIG::drawLeftWin(QImage& mask, QImage& input)
{
	if (mask.width() <= 0)
	{
		mask = QImage(m_curShowLeftImg.width(), m_curShowLeftImg.height(),
			QImage::Format_ARGB32);

		mask.fill(1);
	}

	ui.CLeftWin->setImage(&input);

	ui.CLeftWin->setMaskImage(mask);

	ui.CLeftWin->updateDisplayImage();
}

void StopMotionASIG::drawRightWin(QImage& mask, QImage& input)
{
	if (mask.width()<= 0)
	{
		mask = QImage(m_curShowRightImg.width(), m_curShowRightImg.height(),
		QImage::Format_ARGB32);

     	mask.fill(1);
	}

	ui.CRightWin->setImage(&input);

	ui.CRightWin->setMaskImage(mask);

	ui.CRightWin->updateDisplayImage();

}

void StopMotionASIG::mat2Qimage(const Mat& srcMat, QImage& desQImage)
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

bool StopMotionASIG::initPairFromImageSeq()
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

	//initialize the stop motion algorithm class

    m_sMotion = new StopMotion(m_keyPairs);
	//m_sMotion->setShowPairId(m_showPairIdx);

	connect(m_sMotion, SIGNAL(transferImg(Mat&, Point&, int, int, MatrixXX&, MatrixXX&)), this,
		SLOT(drawEditImg(Mat&, Point&, int, int, MatrixXX&, MatrixXX&)));

	return true;
}

void StopMotionASIG::showButtonsOne()
{
	//tools
	//default for selection 
	ui.groupBox_3->setVisible(true);
	ui.CselectROI->setVisible(true);
	ui.CselectROI->setChecked(true);
	ui.CgloPick->setVisible(false);
	ui.CselectBackG->setVisible(false);
	ui.CaddMatching->setVisible(false);
	ui.CBackGroundBrush->setVisible(false);

	//titles
	ui.CsubWinLeft->setVisible(false);
	ui.CsubWinRight->setVisible(false);
	ui.CsubWinMiddle->setVisible(false);
	ui.CLArrow->setVisible(false);
	ui.CRArrow->setVisible(false);
	ui.CcoorMouse->setVisible(false);

	//functions
	  //back operation
	ui.CBack2Select->setVisible(false);
	ui.CBack2Handremoval->setVisible(false);
	ui.CBack2Global->setVisible(false);
	ui.CBack2LocalMatching->setVisible(false);
	  
	  //cancel operation
	ui.CLocalPairUndo->setVisible(false);
	ui.CStrokeUndo->setVisible(false);
	ui.CGlobalPtsUndo->setVisible(false);
	ui.CRectSelectUndo->setVisible(false);
	ui.CBackGroundReplace->setVisible(false);

	  //next operation
	ui.CimageNextOperationGlob->setVisible(false);
	ui.CimageComROI->setVisible(false);
	ui.C4LocalMatching->setVisible(false);
	ui.C4BackgroundReplace->setVisible(false);
	ui.CNextKeyFrame->setVisible(false);

	//emit the tool status
	getToolStatus();

}

void StopMotionASIG::showButtonAfterROISelected()
{
	//ui.CRectSelectUndo->setVisible(true);//undo
	ui.CimageComROI->setVisible(true); //next operation

}

void StopMotionASIG::getToolStatus()
{
	bool isROIselected = ui.CselectROI->isChecked();

	bool isgloClick = ui.CgloPick->isChecked();

	bool isPaint = ui.CselectBackG->isChecked();

	bool isAddMatchingPs = ui.CaddMatching->isChecked();

	bool isTrans = ui.CImgTranslation->isChecked();

	bool isBGReplace = ui.CBackGroundBrush->isChecked();

	ui.CLeftWin->setToolStatus(isROIselected,isgloClick,isPaint,isAddMatchingPs,isTrans,isBGReplace);

	ui.CRightWin->setToolStatus(isROIselected,isgloClick,isPaint,isAddMatchingPs,isTrans, isBGReplace);

	ui.CmiddleResults->setToolStatus(isROIselected,isgloClick,isPaint,isAddMatchingPs,isTrans, isBGReplace);

	ui.CmiddleResultsB->setToolStatus(isROIselected,isgloClick,isPaint,isAddMatchingPs,isTrans, isBGReplace);

	ui.CmiddleResultsC->setToolStatus(isROIselected,isgloClick,isPaint,isAddMatchingPs,isTrans, isBGReplace);

	//ui.C4BackgroundReplace->setToolStatus(isROIselected, isgloClick, isPaint, isAddMatchingPs, isTrans, isBGReplace);
}

void StopMotionASIG::getROIsFromRect()
{
	//assign values
	m_sltRect.clear();

	ui.CLeftWin->getRect(m_sltRect);

	/*ui.CRightWin->getRect(m_sltRect);*/

	ui.CRightWin->setRect(m_sltRect[0]);

	m_sMotion->setRect(m_sltRect);

	// draw the rois at the same time
	SingleFrameSet& set_ = SingleFrameSet::get_instance();

	set_[m_showPairIdx[0] ].setSecRect(m_sltRect);

	set_[m_showPairIdx[1] ].setSecRect(m_sltRect);

	drawKeyFrame(m_showPairIdx[1], true);

	initalROIsShow();

	showButtonAfterROISelected();

	//finish the first step
	m_stepCtrl = Step_ROIS;

}

//large one than current watched
void StopMotionASIG::showPairIdx(int idx)
{
	if (!m_isInitImSeq)
	{
		m_isInitImSeq = initPairFromImageSeq();
	}

	if (idx >= 1 && idx <= m_keyPairs.size() )
	{
		// get the first image of the pair,means that the idx is the small one.
		m_curPairIdx = idx - 1;

		IndexType showIdA = get_aframe_from_key(m_keyPairs[m_curPairIdx].second);

		IndexType showIdB = get_bframe_from_key(m_keyPairs[m_curPairIdx].second);

		m_showPairIdx[0] = showIdA;

		m_showPairIdx[1] = showIdB;

	}else
	{
		Loggger<<"Out Of Size, please initialize the key-pairs.\n";

		QMessageBox::information(this, "Information", "Out of size!",QMessageBox::Ok);

		return;
	}

	m_sMotion->setShowPairId(m_showPairIdx);
}

void StopMotionASIG::showCurPairs()
{
	if (m_curPairIdx>= m_keyPairs.size())
	{
		Loggger << "Out of size!";
		return;
	}

	drawKeyPair(m_curPairIdx);
}

void StopMotionASIG::initalROIsShow()
{
	Mat roiL,roiR,tempL,tempR, signLHand;

	if (m_sltRect.size() <= 0)
	{
		Loggger<<"Please select the ROIs.\n";
		return;
	}

	m_sMotion->copyROIFromKey(m_showPairIdx[0],roiL);

	m_sMotion->copyROIFromKey(m_showPairIdx[1],roiR);

	roiL.copyTo(m_leftROI);

	roiR.copyTo(m_rightROI);

	displayLeftSubWin(m_leftROI);

	displayRightSubWin(m_rightROI);

	Mat comImg,tComImg;

	translucentImages(m_leftROI,m_rightROI,0.7,comImg);
	
	comImg.copyTo(m_comImgSelect);

	displayMiddleResults(comImg);

	//show the buttons in the platform

}

void StopMotionASIG::translucentImages(Mat& srIMg, Mat& tgImg, 
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

//display three sub-windows

//left sub window
void StopMotionASIG::displayLeftSubWin(Mat& leftWin)
{
	Mat temp;	          
	cvtColor(leftWin,temp,CV_RGB2BGR);

	//draw global matching points

	//QImage tmpImg;
	if (!m_gloMatPsLeft.empty() && ui.CgloPick->isChecked())
	{
		//tmpImg = m_leftSubWinQImg.copy(0, 0, m_leftSubWinQImg.width(), m_leftSubWinQImg.height());

		//draw circles

		for (int i = 0; i < m_gloMatPsLeft.size(); ++i)
		{
			QPoint curP = m_gloMatPsLeft[i];
			Point2f tempp(curP.x(), curP.y());
			Scalar color = Scalar(255, 255, 0);
			circle(temp, tempp, 4, color);
		}
	}

	if (!m_seamPts.empty()&&! ui.CBackGroundBrush->isChecked())
	{
		for (IndexType i = 0; i < m_seamPts.size(); ++ i)
		{
			Point2f tempp = m_seamPts[i];
			Scalar color = Scalar(255, 0, 0);
			circle(temp, tempp, 2, color);
		}

	}


	imwrite(".\\tempImages\\leftROI.jpg",temp);

	m_leftSubWinQImg.load(".\\tempImages\\leftROI.jpg");


	QImage lmask;

	if (ui.CselectBackG->isChecked())
	{
		if (!m_leftmaskImgStack.empty())
		{
			lmask = m_leftmaskImgStack.top();
		}
		else
		{
			QImage lmask = QImage(m_leftSubWinQImg.width(), m_leftSubWinQImg.height(),
				QImage::Format_ARGB32);

			lmask.fill(1);
		}
	}



	if (ui.CBackGroundBrush->isChecked())
	{
		if (!m_leftmaskImgStackBG.empty())
		{
			lmask = m_leftmaskImgStackBG.top();
		}
		else
		{
			QImage lmask = QImage(m_leftSubWinQImg.width(), m_leftSubWinQImg.height(),
				QImage::Format_ARGB32);

			lmask.fill(1);
		}
	}

	ui.CmiddleResults->setVisible(true);

	ui.CmiddleResults->setMaskImage(lmask);

	ui.CmiddleResults->setImage(&m_leftSubWinQImg);

	ui.CmiddleResults->updateDisplayImage();
}


//right sub window
void StopMotionASIG::displayRightSubWin(Mat& rightWin)
{
	Mat temp;	          
	cvtColor(rightWin,temp,CV_RGB2BGR);


	//draw matching points if possible

	if (!m_gloMatPsRight.empty() )
	{
		//draw circles

		for (int i = 0; i < m_gloMatPsRight.size(); ++i)
		{
			QPoint curP = m_gloMatPsRight[i];
			Point2f tempp(curP.x(), curP.y());
			Scalar color = Scalar(255, 0, 0);
			circle(temp, tempp, 4, color);
		}
	}

    //draw the seam
	if (!m_seamPts.empty() && ui.CaddMatching->isChecked() &&!ui.CBackGroundBrush->isChecked())
	{
		for (IndexType i = 0; i < m_seamPts.size(); ++i)
		{
			Point2f tempp = m_seamPts[i];
			Scalar color = Scalar(255, 0, 0);
			circle(temp, tempp, 2, color);
		}

	}

	imwrite(".\\tempImages\\rightROI.jpg",temp);

	m_rightSubWinQImg.load(".\\tempImages\\rightROI.jpg");

	QImage rmask;

	if (ui.CselectBackG->isChecked())
	{
		if (!m_rightmaskImgStack.empty())
		{
			rmask = m_rightmaskImgStack.top();
		}
		else
		{
			rmask = QImage(m_rightSubWinQImg.width(), m_rightSubWinQImg.height(),
				QImage::Format_ARGB32);

			rmask.fill(1);
		}
	}


	if (ui.CBackGroundBrush->isChecked())
	{
		if (!m_rightmaskImgStackBG.empty())
		{
			rmask = m_rightmaskImgStackBG.top();
		}
		else
		{
			rmask = QImage(m_rightSubWinQImg.width(), m_rightSubWinQImg.height(),
				QImage::Format_ARGB32);

			rmask.fill(1);
		}
	}

	ui.CmiddleResultsB->setVisible(true);

	ui.CmiddleResultsB->setMaskImage(rmask);

	ui.CmiddleResultsB->setImage(&m_rightSubWinQImg);

	ui.CmiddleResultsB->updateDisplayImage();
}

// combine sub window
void StopMotionASIG::displayMiddleResults(Mat& mRes)
{
	Mat temp;	          
	cvtColor(mRes,temp,CV_RGB2BGR);

	imwrite(".\\tempImages\\combineRes.jpg",temp);

	m_combineSubQImg.load(".\\tempImages\\combineRes.jpg");


	QImage commask = QImage(m_combineSubQImg.width(), m_combineSubQImg.height(), 
		QImage::Format_ARGB32);

	commask.fill(1);

	ui.CmiddleResultsC->setVisible(true);

	ui.CmiddleResultsC->setMaskImage(commask);

	ui.CmiddleResultsC->setImage(&m_combineSubQImg);

	ui.CmiddleResultsC->updateDisplayImage();

}

void StopMotionASIG::preHandRemovalAction()
{
	//show the  brush tool and 'back' button
    //hide the composited image

	//buttons
	ui.CimageComROI->setVisible(false);
	ui.CBack2Handremoval->setVisible(true);
	ui.CRectSelectUndo->setVisible(false);


	//tools
	ui.CselectROI->setVisible(false);
	ui.CgloPick-> setVisible(false);
	ui.CselectBackG->setVisible(true);
	ui.CselectBackG->setChecked(true);
	getToolStatus();
	//should active status

	//others
	ui.CmiddleResults->setCursor(QCursor(Qt::ArrowCursor));
	ui.CmiddleResultsB->setCursor(QCursor(Qt::ArrowCursor));

	ui.CmiddleResultsC->setVisible(false);

	//back
	ui.CBack2Handremoval->setVisible(false);
}


void StopMotionASIG::getMaskFromLeftROI()
{
	m_maskSubLImg = ui.CmiddleResults->getMaskImage();

	IndexType width = m_maskSubLImg.width();
	IndexType height = m_maskSubLImg.height();

	if (ui.CselectBackG->isChecked())
	{
		m_leftmaskImgStack.push(m_maskSubLImg);

		m_markLeftLabels.resize(height, width);

		QRgb bgColor;
		QRgb fgColor;

		bgColor = qRgb(bgMask[0], bgMask[1], bgMask[2]);
		fgColor = qRgb(fgMask[0], fgMask[1], fgMask[2]);

		for (IndexType hIdx = 0; hIdx < height; ++hIdx)
		{
			for (IndexType wIdx = 0; wIdx < width; ++wIdx)
			{
				QRgb curColor = m_maskSubLImg.pixel(wIdx, hIdx);

				if (curColor == bgColor)
				{
					m_markLeftLabels(hIdx, wIdx) = GC_BGD;
				}
				else if (curColor == fgColor)
				{
					m_markLeftLabels(hIdx, wIdx) = GC_FGD;

				}
				else
				{
					m_markLeftLabels(hIdx, wIdx) = GC_PR_BGD;
				}
			}
		}

		// make left as the 0-label
		m_isRightMask = isMaskFromRight();

		//run graph cut to removal hand region
		combineTwoRois();
	}

	if (ui.CBackGroundBrush->isChecked())
	{
		m_leftmaskImgStackBG.push(m_maskSubLImg);

		m_markLeftLabelsBG.resize(height, width);

		QRgb bgColor;
		QRgb fgColor;

		bgColor = qRgb(bgMask[0], bgMask[1], bgMask[2]);
		fgColor = qRgb(fgMask[0], fgMask[1], fgMask[2]);

		for (IndexType hIdx = 0; hIdx < height; ++hIdx)
		{
			for (IndexType wIdx = 0; wIdx < width; ++wIdx)
			{
				QRgb curColor = m_maskSubLImg.pixel(wIdx, hIdx);

				if (curColor == bgColor)
				{
					m_markLeftLabelsBG(hIdx, wIdx) = GC_BGD;
				}
				else if (curColor == fgColor)
				{
					m_markLeftLabelsBG(hIdx, wIdx) = GC_FGD;

				}
				else
				{
					m_markLeftLabelsBG(hIdx, wIdx) = GC_PR_BGD;
				}
			}
		}

		// make left as the 0-label
		m_isRightMask = isMaskFromRight();

		//run graph cut to removal hand region--combine two roi and background

		completedwithBG();
		ui.CBackGroundReplace->setVisible(true);
	}


}

void StopMotionASIG::getMaskFromRightROI()
{
	m_maskSubRImg = ui.CmiddleResultsB->getMaskImage();

	//assert()
	IndexType width = m_maskSubRImg.width();
	IndexType height = m_maskSubRImg.height();


	//assert(width == oriW && height == oriH);
	if (ui.CselectBackG->isChecked())
	{
        m_rightmaskImgStack.push(m_maskSubRImg);

		m_markRightLabels.resize(height, width);

		QRgb bgColor;  //();
		QRgb fgColor;

		bgColor = qRgb(bgMask[0], bgMask[1], bgMask[2]);
		fgColor = qRgb(fgMask[0], fgMask[1], fgMask[2]);

		for (IndexType hIdx = 0; hIdx < height; ++hIdx)
		{
			for (IndexType wIdx = 0; wIdx < width; ++wIdx)
			{
				QRgb curColor = m_maskSubRImg.pixel(wIdx, hIdx);

				if (curColor == bgColor)
				{
					m_markRightLabels(hIdx, wIdx) = GC_BGD;
				}
				else if (curColor == fgColor)
				{
					m_markRightLabels(hIdx, wIdx) = GC_FGD;

				}
				else
				{
					m_markRightLabels(hIdx, wIdx) = GC_PR_BGD;
				}
			}
		}

		//run graph cut
		combineTwoRois();

	}

	if (ui.CBackGroundBrush->isChecked())
	{
		m_rightmaskImgStackBG.push(m_maskSubRImg);

		m_markRightLabelsBG.resize(height, width);

		QRgb bgColor;  //();
		QRgb fgColor;

		bgColor = qRgb(bgMask[0], bgMask[1], bgMask[2]);
		fgColor = qRgb(fgMask[0], fgMask[1], fgMask[2]);

		for (IndexType hIdx = 0; hIdx < height; ++hIdx)
		{
			for (IndexType wIdx = 0; wIdx < width; ++wIdx)
			{
				QRgb curColor = m_maskSubRImg.pixel(wIdx, hIdx);

				if (curColor == bgColor)
				{
					m_markRightLabelsBG(hIdx, wIdx) = GC_BGD;
				}
				else if (curColor == fgColor)
				{
					m_markRightLabelsBG(hIdx, wIdx) = GC_FGD;

				}
				else
				{
					m_markRightLabelsBG(hIdx, wIdx) = GC_PR_BGD;
				}
			}
		}

		//run graph cut
		completedwithBG();
		ui.CBackGroundReplace->setVisible(true);
	}



}

bool StopMotionASIG::isMaskFromRight()
{
	bool isRightMask = true;

	IndexType nLMask = m_leftmaskImgStack.size();
	IndexType nRMask = m_rightmaskImgStack.size();

	if (nLMask >= nRMask)
	{
		isRightMask = false;
	}
	//
	return isRightMask;
}

void StopMotionASIG::combineTwoRois()
{

	handRemoveByGCut(m_leftROI, m_rightROI);
  
	m_sMotion->setShowPairId(m_showPairIdx);

	Mat outPutImg, finalImg;


	if (m_isRightMask)
	{
		m_sMotion->setInputImg(m_leftROI, m_rightROI, m_cutLabels);

		//m_sMotion->imageGenerate(m_leftROI, m_rightROI, m_cutLabels, outPutImg);

		m_sMotion->imageGenerateAll(m_leftROI, m_rightROI, m_cutLabels, outPutImg,finalImg);

	}else
	{
		m_sMotion->setInputImg(m_rightROI, m_leftROI, m_cutLabels);

		//m_sMotion->imageGenerate(m_rightROI, m_leftROI, m_cutLabels, outPutImg);

		m_sMotion->imageGenerateAll(m_rightROI, m_leftROI, m_cutLabels, outPutImg,finalImg);
	}

	//show the middle result


	displayMiddleResults(outPutImg);
	outPutImg.copyTo(m_combineROI);

	QImage mask, fOut;
	mat2Qimage(finalImg, fOut);

	drawRightWin(mask, fOut);

	//show the global combined image on the right window

	//show the 'back' button
	//show the 'next' button
	if (m_leftmaskImgStack.empty() && m_rightmaskImgStack.empty())
	{
		Loggger << "Masks empty.\n";
		ui.CStrokeUndo->setVisible(false);
		ui.CimageNextOperationGlob->setVisible(false);

	}else
	{
		ui.CStrokeUndo->setVisible(true);
		ui.CimageNextOperationGlob->setVisible(true);
		ui.CBack2Select->setVisible(true);
	}


	//show the back button
	
}


void StopMotionASIG::combineImages(Mat& leftImg, Mat& rightImg)
{
	Mat outPutImg, finalImg;

	if (m_cutLabels.rows() <= 0)
	{
		return;
	}

	if (m_isRightMask)
	{
		m_sMotion->imageGenerateAll(leftImg, rightImg, m_cutLabels, outPutImg, finalImg);

		//can using extend labels
	}
	else
	{
		m_sMotion->imageGenerateAll(rightImg, leftImg, m_cutLabels, outPutImg, finalImg);
	}

	//record the composite image
	//m_combineImgHis.push_back(&outPutImg);

	//show the middle result
	displayMiddleResults(outPutImg);

	m_combineROI = Mat();
	outPutImg.copyTo(m_combineROI);

	QImage mask, fOut;

	mat2Qimage(finalImg, fOut);

	m_gloCombineImg = fOut.copy();

	drawRightWin(mask, fOut);

	//show the global combined image on the right window

	//show the 'back' button
	//show the 'next' button

// 	if (m_leftmaskImgStack.empty() && m_rightmaskImgStack.empty())
// 	{
// 		//Loggger << "Masks empty.\n";
// 		ui.CStrokeUndo->setVisible(false);
// 		ui.CimageNextOperationGlob->setVisible(false);
// 
// 	}
// 	else
// 	{
// 		ui.CStrokeUndo->setVisible(true);
// 		ui.CimageNextOperationGlob->setVisible(true);
// 	}
}

void StopMotionASIG::handRemoveByGCut(Mat& leftImg, Mat& rightImg)
{
	IndexType height = leftImg.rows;
	IndexType width = rightImg.cols;

	if (m_sltRect.size() <= 0)
	{
		Loggger << "please select the ROIs again!\n";
		return;
	}

	vector<Mat> imgData;
	MatrixXXi maskFromStk;

	m_isRightMask = isMaskFromRight();

	if (m_isRightMask)
	{
		imgData.push_back(rightImg);
		imgData.push_back(leftImg);


 		if (!getLabesFromMaskROI(m_maskSubRImg, maskFromStk))
 		{
 			//Loggger<<"Please brush the regions in ROI.\n";
 
 			//QMessageBox::information(this, "Information", "Please brush the regions in ROI!", QMessageBox::Ok);
 			//return;
 		}

	}
	else
	{
 		imgData.push_back(leftImg);
 		imgData.push_back(rightImg);

 		if (!getLabesFromMaskROI(m_maskSubLImg, maskFromStk))
 		{
 			//QMessageBox::information(this, "Information", "Please brush the regions in ROI!", QMessageBox::Ok);
 			//return;
 		}

	}

	MatrixXXi initLabels;
	initLabels.resize(height, width);
	initLabels.setZero();

	m_dialCut = new DialCut(width, height, imgData, maskFromStk, initLabels);

	m_dialCut->compute();

	MatrixXXi& resSeg = m_dialCut->getLabels();

	_gcutLabels.clear();
	_gcutLabels.push_back(resSeg);//for the final p smooth.


	if (ui.CselectBackG->isChecked())
	{
	   m_cutLabels = resSeg;
	}

	if (ui.CBackGroundBrush->isChecked())
	{
		m_cutLabelsBG = resSeg;
	}

	//m_cutLabelsBG = resSeg;

	//getSeamFromGraphcut();

}


void StopMotionASIG::completedwithBG()
{
	handRemoveByGCut(m_combineROI, m_bgROI);
	Mat outPutImg, finalImg;


	if (m_isRightMask)
	{
		m_sMotion->setInputImg(m_combineROI, m_bgROI, m_cutLabelsBG/*m_cutLabels*/);

		//m_sMotion->imageGenerate(m_leftROI, m_rightROI, m_cutLabels, outPutImg);

		m_sMotion->imageGenerateAll(m_combineROI, m_bgROI, m_cutLabelsBG/*m_cutLabels*/, outPutImg, finalImg);

	}
	else
	{
		m_sMotion->setInputImg(m_bgROI, m_combineROI, m_cutLabelsBG/*m_cutLabels*/);

		//m_sMotion->imageGenerate(m_rightROI, m_leftROI, m_cutLabels, outPutImg);

		m_sMotion->imageGenerateAll(m_bgROI, m_combineROI, m_cutLabelsBG/*m_cutLabels*/, outPutImg, finalImg);
	}

	displayMiddleResults(outPutImg);
	//outPutImg.copyTo(m_combineROI);

	QImage mask, fOut;
	mat2Qimage(finalImg, fOut);

	if (m_isRightMask)
	{
		drawLeftWin(mask, fOut);
		
	}else
	{
    	drawRightWin(mask, fOut);
	}

	displayListImg(finalImg, m_curPairIdx);

	ui.CNextKeyFrame->setVisible(true);

}

bool StopMotionASIG::getLabesFromMaskROI(QImage& _maskIMg, MatrixXXi& _labels)
{
	if (m_sltRect.size() <= 0)
	{
		Loggger << "Please select the ROIs!.\n";
		return false;
	}

	bool isValid = false;

	IndexType width = _maskIMg.width();

	IndexType height = _maskIMg.height();

	//assert(width>0 && height>0);


	if (width <= 0 || height <= 0)
	{
		Loggger << "ROIs are empty.\n";
		return false;
	}

	_labels.resize(height, width);

	_labels.setZero();

	QRgb bgColor;  //();
	QRgb fgColor;

	bgColor = qRgb(bgMask[0], bgMask[1], bgMask[2]);
	fgColor = qRgb(fgMask[0], fgMask[1], fgMask[2]);

	for (IndexType hIdx = 0; hIdx < height; ++hIdx)
	{
		for (IndexType wIdx = 0; wIdx < width; ++wIdx)
		{
			QRgb curColor = _maskIMg.pixel(wIdx, hIdx);

			if (curColor == bgColor)
			{
				_labels(hIdx, wIdx) = GC_FGD;
				isValid = true;
			}
			else if (curColor == fgColor)
			{
				_labels(hIdx, wIdx) = GC_FGD;
				isValid = true;
			}
			else
			{
				_labels(hIdx, wIdx) = GC_BGD;
				//isValid = true;
			}
		}
	}

	return isValid;
}

void StopMotionASIG::undoHandRemoval()
{
	if (m_isRightMask)
	{
		if (!m_rightmaskImgStack.empty())
		{
			m_rightmaskImgStack.pop();
		}

		if (m_rightmaskImgStack.empty())
		{
			ui.CStrokeUndo->setVisible(false);

			m_maskSubRImg = QImage(m_leftROI.cols, m_leftROI.rows,
				QImage::Format_ARGB32);

			m_maskSubRImg.fill(0);

		}else
		{
			m_maskSubRImg = m_rightmaskImgStack.top();
		}


	}else
	{
		if (!m_leftmaskImgStack.empty())
		{
			m_leftmaskImgStack.pop();
		}

		if (m_leftmaskImgStack.empty())
		{
			ui.CStrokeUndo->setVisible(false);
			m_maskSubLImg = QImage(m_leftROI.cols,m_leftROI.rows,
				QImage::Format_ARGB32);

			m_maskSubLImg.fill(0);
		}
		else
		{
			m_maskSubLImg = m_leftmaskImgStack.top();
		}
	}

	combineTwoRois();

	if (m_isRightMask)
	{
	  displayRightSubWin(m_rightROI);
	}else
	{
	  displayLeftSubWin(m_leftROI);
	}

}

void StopMotionASIG::undoBackgroundReplace()
{
	if (m_isRightMask)
	{
		if (!m_rightmaskImgStackBG.empty())
		{
			m_rightmaskImgStackBG.pop();
		}

		if (m_rightmaskImgStackBG.empty())
		{
			ui.CStrokeUndo->setVisible(false);

			m_maskSubRImg = QImage(m_leftROI.cols, m_leftROI.rows,
				QImage::Format_ARGB32);

			m_maskSubRImg.fill(0);

			ui.CBackGroundReplace->setVisible(false);

		}
		else
		{
			m_maskSubRImg = m_rightmaskImgStackBG.top();
		}

	}
	else
	{
		if (!m_leftmaskImgStackBG.empty())
		{
			m_leftmaskImgStackBG.pop();
		}

		if (m_leftmaskImgStackBG.empty())
		{
			ui.CStrokeUndo->setVisible(false);
			m_maskSubLImg = QImage(m_leftROI.cols, m_leftROI.rows,
				QImage::Format_ARGB32);

			m_maskSubLImg.fill(0);

			ui.CBackGroundReplace->setVisible(false);
		}
		else
		{
			m_maskSubLImg = m_leftmaskImgStackBG.top();
		}
	}

	completedwithBG();

	if (m_isRightMask)
	{
		displayRightSubWin(m_combineROI);
	}
	else
	{
		displayLeftSubWin(m_combineROI);
	}

}

void StopMotionASIG::undoGloMatching()
{
	//delete the global matching points

   if(!m_gloMatPsLeft.empty() && !m_gloMatPsRight.empty())
   {
	   m_gloMatPsLeft.pop_back();
	   m_gloMatPsRight.pop_back();

	   if (!m_gloMatPsLeft.empty() && !m_gloMatPsRight.empty())
	   {
		   //deformation
		   Mat dImg;
		   //globalDeformation(m_leftROI, m_rightROI, dImg);
		   //
		   globalDeformationSym(m_leftROI, m_rightROI, dImg);
		   dImg.copyTo(m_gloDeformROI);

		   //show in the right
		   Mat comImg;
		   translucentImages(dImg, m_rightROI, 0.5, comImg);

		   displayMiddleResults(comImg);

		   //show the combined results
		   combineImages(dImg, m_rightROI);

	   }else
	   {
		   combineImages(m_leftROI, m_rightROI);
	   }

	   //update the sub window
	   displayLeftSubWin(m_leftROI);
	   displayRightSubWin(m_rightROI);

	   if (m_gloMatPsLeft.empty() && m_gloMatPsRight.empty())
	   {
		   ui.CGlobalPtsUndo->setVisible(false);
		   //ui.C4LocalMatching->setVisible(false);
	   }
   }
}

void StopMotionASIG::undoLocalMatching()
{

    //judge the matching points
	if (m_locMatPsLeft.size()>0 && m_locMatPsRight.size()>0)
	{
		m_locMatPsLeft.pop_back();
		m_locMatPsRight.pop_back();

		if (m_locMatPsRight.size() > 0 && m_locMatPsLeft.size() > 0) //do not really play the local deformation 
		{
			//play local deformation
			localDeformation();
			//show the current local matching result--combined results!

			if (m_isRightMask)
			{
				combineImages(m_locDeformROI, m_rightROI);
				//displayRightSubWin(m_combineROI);
			}
			else
			{
				combineImages(m_leftROI, m_locDeformROI);
				//displayLeftSubWin(m_combineROI);
			}
	

			drawSubWin4LocalMatching();

			displayMiddleResults(m_combineROI);

			QImage mask;
			drawRightWin(mask, m_gloCombineImg);

// 			m_combineImgHis.pop_back();
// 
// 			Mat curCom = *m_combineImgHis.rbegin();
// 
// 			if (m_isRightMask)
// 			{
// 				displayRightSubWin(curCom);
// 			}
// 			else
// 			{
// 				displayLeftSubWin(curCom);
// 			}

			if (!ui.C4BackgroundReplace->isVisible())
			{
				ui.C4BackgroundReplace->setVisible(true);
			}

 		}else
 		{
 			//back to the original version
 			ui.CLocalPairUndo->setVisible(false);
 			//ui.C4BackgroundReplace->setVisible(false);
 			//
 			Mat curCom = *m_combineImgHis.begin();
 
 			if (m_isRightMask)
 			{
 				displayRightSubWin(curCom);
 			}
 			else
 			{
 				displayLeftSubWin(curCom);
 			}
 
 			//displayMiddleResults(m_combineROI);
			displayMiddleResults(curCom);

 			QImage mask;
 			//drawRightWin(mask, m_gloCombineImg);
			drawRightWin(mask, m_iniglo4LocComImg);
 
 		}
	} 
	else
	{
	  //back to the original version
		ui.CLocalPairUndo->setVisible(false);
		ui.C4BackgroundReplace->setVisible(false);
		//
		Mat curCom = *m_combineImgHis.begin();

		if (m_isRightMask)
		{
			displayRightSubWin(curCom);
		}
		else
		{
			displayLeftSubWin(curCom);
		}

		displayMiddleResults(m_combineROI);

		QImage mask;

		drawRightWin(mask, m_gloCombineImg);

	}

}


void StopMotionASIG::preGlobalMatching()
{
	//tools
	ui.CselectBackG->setVisible(false);
	ui.CgloPick->setVisible(true);
	ui.CaddMatching->setVisible(false);

	ui.CImgTranslation->setChecked(true);
	getToolStatus();

	//buttons
	ui.C4LocalMatching->setVisible(true);
	ui.CimageNextOperationGlob->setVisible(false);
	ui.CStrokeUndo->setVisible(false);

	//back
	ui.CBack2Select->setVisible(false);
	ui.CBack2Handremoval->setVisible(true);
	ui.CBack2Global->setVisible(false);
	ui.CLocalPairUndo->setVisible(false);
	ui.C4BackgroundReplace->setVisible(false);
	ui.CNextKeyFrame->setVisible(false);

    //clear the current parameters
	m_gloMatPsLeft.clear();
	m_gloMatPsRight.clear();

}

void StopMotionASIG::preLocalMatching()
{
	//tools
	ui.CgloPick->setVisible(false);
	ui.CaddMatching->setVisible(true);

	ui.CImgTranslation->setChecked(true);
	getToolStatus();

	//buttons

	//back 
	ui.CGlobalPtsUndo->setVisible(false);
	ui.CBack2Handremoval->setVisible(false);
	ui.CBack2Global->setVisible(true);

	//next button
	ui.C4LocalMatching->setVisible(false);

	//ignore this local matching step
	ui.C4BackgroundReplace->setVisible(true);

	getSeamFromGraphcut();

	//show the result of above steps
	if (m_isRightMask)
	{
		if (!m_isGloMatching)
		{
			m_leftROI.copyTo(m_gloDeformROI);
		}

	    combineImages(m_gloDeformROI, m_rightROI);
		displayRightSubWin(m_combineROI);
	}else
	{
		if (!m_isGloMatching)
		{
			m_rightROI.copyTo(m_gloDeformROI);
		}
		combineImages(m_leftROI, m_gloDeformROI);
	    displayLeftSubWin(m_combineROI);
	}
	
	//record the combined image--should the first push_back operation of this vector!

	m_combineImgHis.clear();

	Mat curCom;
	m_combineROI.copyTo(curCom);
	m_combineImgHis.push_back(curCom);

	m_iniglo4LocComImg = m_gloCombineImg.copy();

}

void StopMotionASIG::setting4back2localMatching()
{
	//tools
	ui.CgloPick->setVisible(false);
	ui.CaddMatching->setVisible(true);

	ui.CImgTranslation->setChecked(true);
	getToolStatus();

	//buttons

	//back 
	ui.CGlobalPtsUndo->setVisible(false);
	ui.CBack2Handremoval->setVisible(false);
	ui.CBack2Global->setVisible(true);

	//next button
	ui.C4LocalMatching->setVisible(false);

	//show the result of above steps
	if (m_isRightMask)
	{
		combineImages(m_gloDeformROI, m_rightROI);
		displayRightSubWin(m_combineROI);
		displayLeftSubWin(m_leftROI);
	}
	else
	{
		combineImages(m_leftROI, m_gloDeformROI);
		displayLeftSubWin(m_combineROI);
		displayRightSubWin(m_rightROI);
	}

	//record the combined image--should the first push_back operation of this vector!

	m_combineImgHis.clear();

	Mat curCom;
	m_combineROI.copyTo(curCom);
	m_combineImgHis.push_back(curCom);
}

void StopMotionASIG::preBackgroundReplace()
{
	//tools
	ui.CgloPick->setVisible(false);
	ui.CaddMatching->setVisible(false);

	ui.CBackGroundBrush->setVisible(true);
	ui.CBackGroundBrush->setChecked(true);


	getToolStatus();

	//undo
	ui.CLocalPairUndo->setVisible(false);

	//back 
	ui.CGlobalPtsUndo->setVisible(false);
	ui.CBack2Handremoval->setVisible(false);
	ui.CBack2Global->setVisible(false);
	ui.CBack2LocalMatching->setVisible(true);

	//next button

	ui.C4BackgroundReplace->setVisible(false);

	if (!m_isLocMatching)
	{
	  m_gloDeformROI.copyTo(m_locDeformROI);
	}
	//
	//show the background image
	//mainwindow and sub window
	Mat bg;// , subbg;

	(*(SingleFrameSet::get_instance().getBackGround().body)).copyTo(bg);
	
	if (m_sltRect.size()>=1)
	{
	  bg(m_sltRect[0]).copyTo(m_bgROI);

	}else
	{
		return;
	}


	QImage mask;
	if (m_isRightMask)
	{
	   mat2Qimage(bg, m_curShowLeftImg);
	   drawRightWin(mask, m_curShowLeftImg);
	   displayLeftSubWin(m_bgROI);
	   displayRightSubWin(m_combineROI);

	}else
	{
		mat2Qimage(bg, m_curShowRightImg);
		drawRightWin(mask, m_curShowRightImg);
		displayRightSubWin(m_bgROI);
		displayLeftSubWin(m_combineROI);
	}
}

void StopMotionASIG::preNextKeyFrame()
{

  //clear all the temp data and parameters
	m_leftROI = Mat();
	m_rightROI = Mat();
	m_combineROI = Mat();
	m_combineImgHis.clear();
	m_comImgSelect = Mat();

	m_gloDeformROI = Mat();
	m_locDeformROI = Mat();
	m_bgROI = Mat();
	m_isGloMatching = false;
	m_isLocMatching = false;

	if (!!m_dialCut)
	{
		delete m_dialCut;
		m_dialCut == NULL;
	}

	m_seamPts.clear();
	m_gloMatPsLeft.clear();
	m_gloMatPsRight.clear();

	m_locMatPsRight.clear();
	m_locMatPsLeft.clear();

	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	while (!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.empty();
	}


	while (!m_leftmaskImgStackBG.empty())
	{
		m_leftmaskImgStackBG.pop();
	}

	while (!m_rightmaskImgStackBG.empty())
	{
		m_rightmaskImgStackBG.empty();
	}



	//disable the window

	ui.CmiddleResults->setVisible(false);
	ui.CmiddleResultsB->setVisible(false);
	ui.CmiddleResultsC->setVisible(false);
	ui.CNextKeyFrame->setVisible(false);

  //tools

	ui.CBackGroundReplace->setVisible(false);
	ui.CBack2LocalMatching->setVisible(false);
	ui.CBackGroundBrush->setVisible(false);

	ui.CselectROI->setVisible(true);
	ui.CselectROI->setChecked(true);

	getToolStatus();

	m_curPairIdx += 1;

	if (m_curPairIdx >= m_keyPairs.size())
	{
		Loggger << "Completed the process!.\n";
		return;
	}

	ui.CpairIndex->setValue(m_curPairIdx + 1);

	drawKeyPair(m_curPairIdx);

	return;
}

void StopMotionASIG::back2Select()
{
	showButtonsOne();

	m_leftROI = Mat();
	m_rightROI = Mat();
	m_combineROI = Mat();

	ui.CmiddleResults->setVisible(false);
	ui.CmiddleResultsB->setVisible(false);
	ui.CmiddleResultsC->setVisible(false);

	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	while (!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.pop();
	}

	_gcutLabels.clear();
	m_cutLabels.setZero(0,0);
	if (!!m_dialCut)
	{
	  delete m_dialCut;
	  m_dialCut = NULL;
	}


	//show the original image

	m_sltRect.clear();
	m_sMotion->setRect(m_sltRect);
	
	cv::Rect crect;
	ui.CRightWin->setRect(crect);
	ui.CLeftWin->setRect(crect);

	drawKeyPair(m_curPairIdx);


	//drawKeyFrame(m_showPairIdx[1], true);
	//drawKeyFrame(m_showPairIdx[0], false);
}


void StopMotionASIG::back2HandRemoval()
{
	//tools and buttons
	preHandRemovalAction();

	ui.CBack2Select->setVisible(true);
	ui.CBack2Handremoval->setVisible(false);
	ui.C4LocalMatching->setVisible(false);
	ui.CGlobalPtsUndo->setVisible(false);

	//parameters

	m_gloMatPsLeft.clear();
	m_gloMatPsRight.clear();

	//ready to hand removal operation
	while (!m_leftmaskImgStack.empty())
	{
		m_leftmaskImgStack.pop();
	}

	while (!m_rightmaskImgStack.empty())
	{
		m_rightmaskImgStack.pop();
	}

	_gcutLabels.clear();
	m_cutLabels.setZero(0, 0);

	if (!!m_dialCut)
	{
		delete m_dialCut;
		m_dialCut = NULL;
	}

	//windows display
	displayLeftSubWin(m_leftROI);
	displayRightSubWin(m_rightROI);
	displayMiddleResults(m_comImgSelect);


}


void StopMotionASIG::back2GlobalMatching()
{
	//tools and buttons
	preGlobalMatching();

	//parameters
	m_seamPts.clear();
	m_locMatPsLeft.clear();
	m_locMatPsRight.clear();
	m_locDeformROI = Mat();

	//ready to global matching operation
	
	//windows display of the hand removal results
	displayLeftSubWin(m_leftROI);
	displayRightSubWin(m_rightROI);
	displayMiddleResults(m_combineROI);

}


void StopMotionASIG::back2LocalMatching()
{
	//preLocalMatching();
	setting4back2localMatching();
	//m_seamPts.clear();
	m_locMatPsLeft.clear();
	m_locMatPsRight.clear();
	m_locDeformROI = Mat();

	ui.CBack2LocalMatching->setVisible(false);
	ui.CBackGroundBrush->setVisible(false);
	//ui.CBackGroundReplace->setVisible(false);
	ui.CNextKeyFrame->setVisible(false);

    while(!m_leftmaskImgStackBG.empty())
	{
		m_leftmaskImgStackBG.pop();
	}

    while (!m_rightmaskImgStackBG.empty())
    {
		m_rightmaskImgStackBG.pop();
    }
}

void StopMotionASIG::obtainCoorLeft(const QPoint& pos)
{
	if (ui.CgloPick->isChecked())
	{
		int lSize = m_gloMatPsLeft.size();

		int rSize = m_gloMatPsRight.size();

		m_selectPointLeft.setX(pos.x());
		m_selectPointLeft.setY(pos.y());

		m_gloMatPsLeft.push_back(m_selectPointLeft);

		int mSize = m_gloMatPsLeft.size();

		//show the selected matching points
		while (!m_leftmaskImgStack.empty())
		{
			m_leftmaskImgStack.pop();
		}

		displayLeftSubWin(m_leftROI);

		//play matching processing with the same points
		Mat dImg;
		//play matching processing with the same points
		if (m_gloMatPsLeft.size() == m_gloMatPsRight.size())
		{
			//globalDeformation(m_leftROI, m_rightROI, dImg);
			//
			globalDeformationSym(m_leftROI, m_rightROI, dImg);

			dImg.copyTo(m_gloDeformROI);

			//show in the right
			Mat comImg;
			translucentImages(dImg, m_rightROI, 0.5, comImg);

			displayMiddleResults(comImg);

			ui.CGlobalPtsUndo->setVisible(true);
			ui.C4LocalMatching->setVisible(true);

			combineImages(dImg, m_rightROI);

		}



	}
}
void StopMotionASIG::obtainCoorRight(const QPoint& pos)
{
	if (ui.CgloPick->isChecked())
	{
		int lSize = m_gloMatPsLeft.size();

		int rSize = m_gloMatPsRight.size();

		m_selectPointRight.setX(pos.x());
		m_selectPointRight.setY(pos.y());

		m_gloMatPsRight.push_back(m_selectPointRight);

		//show the selected points
		while (!m_rightmaskImgStack.empty())
		{
			m_rightmaskImgStack.pop();
		}

		displayRightSubWin(m_rightROI);

		Mat dImg;
		//play matching processing with the same points
		if (m_gloMatPsLeft.size() == m_gloMatPsRight.size())
		{
			//globalDeformation(m_leftROI, m_rightROI, dImg);
			//

			globalDeformationSym(m_leftROI, m_rightROI, dImg);
			dImg.copyTo(m_gloDeformROI);

			//show in the right
			Mat comImg;//only the mask region
			translucentImages(dImg, m_rightROI, 0.5, comImg);

			displayMiddleResults(comImg);

			combineImages(dImg, m_rightROI);

			ui.CGlobalPtsUndo->setVisible(true);

			ui.C4LocalMatching->setVisible(true);
		}


	}
}


void StopMotionASIG::getMatchingPs(vector<QPoint>& mPs)
{
	if (ui.CaddMatching->isChecked())
	{
		if (mPs.size() % 2)
		{
			Loggger << "The selection of matching is not correct!.\n";
			return;
		}

		for (IndexType i = 0; i < mPs.size(); i += 2)
		{
			QPoint sP = mPs[i];
			QPoint tP = mPs[i + 1];

			//should find the closest point around the seam, m_seamPts.
			Point scvp, tcvp;

			bool isSOk = findClosestPointOnSeam(sP, scvp);
			bool isTOk = findClosestPointOnSeam(tP, tcvp);

			ScalarType dis = sqrt((scvp.x - tcvp.x)* (scvp.x - tcvp.x) +
				(scvp.y - tcvp.y)*(scvp.y - tcvp.y));
			if (dis < 2.)
			{
				QMessageBox::information(this, "Information", "It's too short!", QMessageBox::Ok);
				continue;
			}

			m_locMatPsLeft.push_back(scvp);
			m_locMatPsRight.push_back(tcvp);

		}
	}

	//draw the arrows and selected points
	drawSubWin4LocalMatching();

	if (!ui.CLocalPairUndo->isVisible())
	{
		ui.CLocalPairUndo->setVisible(true);
	}

	//play local deformation
	localDeformation();
	//show the current local matching result--combined results!

	if (m_isRightMask)
	{
		combineImages(m_locDeformROI,m_rightROI);
	}
	else
	{
		combineImages(m_leftROI, m_locDeformROI);
	}

	//m_combineImgHis.push_back(m_combineROI);

// 	for (int i = 0; i < m_combineImgHis.size(); ++i)
// 	{
// 		char filename[1024];
// 		sprintf(filename, "combineimage%d", i);
// 
// 		Mat currec;
// 		cvtColor(m_combineImgHis[i], currec, CV_RGB2BGR);
// 		imshow(filename, currec);
// 
// 	}

	if (!ui.C4BackgroundReplace->isVisible())
	{
		ui.C4BackgroundReplace->setVisible(true);
	}
}


bool StopMotionASIG::findClosestPointOnSeam(QPoint& curP, Point& cloPs)
{
	IndexType nSize = m_seamPts.size();

	//assert(nSize > 0);

	if (nSize <= 0)
	{
		Loggger << "finding closed points makes error.\n";
		return false;
	}

	vector<ScalarType> dis;
	vector<IndexType> pIdx;
	dis.clear();
	pIdx.clear();

	for (IndexType i = 0; i < nSize; ++i)
	{
		ScalarType dx = curP.x() - (m_seamPts[i].x);
		ScalarType dy = curP.y() - (m_seamPts[i].y);

		ScalarType idis = sqrt(dx * dx + dy * dy);
		dis.push_back(idis);
		pIdx.push_back(i);
	}

	bubleSort(dis, pIdx, nSize);

	cloPs = m_seamPts[pIdx[0]];// select the closest point for mathcing 

	return true;
}


void StopMotionASIG::bubleSort(vector<ScalarType>& oriData, vector<IndexType>& labels, IndexType lSize)
{
	ScalarType temp;
	IndexType labelTemp = 0;
	bool flag = false;
	for (int i = 0; i < lSize; i++)
	{
		flag = true;
		for (int j = 0; j < lSize - i - 1; j++)
		{
			if (oriData[j] > oriData[j + 1])
			{
				temp = oriData[j];
				labelTemp = labels[j];

				oriData[j] = oriData[j + 1];
				labels[j] = labels[j + 1];

				oriData[j + 1] = temp;
				labels[j + 1] = labelTemp;

				flag = false;
			}
		}
		if (flag) break;
	}
}

bool StopMotionASIG::globalDeformationSym(Mat& roiL, Mat& roiR, Mat& outImg)
{
	m_isGloMatching = true;

	vector<CvPoint2D32f> lMatchingPs, rMatchingPs;

	//global matching
	//m_sMotion->getMatchingPoints(roiL,roiR,lMatchingPs,rMatchingPs);

	//interaction by user

	IndexType lmSize = m_gloMatPsLeft.size();

	IndexType rmSize = m_gloMatPsRight.size();

	if (lmSize != rmSize)
	{
		//QMessageBox::information(this, "Information", "Matching points are not equal!", QMessageBox::Ok);

		m_gloMatPsLeft.clear();
		m_gloMatPsRight.clear();
		return false;
	}

	for (IndexType i = 0; i < lmSize; ++i)
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

	if (lgSize == 0 || lgSize != rgSize)
	{
		QMessageBox::information(this, "Information", "Matching points are not equal!", QMessageBox::Ok);
		return false;
	}


	//preserving lines
	//m_sMotion->gloDeformRoiwithLines(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	//without preserving lines
	if (m_isRightMask)
	{
	    m_sMotion->gloDeformRoi(roiL, roiR, lMatchingPs, rMatchingPs, outImg);
	}else
	{
		m_sMotion->gloDeformRoi( roiR,roiL, rMatchingPs, lMatchingPs, outImg);
	}


	//global transformation while preserving lines

	//imwrite("gloDefRes.jpg",outImg);

	//for undo operation
	//m_gloMatPsLeft.clear();

	//m_gloMatPsRight.clear();

	return true;

}


bool StopMotionASIG::globalDeformation(Mat& roiL, Mat& roiR, Mat& outImg)
{

	vector<CvPoint2D32f> lMatchingPs, rMatchingPs;

	//global matching
	//m_sMotion->getMatchingPoints(roiL,roiR,lMatchingPs,rMatchingPs);

	//interaction by user

	IndexType lmSize = m_gloMatPsLeft.size();

	IndexType rmSize = m_gloMatPsRight.size();

	if (lmSize != rmSize)
	{
		QMessageBox::information(this, "Information", "Matching points are not equal!", QMessageBox::Ok);

		m_gloMatPsLeft.clear();
		m_gloMatPsRight.clear();
		return false;
	}


	for (IndexType i = 0; i < lmSize; ++i)
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

	if (lgSize == 0 || lgSize != rgSize)
	{
		QMessageBox::information(this, "Information", "Matching points are not equal!", QMessageBox::Ok);
		return false;
	}


	//preserving lines
	//m_sMotion->gloDeformRoiwithLines(roiL,roiR,lMatchingPs,rMatchingPs,outImg);

	//without preserving lines
	m_sMotion->gloDeformRoi(roiL, roiR, lMatchingPs, rMatchingPs, outImg);

	//global transformation while preserving lines

	//imwrite("gloDefRes.jpg",outImg);

	//for undo operation
	//m_gloMatPsLeft.clear();

	//m_gloMatPsRight.clear();

	return true;
}

void StopMotionASIG::drawEditImg(Mat& oriImg, Point& minPs, int height, int width, MatrixXX& quadCoor, MatrixXX& textCoor)
{


	//draw meshes
	//ui.outputVideo->updateDeformImage(oriImg, minPs, height, width, quadCoor, textCoor);

	//draw deformed images
	m_canvas->getTextureCanvas()->drawDeformedImg(oriImg, height, width, quadCoor, textCoor);
}

void StopMotionASIG::drawSubWin4LocalMatching()
{

	// what's the paint image?  'm_combineROI'

	if (m_combineROI.empty())
	{
		return;
	}

	Mat curImg;
	//m_combineROI.copyTo(curImg);//please

	if(m_combineImgHis.size()>=1)
	{
		(m_combineImgHis[0]).copyTo(curImg);
	}else
	{
		return;
	}

	//draw the seam on bg
	Scalar color(255, 0, 0);
    //draw matching lines

	RNG rng(12345);

// 	if ()
// 	{
// 	}

	for (IndexType i = 0; i < m_locMatPsLeft.size(); ++i)
	{
		Point2f curS = m_locMatPsLeft[i];

		Point2f curT = m_locMatPsRight[i];

		circle(curImg, curS, .1, color);

		circle(curImg, curT, .1, color);

		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		arrowedLine(curImg, (curS), (curT), color, 3, 8, 0, 0.4);
	}

	if (m_isRightMask)
	{
		displayRightSubWin(curImg);
	}
	else
	{
		displayLeftSubWin(curImg);
	}

}

void StopMotionASIG::getSeamFromGraphcut()
{
	if (m_cutLabels.rows() != m_leftROI.rows)
	{
		return;
	}

	vector<Point2f> ptsList;

	MatrixXXi expLabels;

	m_seamPts.clear();

	m_sMotion->findSeamExpand(m_cutLabels, ptsList, expLabels);

	m_seamPts = ptsList;

	m_cutExLabels = expLabels;

	m_totSeamPsLeft = ptsList.size();

	m_totSeamPsright = ptsList.size();

}

void StopMotionASIG::localDeformation()
{
	m_isLocMatching = true;
	//judge the matching points

	if (m_locMatPsLeft.size() <= 0 || m_locMatPsRight.size()<=0 || (m_locMatPsLeft.size() != m_locMatPsRight.size()))
	{
		return;
	}


	vector<CvPoint2D32f> lmtemp, rmtemp;
	lmtemp.clear();
	rmtemp.clear();

	lmtemp = m_locMatPsLeft;
	rmtemp = m_locMatPsRight;


	Mat dOutImg;
	//bool isLines = true;
	bool isLines = false;

	//m_sMotion->setInputImg(m_gcNHandROI,m_rightROI,m_cutLabels); // ? dleftROI correct!

	m_sMotion->setInputImg(m_gloDeformROI, m_rightROI, m_cutLabels);

	if (isLines)
	{
		if (m_isRightMask)
		{
			m_sMotion->deformLinesPoints(m_gloDeformROI, m_rightROI, m_seamPts,
				lmtemp, rmtemp, dOutImg);
		}
		else
		{
			m_sMotion->deformLinesPoints(m_rightROI, m_gloDeformROI, m_seamPts,
				lmtemp, rmtemp, dOutImg);
		}

	}
	else
	{
		if (m_isRightMask)
		{
			m_sMotion->deformRoiWithMathcing(m_gloDeformROI, m_rightROI, m_seamPts,
				lmtemp, rmtemp, dOutImg);
		}
		else
		{
			m_sMotion->deformRoiWithMathcing(m_rightROI, m_gloDeformROI, m_seamPts,
				lmtemp, rmtemp, dOutImg);
		}
	}

	dOutImg.copyTo(m_locDeformROI);
}

void StopMotionASIG::zoomSameTime(float& ration, ImageLabel& _panter)
{
	if (ration <= 0.)
	{
		return;
	}

	if (&(_panter) == (ui.CLeftWin))
	{
		ui.CRightWin->setZoom(ration);
	}

	if (&(_panter) == (ui.CRightWin))
	{
		ui.CLeftWin->setZoom(ration);
	}

	if (&(_panter) == (ui.CmiddleResults))
	{
		ui.CmiddleResultsB->setZoom(ration);
	}

	if (&(_panter) == (ui.CmiddleResultsB))
	{
		ui.CmiddleResults->setZoom(ration);
	}

}

void StopMotionASIG::transImagesSameTime(float& dx, float& dy, ImageLabel& _panter)
{

	if (dx == 0 && dy == 0)
	{
		return;
	}

	if (&(_panter) == (ui.CLeftWin))
	{
		ui.CRightWin->setTranslation(dx, dy);
	}

	if (&(_panter) == (ui.CRightWin))
	{
		ui.CLeftWin->setTranslation(dx, dy);
	}

	if (&(_panter) == (ui.CmiddleResults))
	{
		ui.CmiddleResultsB->setTranslation(dx, dy);
	}

	if (&(_panter) == (ui.CmiddleResultsB))
	{
		ui.CmiddleResults->setTranslation(dx, dy);
	}
}
