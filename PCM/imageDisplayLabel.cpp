#include "imageDisplayLabel.h"

ImageShow::ImageShow(QWidget *parent /*= 0*/, Qt::WindowFlags flags /*= 0*/):
	QLabel(parent)
{
	if (!!m_srcImage)
	{
		m_srcImage = nullptr;
	}

	this->setScaledContents(true);
	this->setMouseTracking(true);
	this->setFocusPolicy(Qt::StrongFocus);
	m_isSelect = false;
	m_backBrush = false;
	m_foreBrush = false;
	m_isAddM = false;

	m_brushSize = 5;
	m_isVideo = false;

	m_matchingPs.clear();

	m_isTransByMouse = false;

	m_isDel = false;

	m_scaleDge =  - 1.;

	m_transDis[0] = m_transDis[1] = 0.0f;

	m_isROI = false;
}

void ImageShow::paintEvent(QPaintEvent *event)
{

	if (!!m_painter)
	{
		m_painter = nullptr;
	}

	m_painter = new QPainter(this);

	m_painter->translate(m_transDis[0],m_transDis[1]);
	m_painter->scale(m_scaleDge,m_scaleDge);

	m_painter->drawImage(0,0,m_curImage);

	m_painter->setPen(QPen(Qt::red, 3, Qt::DashLine) );

	if (!m_isROI)
	{
	  m_painter->drawRect(QRect(m_strPt[0], m_endPt[0]) );
	}



	if (m_curTool == TOOL_BRUSH)
	{
		m_painter->setPen(QPen(Qt::green, 3, Qt::DashLine) );

		m_painter->drawEllipse(m_curPt,m_brushSize,m_brushSize);
	}

	if (m_isDel)
	{
		m_painter->setPen(QPen(Qt::blue, 3, Qt::DashLine) );
		m_painter->drawLine(m_lineSP,m_lineEP);
	}
}


void ImageShow::setImage(QImage* srcImage)
{

	this->m_srcImage = srcImage;

	if (m_maskImage.width() == 0 || m_maskImage.height() == 0)
	{
		m_maskImage = QImage(m_srcImage->width(), m_srcImage->height(), 
			QImage::Format_ARGB32);
	}

	if (m_scaleDge < 0)
	{
		m_scaleDge = (ScalarType)this->width()/srcImage->width();
	}

}

void ImageShow::setImage(QImage* srcImage,bool isROI)
{
	this->m_srcImage = srcImage;

	if (m_maskImage.width() == 0 || m_maskImage.height() == 0)
	{
		m_maskImage = QImage(m_srcImage->width(), m_srcImage->height(), 
			QImage::Format_ARGB32);

		m_maskImage.fill(1);
	}

	if (m_scaleDge < 0)
	{
		m_scaleDge = (ScalarType)this->width()/srcImage->width();
	}

	m_isROI = isROI;
}
void ImageShow::updateDisplayImage()
{
	assert(! m_srcImage->size().isEmpty() );

	m_curImage = * m_srcImage;

	QPainter painter(&m_curImage);
	painter.setBrush(QBrush(QColor::fromRgba(0x30ffffff)) );
	painter.setCompositionMode(QPainter::CompositionMode_Multiply);

    painter.drawImage(0,0,m_maskImage);

	if (!m_isSelect && !m_sltRect.empty())
	{
		 QRect curR;
		 cvRect2Qrect(m_sltRect[0],curR);

		 painter.setPen(QPen(Qt::red, 3, Qt::DashLine) );
		 painter.drawRect(curR);
	}

	painter.end();

	this->setPixmap(QPixmap::fromImage(m_curImage,Qt::AutoColor));

	this->update();
}

void ImageShow::showMiddleImage()
{
	assert(! m_srcImage->size().isEmpty() );

	m_curImage = * m_srcImage;

	this->setPixmap(QPixmap::fromImage(m_curImage));

	this->update();
}

void ImageShow::mousePressEvent(QMouseEvent *event)
{

 	if (event->button() == Qt::LeftButton)
 	{
		//m_curPt = event->pos();

		QPoint releasePos = event->pos();
		QPoint curPs;
		curPs.setX((releasePos.x() - m_transDis[0])/m_scaleDge );
		curPs.setY((releasePos.y() - m_transDis[1])/m_scaleDge );

		m_curPt = curPs;

		m_clickStrPoint = curPs;

		if (m_curTool == TOOL_SELECT)
		{
			if (m_selectROIs)
			{
				if (!m_isSelect)
				{
					m_strPt[0].setX(curPs.x());
					m_strPt[0].setY(curPs.y());

					//m_sltRect.push_back(cv::Rect(m_strPt[0].x(), m_strPt[0].y(),1,1) );

				}/*else
				{

					m_strPt[1].setX(curPs.x());
					m_strPt[1].setY(curPs.y());

					m_sltRect[1] = cv::Rect(m_strPt[1].x(), m_strPt[1].y(),1,1);
				}*/
			}
		}
		else if (m_curTool == TOOL_BRUSH)
		{
			m_isPainting = true;
		}

 	}//Qt::LeftButton 

 
	//for right button action

	if (event->button() == Qt::RightButton)
	{
		if(!m_sltRect.empty() && (m_curTool == TOOL_SELECT))
		{
			m_sltRect.clear();

			m_strPt[0] = m_endPt[0];

			clearSltRegion();

			emit deleteRect();

			emit changeRect();
		}

		// delete the lines directly
// 		if (m_isDel && (m_curTool == TOOL_DEL) )
// 		{
// 			emit delCurNearLine();
// 			m_isDel = false;
// 		}

	}//Qt:: rightbutton
 	this->update();	
}

//new for mouse move
void ImageShow::mouseMoveEvent(QMouseEvent *event)
{
 	QPoint releasePos = event->pos();

	QRgb curV(0);
	QPoint curPs;

	curPs.setX((releasePos.x() - m_transDis[0])/m_scaleDge );
	curPs.setY((releasePos.y() - m_transDis[1])/m_scaleDge );

	emit showCoorAndRGB(curPs,curV);


	if ((event->buttons() & Qt::LeftButton) )
	{
		if (m_curTool == TOOL_SELECT)
		{
			if (m_selectROIs)
			{
				if (!m_isSelect)
				{
					m_endPt[0].setX(curPs.x());
					m_endPt[0].setY(curPs.y());
				}
			}
		}
		else if(m_curTool == TOOL_BRUSH)
		{
			if (m_isPainting )
			{
               drawMaskImage(curPs);
			}
		}

	}
 
      m_curPt = curPs;

 	this->update();
}

//new version for releaseEvent

void ImageShow::mouseReleaseEvent(QMouseEvent *event)
{

	if (event->button() == Qt::LeftButton)
	{
		QPoint releasePos = event->pos();

		QPoint curPs;
		curPs.setX((releasePos.x() - m_transDis[0])/m_scaleDge );
		curPs.setY((releasePos.y() - m_transDis[1])/m_scaleDge );

		if (m_curTool == TOOL_SELECT)
		{

			if ( m_isSelect == false)
			{
				m_endPt[0].setX(curPs.x());
				m_endPt[0].setY(curPs.y());

				m_isSelect = ( abs(m_strPt[0].x() - m_endPt[0].x()) > 10 ? true:false);

				if (m_isSelect)
				{
					m_sltRect.push_back(cv::Rect(cv::Point(m_strPt[0].x(),m_strPt[0].y()),
						cv::Point(m_endPt[0].x(), m_endPt[0].y()) ));
					emit changeRect();

				}else
				{
					Loggger<<"Please select ROI again!.\n";
				}
			}

		}else if(TOOL_GLOADD == m_curTool)
		{
			if (m_isROI)
			{
				//global matching
				//global registration pick matching points
				if (!m_isTransByMouse)
				{
					emit propaCoor(curPs);//using this point to matching
					return;
				}

			}
		}else if(TOOL_GLOD == m_curTool)
		{
			emit delGloCoor(curPs);//delete the dots for global matching

		}else if(m_curTool == TOOL_BRUSH)
		{
			if (m_isPainting)
			{
				drawMaskImage(curPs);
				m_isPainting = false;
				emit bgfgMarkers();
			}

		}else if (m_curTool == TOOL_DEL)
		{
// 			if (m_isROI)
// 			{
// 				emit delGloCoor(curPs);//delete the dots for global matching
// 			}else
// 			{
				emit propaCoor(curPs);//using this point to delete a pair (lines!).
			//}

		}else if (m_curTool == TOOL_ADD)
		{
			m_matchingPs.push_back(curPs);

			emit propaCoor(curPs);
			//should draw this points.
			if (!(m_matchingPs.size()%2))
			{
				emit selectMatchingPs(m_matchingPs);
				m_matchingPs.clear();
			}

		}else if (m_curTool == TOOL_TRANSLATION)
		{
			ScalarType dx = curPs.x() - m_clickStrPoint.x();
			ScalarType dy = curPs.y() - m_clickStrPoint.y();

			m_transDis[0] += dx;
			m_transDis[1] += dy;

			emit meantimeTrans(dx,dy,*this);
		}else if (m_curTool == TOOL_PSMOOTH)
		{
			//select this point for poisson smooth
			emit propaCoor(curPs);
		}


	}//left button

	this->update();	
}

void ImageShow::wheelEvent(QWheelEvent * event)
{

	ScalarType r = qMax(0.1f,1.0f + event->delta() / 1000.0f);

	if (event->modifiers() == Qt::ControlModifier)
	{
		if (event->delta() < 0)
		{
			m_brushSize -= 1.f * r;

			if (m_brushSize < 1.)
			{
				m_brushSize = 1.f;
			}
		}else
		{
		    m_brushSize += 1.f * r;
		}

	}else
	{
		m_scaleDge = m_scaleDge * r;
		emit bothZoom(r,*this);

	}

	this->update();

}

void ImageShow::keyPressEvent(QKeyEvent* event)
{

	if (event->type() == QKeyEvent::KeyPress)
	{
		if (event->key() == Qt::Key_Left)
		{
			m_transDis[0] -= 10;
		}

		if (event->key() == Qt::Key_Right)
		{
			m_transDis[0] += 10;
		}

		if (event->key() == Qt::Key_Up)
		{
			m_transDis[1] -= 10;
		}

		if (event->key() == Qt::Key_Down)
		{
			m_transDis[1] += 10;
		}
	}

	if (!m_sltRect.empty() && (m_curTool == TOOL_SELECT) && event->key() == Qt::Key_Delete)
	{

		m_sltRect.clear();

		m_strPt[0] = m_endPt[0];

	    clearSltRegion();

		emit deleteRect();

		emit changeRect();

	}

	if (m_curTool == DELMATCHING && event->key() == Qt::Key_Delete)
	{
		if (m_isDel)
		{
			//delete the active one
			emit delCurNearLine();
			m_isDel = false;
 		}

	}

	this->update();
}

void ImageShow::drawMaskImage(QPoint& endPs)
{

	if (m_curTool == TOOL_BRUSH) // only for graph cut 
	{
		QPainter painter(&m_maskImage);

		painter.setPen(QPen(QBrush(m_maskColor),m_brushSize*2,
			Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));

		painter.drawLine(m_curPt,endPs);
		painter.translate(m_transDis[0],m_transDis[1]);
		painter.scale(m_scaleDge,m_scaleDge);

		m_curPt = endPs;

		updateDisplayImage();
	}

}

void ImageShow::drawMatching(vector<QPoint>& mthPs)
{
	if (m_maskImage.width() == 0 || m_maskImage.height() == 0)
	{
		m_maskImage = QImage(m_srcImage->width(), m_srcImage->height(), 
			QImage::Format_ARGB32);
	}

	QPainter painter(&m_maskImage);

	QColor pcolor(0,255,255);
	painter.setPen(QPen(QBrush(pcolor),m_brushSize*5,
		Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));

	for(IndexType i = 0; i < mthPs.size(); i += 2)
	{
    	painter.drawLine(mthPs[i],mthPs[i+1]);
	}

	painter.translate(m_transDis[0],m_transDis[1]);
	painter.scale(m_scaleDge,m_scaleDge);

	updateDisplayImage();

}

void ImageShow::getRect(std::vector<cv::Rect>& desRect)
{
IndexType vSize = m_sltRect.size();

	 for (IndexType i = 0; i< vSize; ++i )
	 {
		 desRect.push_back(m_sltRect[i]);
	 }

	//return desRect = m_sltRect;
}

bool ImageShow::setRect(cv::Rect& rect_)
{
	//only one selection region;

	m_sltRect.clear();

	m_sltRect.push_back(rect_) ;

	return true;
}

void ImageShow::cvRect2Qrect(const cv::Rect& oriRect, QRect& tgRect)
{
	//assert(oriRect.width > 0 && oriRect.height > 0);

	QPoint fPst(oriRect.x, oriRect.y);

	QPoint sPst(oriRect.x + oriRect.width, oriRect.y + oriRect.height);

	tgRect.setTopLeft(fPst);

	tgRect.setBottomRight(sPst);
}

void ImageShow::setStatusTool(bool isSelect, bool isBackG, bool isForeG, bool isAddMch)
{
	if (isSelect)
	{
	   m_selectROIs = true;
	   m_curTool = TOOL_SELECT;
	}else
	{
		m_isBrush = true;
		m_curTool = TOOL_BRUSH;

		if (isBackG)
		{
			m_curBrush = BRUSH_GRAPHCUT;
			m_backBrush = true;
			m_foreBrush = false;
			m_maskColor.setRgb(0,255,57);
			m_strokePen.setColor( (m_maskColor) );

		}else if(isForeG)
		{
			m_curBrush = DELMATCHING;
			m_foreBrush = true;
			m_backBrush = false;
			m_maskColor.setRgb(255,0,0);
			m_strokePen.setColor(m_maskColor);
		}else if(isAddMch)
		{
			m_isAddM = true;
			m_curBrush = BRUSH_ADDMATCHING;
			m_foreBrush = false;
			m_backBrush = false;
		}
	}
}

//add a new model for image translation
void ImageShow::setStatusToolNew(bool isSelect, bool isGloDel,bool isBackG,
								  bool isForeG, bool isAddMch,bool isTrans)
{
	if (isSelect)
	{
		m_selectROIs = true;
		m_curTool = TOOL_SELECT;

	}else if(isGloDel)
	{
		m_curTool = TOOL_GLOD;

	}else if (isBackG)
	{
		m_isBrush = true;
		m_curTool = TOOL_BRUSH;
		m_maskColor.setRgb(0,255,57);
		m_strokePen.setColor( (m_maskColor) );

	}else if (isForeG)
	{
		m_curTool = TOOL_DEL;
		m_foreBrush = true;
		m_backBrush = false;

	}else if (isAddMch)
	{
		m_curTool = TOOL_ADD;
		m_isAddM = true;
		m_curBrush = BRUSH_ADDMATCHING;
		m_foreBrush = false;
		m_backBrush = false;

	}else if (isTrans)
	{
		m_isTrans = true;
		m_curTool = TOOL_TRANSLATION;

	}
	 
}


void ImageShow::setStatusToolNew2(bool isSelect, bool isGloAdd,bool isGloDel,
								   bool isBackG, bool isForeG, bool isAddMch,bool isTrans, bool isPickSmooth)
{
	if (isSelect)
	{
		m_selectROIs = true;
		m_curTool = TOOL_SELECT;

	}else if(isGloAdd)
	{
      m_curTool = TOOL_GLOADD;
	}
	else if(isGloDel)
	{
		m_curTool = TOOL_GLOD;
	}else if (isBackG)
	{
		m_isBrush = true;
		m_curTool = TOOL_BRUSH;
		m_maskColor.setRgb(0,255,57);
		m_strokePen.setColor( (m_maskColor) );

	}else if (isForeG)
	{
		m_curTool = TOOL_DEL;
		m_foreBrush = true;
		m_backBrush = false;

	}else if (isAddMch)
	{
		m_curTool = TOOL_ADD;
		m_isAddM = true;
		m_curBrush = BRUSH_ADDMATCHING;
		m_foreBrush = false;
		m_backBrush = false;

	}else if (isTrans)
	{
		m_isTrans = true;
		m_curTool = TOOL_TRANSLATION;
	}else if (isPickSmooth)
	{
		m_curTool = TOOL_PSMOOTH;

	}

}


void ImageShow::setVideoSelectMode()
{
	m_selectROIs = true;
	m_curTool = TOOL_SELECT;
}

void ImageShow::setVideoTransModel()
{
	m_isTrans = true;
	m_curTool = TOOL_TRANSLATION;
}
void ImageShow::setRectZero()
{
	if (m_strPt[0]!= m_endPt[0])
	{
		m_endPt[0] = m_strPt[0];/* = QPoint();*/
	}

}

void ImageShow::updateDeformImage(cv::Mat& ori, cv::Point& minPs, int _heigh, int _width, MatrixXX& quadCoor, MatrixXX& textCoor)
{
	//draw the deformed quads only, for deformation processing.
	QImage tempImage = m_curImage.copy();

	QPainter painter(&tempImage);
	painter.setBrush(QBrush(QColor::fromRgba(0x30ffffff)) );
	//painter.setBrush(QBrush(QColor(255,0,0)) );
	painter.setCompositionMode(QPainter::CompositionMode_Multiply);
	
	QPoint qMinP;
	qMinP.setX(minPs.x);
	qMinP.setY(minPs.y);

   // painter.endNativePainting();

  	IndexType nQuad = quadCoor.rows()/4;
  
	ScalarType dx = qMinP.x() + m_strPt[0].x();
	ScalarType dy = qMinP.y() + m_strPt[0].y();

  	for (IndexType i = 0; i < nQuad; ++ i)
  	{
    	QPointF points[4];

		points[0] = QPointF(quadCoor(4*i,0) + dx,quadCoor(4*i,1) + dy);
		points[1] = QPointF(quadCoor(4*i + 1,0) + dx,quadCoor(4*i + 1,1) + dy);
		points[2] = QPointF(quadCoor(4*i + 2,0) + dx,quadCoor(4*i + 2,1) + dy);
		points[3] = QPointF(quadCoor(4*i + 3,0) + dx,quadCoor(4*i + 3,1) + dy);
		painter.drawPolygon(points,4);
  	}

	tempImage.save("2d_meshes.jpg");

	this->update();
}

void ImageShow::setDelLine(QPoint& sP,QPoint& eP)
{
	m_lineSP = sP;
	m_lineEP = eP;
	m_isDel = true; //make scale and translation

}

void ImageShow::setTranslation(ScalarType dx, ScalarType dy)
{
	m_transDis[0] += dx;
	m_transDis[1] += dy;

	this->update();
}

void ImageShow::setZoom(ScalarType ratio)
{
	m_scaleDge = m_scaleDge * ratio;

	this->update();
}

void ImageShow::resetTransZoom()
{
	m_scaleDge = -1;

	m_transDis[0] = m_transDis[1] = 0.;

}

//for new ui 
void ImageShow::setToolStatus( bool isSelect, bool isGloAdd,
							    bool isBrush, bool isAddPair, 
							    bool isTranslation)
{
	if (isSelect)
	{
		m_newCurTool = TROI;

	}else if(isGloAdd)
	{
		m_newCurTool = TGLOPTS;
	}
	else if(isBrush)
	{
		m_newCurTool = BRUSH;

	}else if (isAddPair)
	{
		m_newCurTool = APAIR;

	}else if (isTranslation)
	{
		m_newCurTool = TANSLATION;
	}
}