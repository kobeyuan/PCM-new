#ifndef IMAGEDISPLAYWIDGET_H_
#define IMAGEDISPLAYWIDGET_H_

#include <QtGui>
#include <QLabel>
#include <QWidget>
#include <QMouseEvent>
#include <QPoint>	
#include <QWheelEvent>
#include <QImage>
#include <QPaintEvent>
#include "cv.h"
#include <vector>
#include "basic_types.h"

using namespace std;

class ImageShow :public QLabel
{
	Q_OBJECT
public:
	ImageShow(QWidget* parent = 0, Qt::WindowFlags flags = 0);
	~ImageShow(){};

public:
	enum BrushMode{BRUSH_GRAPHCUT = 0, DELMATCHING, BRUSH_ADDMATCHING};
	enum ToolMode{TOOL_SELECT = 0, TOOL_GLOADD,TOOL_GLOD, TOOL_BRUSH,
		TOOL_DEL,TOOL_ADD,TOOL_PSMOOTH,TOOL_TRANSLATION};

	//FOR NEW UI
	enum NewToolMode{TROI = 0, TGLOPTS, BRUSH, APAIR, TANSLATION};

public:
	void clearSltRegion(){m_isSelect = false;m_sltRect.clear();}
	void setImage(QImage* srcImage);
	void setImage(QImage* srcImage,bool isROI);
	void updateDisplayImage();
	void getRect(std::vector<cv::Rect>& desRect);
	bool setRect(cv::Rect& rect_);

	void cvRect2Qrect(const cv::Rect& oriRect, QRect& tgRect);
	QImage& getMaskImage(){return m_maskImage;}
	void setMaskImage(QImage& mask_){m_maskImage = mask_;}
	void setIsVideo(bool isVideo){m_isVideo = isVideo;}
	//void setSltRect(cv::Rect& rect_);
	void setRectZero();

	void setDelLine(QPoint& sP,QPoint& eP);

	void setVideoTransModel();

public:
	void updateDeformImage(cv::Mat& ori, cv::Point& minPs, int _heigh, int _width, MatrixXX& quadCoor, MatrixXX& textCoor);

public:
	void setStatusTool(bool isSelect, bool isBackG, bool isForeG, bool isAddMch);
	//add a new version
	void setStatusToolNew(bool isSelect, bool isGloDel, bool isBackG, bool isForeG, bool isAddMch,bool isTrans);

	//user study-1
	void setStatusToolNew2(bool isSelect, bool isGloAdd,bool isGloDel,
		bool isBackG, bool isForeG, bool isAddMch,bool isTrans, bool isPickSmooth);

	//for new ui
	void setToolStatus(bool isSelect, bool isGloAdd,bool isBrush, bool isAddPair, bool isTranslation);

	void drawMaskImage(QPoint& endPs);
	void drawMatching(vector<QPoint>& mthPs);
	void showMiddleImage();

	//set translation
	void setTranslation(ScalarType dx, ScalarType dy);
	void setZoom(ScalarType ratio);

	//reset
	void resetTransZoom();

	//autoselect
	void setVideoSelectMode();
signals:

	void changeRect();
	void showCoorAndRGB(const QPoint& point,const QRgb& rgb_);
	void bgfgMarkers();
	void deleteRect();
	void selectMatchingPs(vector<QPoint>&);

	//translate the image at the same time
	void meantimeTrans(float&,float&,ImageShow&);
	void bothZoom(float&,ImageShow&);

signals:
	void propaCoor(const QPoint& point);
	void delCurNearLine();
	void delGloCoor(const QPoint& point);

protected:
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent* event);
	void keyPressEvent(QKeyEvent* event);

private:
	QPoint m_strPt[2], m_endPt[2]; //draw rectangle
	QPoint m_curPt;//mouse positions
	QPoint m_lastPt;
	std::vector<cv::Rect> m_sltRect;
	cv::Rect m_sltRectLarge;

	QImage* m_srcImage;
	QImage  m_curImage;

	ScalarType m_transDis[2];
	ScalarType m_scaleDge;// scale of the image
	//ScalarType m_scaleDgeY;

	//delete line
	QPoint m_lineSP;
	QPoint m_lineEP;
	bool m_isDel;

private:
	QPainter* m_painter;

private:
	bool m_isSelect;
	bool m_selectROIs;
	bool m_isBrush;
	bool m_backBrush;
	bool m_foreBrush;
	bool m_isPainting;
	bool m_isVideo;
	bool m_isAddM;
	bool m_isTrans;

	QImage  m_maskImage;
	QPen m_strokePen;
	QPoint m_lastMousePoint;
	QPoint m_curMousePoint;
	QColor m_maskColor;
	IndexType m_brushSize;

	ToolMode m_curTool;
	BrushMode m_curBrush;
	//new ui
	NewToolMode m_newCurTool;

	//for matching points
private:
	vector<QPoint> m_matchingPs;
	vector<QPoint> m_drawmatchingPs;
private:
	bool m_isTransByMouse;//record the mouse move for transform the image.
	QPoint m_clickStrPoint;

private:
	bool m_isROI;
};


#endif