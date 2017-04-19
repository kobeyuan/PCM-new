#ifndef DRAW_TEXTURE_H
#define DRAW_TEXTURE_H

#include <QtWidgets>
#include <QGLWidget>
#include <QtOpenGL>

#include "QtGui/QImage"

#include "basic_types.h"
#include "cv.h"
class dTexture : public QGLWidget
{
	Q_OBJECT

public:

	dTexture(QWidget *parent = 0);
	~dTexture();


	public slots:
		void drawDeformedImg(cv::Mat&,int,int, MatrixXX&,MatrixXX& );
		void setInputData(MatrixXX& vertCoor, MatrixXX& _textCoor, QImage& inImg);
protected:
	void paintGL();
	void initializeGL();
	void resizeGL(int w, int h);

private:
	MatrixXX textCoor;
	MatrixXX quadCoor;
	QImage   textSource;
	//QColor clearColor;
	GLuint nFaces;

private:
	GLuint pbufferList;
	GLuint bbxTexture;
	QGLFramebufferObject *fbo;
	GLuint nW,nH;
	GLuint cubeTexture;

};

#endif