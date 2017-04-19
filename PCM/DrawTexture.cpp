#include <QtWidgets>
#include <QtOpenGL>
//#include "taucsInclude/GL/glut.h"
#include <GL/GLU.h>
#include "DrawTexture.h"
#include "cv.h"
#include "highgui.h"
using namespace cv;
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

#define GL_PI 3.1415926
#define GL_RADIUX  0.2f

#define checkImageWidth	 64
#define checkImageHeight	64
static GLubyte checkImage[checkImageHeight][checkImageWidth][4];
static GLubyte otherImage[checkImageHeight][checkImageWidth][4];
static GLuint texName[2];
static GLint cubeTextureArray[][2] = {
	{0, 0}, {1, 0}, {1, 1}, {0, 1},
	{0, 0}, {0, 1}, {1, 1}, {1, 0},
	{0, 0}, {1, 0}, {1, 1}, {0, 1},
	{1, 0}, {0, 0}, {0, 1}, {1, 1},
	{0, 0}, {1, 0}, {1, 1}, {0, 1},
	{1, 0}, {0, 0}, {0, 1}, {1, 1}
};
static GLint faceArray[][2] = {
	{1, -1}, {1, 1}, {-1, 1}, {-1, -1}
};
static GLubyte colorArray[][4] = {
	{102, 176, 54, 255},
	{81, 141, 41, 255},
	{62, 108, 32, 255},
	{45, 79, 23, 255}
};

void makeCheckImage(void);
dTexture::dTexture(QWidget *parent): QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
	makeCurrent();
	nH = 1;
	nW = 1;
}

dTexture::~dTexture()
{
	delete fbo;
}

void dTexture::setInputData(MatrixXX& vertCoor, MatrixXX& _textCoor, QImage& inImg)
{
	assert(vertCoor.rows() >0 && _textCoor.rows()>0);

	quadCoor = vertCoor;
	textCoor = _textCoor;

	textSource = inImg;
	//inImg.save("qimageTest2.png");

	nH = inImg.height();
	nW = inImg.width();

	nFaces = vertCoor.rows()/4;

}

//huayun

void makeCheckImage(void)
{
	int i, j, c;
	for (i = 0; i < checkImageHeight; i++) {
		for (j = 0; j < checkImageWidth; j++) {
			c = (((i&0x8)==0)^((j&0x8)==0))*255;
			checkImage[i][j][0] = (GLubyte)c;
			checkImage[i][j][1] = (GLubyte)c;
			checkImage[i][j][2] = (GLubyte)c;
			checkImage[i][j][3] = (GLubyte)255;
			c = (((i&0x10)==0)^((j&0x10)==0))*255;
			otherImage[i][j][0] = (GLubyte)c;
			otherImage[i][j][1] = (GLubyte)0;
			otherImage[i][j][2] = (GLubyte)0;
			otherImage[i][j][3] = (GLubyte)255;
		}
	}
}

void dTexture::initializeGL()
{
	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_CULL_FACE);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_INT, 0, cubeTextureArray);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	cubeTexture = bindTexture(textSource);

	glPushMatrix(); // push to avoid stack underflow in the first paintGL() call
}

void dTexture::resizeGL(int width, int height)
{

	if(0 == height)
		height = 1;//防止一条边为0
	glViewport(0, 0, (GLint)width, (GLint)height);//重置当前视口，本身不是重置窗口的，只不过是这里被Qt给封装好了
	glMatrixMode(GL_PROJECTION);//选择投影矩阵

}

void dTexture::paintGL()
{
	cubeTexture = bindTexture(textSource);
	glBindTexture(GL_TEXTURE_2D, cubeTexture);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0,nW, 0,nH ,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int row = quadCoor.rows();
	int col = quadCoor.cols();


 	for( int i= 0; i<row; i+=4)
 	{
 		glBegin(GL_QUADS);
 		for(int j =3; j>=0; --j)
 		{	
 			glTexCoord2f(textCoor(i+j,0), 1.- textCoor(i+j,1));
 			glVertex2f(quadCoor(i+j,0), nH - quadCoor(i+j,1)); 
 		}
 		glEnd(); 
 	}


	QImage t_image = grabFrameBuffer(true);
	t_image.save(".\\tempImages\\homoDeform2.png");

}

void dTexture::drawDeformedImg(cv::Mat& oriImg,int height,int width, MatrixXX& quads,MatrixXX& texCoors)
{

	cv::Mat newmat;
	//setInputData(quads,texCoors,oriImg);
	cvtColor(oriImg,newmat,CV_RGB2BGR);

	char comName[1024];
	char maskName[1024];

	sprintf(comName,".\\tempImages\\oriimg.jpg");

	//imwrite(comName, tImg);
	imwrite(comName, newmat);
	QImage m_image;
	m_image.load(".\\tempImages\\oriimg.jpg");

	setInputData(quads,texCoors,m_image);

 	setFixedHeight(height);
 	setFixedWidth(width);

	resizeGL(width,height);
	updateGL();

}
