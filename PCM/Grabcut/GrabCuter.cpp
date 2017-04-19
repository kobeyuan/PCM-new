#include "GrabCuter.h"
#include "gmm.h"
#include "MinCut\energy.h"
#include "OpticalFlow/OpticalFlow.h"
#include <iostream>
#include "ANN.h"

using namespace std; 
#define  EPS_6 1e-6

GrabCuter::GrabCuter(IplImage* src, IplImage * trimap)
{
	m_Alpha = NULL;
	m_kFG = NULL;
	m_kBG = NULL;
	m_bToFix = FALSE;
	m_weight = 0.8;

	// TODO:  Add your specialized creation code here

	imgOriginal=cvCloneImage(src);
	imgGMM=cvCloneImage(src);
	imgForeground=cvCloneImage(src);
	imgOverlay=cvCloneImage(src);
	imgInput=cvCloneImage(trimap);
    imgMask=cvCloneImage(trimap);
	imgUI=cvCreateImage(cvGetSize(src),IPL_DEPTH_32S,1);
	cvZero(imgUI);

	
	UserInput.clear();
	for (int i=0;i<imgInput->height;i++)
		for (int j=0;j<imgInput->width;j++)
		{
			if (CV_IMAGE_ELEM(imgInput,BYTE,i,j)>100)
			{
				UserInput.push_back(cvPoint(j,i));
				CV_IMAGE_ELEM(imgUI,int,i,j)=UserInput.size()-1;
			}
		}

	m_nWidth = imgOriginal->width;
	m_nHeight = imgOriginal->height;
	int nImgSize = m_nWidth * m_nHeight;

	m_pImgData = new float [nImgSize * 3];
	m_Alpha = new BYTE [nImgSize];
	m_kFG = new BYTE [nImgSize];
	m_kBG = new BYTE [nImgSize];

	memset(m_Alpha,2,nImgSize*sizeof(BYTE));

	int nPitch = imgOriginal->widthStep;
	BYTE *pBits = (BYTE *)(imgOriginal->imageData);
	for (int j = 0; j < m_nHeight; j++) {
		for (int i = 0; i < m_nWidth*3; i++) {
			m_pImgData[j*m_nWidth*3 + i] = pBits[i];
		}
		pBits += nPitch;
	}
	m_bOpen = TRUE;
}

GrabCuter::~GrabCuter(void)
{
}
int GrabCuter::InitGMM()
{
//	using namespace Torch;
//	CFile filefg(_T("datafg.bin"), CFile::modeCreate | CFile::modeWrite);
//	CFile filebg(_T("databg.bin"), CFile::modeCreate | CFile::modeWrite);
	ofstream filefg,filebg;
	filefg.open("datafg.bin", ios_base::out | ios_base::binary);
	filebg.open("databg.bin", ios_base::out | ios_base::binary);

	//int nFrames = m_nWidth * m_nHeight;
	int nFrameSize = 3;

	int buffersize = m_nWidth * m_nHeight * 3 * sizeof(float);
	float *fg = new float [m_nWidth * m_nHeight * 3 ];
	float *bg = new float [m_nWidth * m_nHeight * 3];

	int fg_size = 0;
	int bg_size = 0;
	float Ez2 = 0;
	float Ez[3];
	memset(Ez, 0, 3*sizeof(float));


	for (int y = 0; y < m_nHeight; y++) {
		for (int x = 0; x < m_nWidth; x++) {
			Ez2 += m_pImgData[y*m_nWidth*3 + x*3] * m_pImgData[y*m_nWidth*3 + x*3] +
				   m_pImgData[y*m_nWidth*3 + x*3 + 1] * m_pImgData[y*m_nWidth*3 + x*3 + 1] +
				   m_pImgData[y*m_nWidth*3 + x*3 + 2] * m_pImgData[y*m_nWidth*3 + x*3 + 2];

			Ez[0] += m_pImgData[y*m_nWidth*3 + x*3];
			Ez[1] += m_pImgData[y*m_nWidth*3 + x*3 + 1];
			Ez[2] += m_pImgData[y*m_nWidth*3 + x*3 + 2];

			if (CV_IMAGE_ELEM(imgInput,BYTE,y,x)==255) {
				memcpy(&fg[fg_size*3], &m_pImgData[y*m_nWidth*3 + x*3], 3*sizeof(float));
				fg_size++;
				m_Alpha[y*m_nWidth + x] = 1;
			}

			if(CV_IMAGE_ELEM(imgInput,BYTE,y,x)==0) 
			{
				memcpy(&bg[bg_size*3], &m_pImgData[y*m_nWidth*3 + x*3], 3*sizeof(float));
				bg_size++;
				m_Alpha[y*m_nWidth + x] = 0;
			}
		}
	}
	Ez2 = Ez2 / (m_nWidth * m_nHeight);
	Ez2 -= (Ez[0]*Ez[0] + Ez[1]*Ez[1] + Ez[2]*Ez[2]) / (m_nWidth * m_nHeight) / (m_nWidth * m_nHeight);

	m_beta = 0.25 / Ez2;
	filefg.write((const char*)(&fg_size), sizeof(int));
	filefg.write((const char*)(&nFrameSize), sizeof(int));
	filebg.write((const char*)(&bg_size), sizeof(int));
	filebg.write((const char*)(&nFrameSize), sizeof(int));

	filefg.write((const char*)fg, fg_size*3*sizeof(float));
	filebg.write((const char*)bg, bg_size*3*sizeof(float));

	filefg.close();
	filebg.close();

	delete [] fg;
	delete [] bg;

	GMM("datafg.bin", m_meansFG, m_varFG, m_logwFG);
	GMM("databg.bin", m_meansBG, m_varBG, m_logwBG);

	return 0;
}

int GrabCuter::PixelLabel(void)
{
	int minGauss;
	float *buffer = m_pImgData;

	for (int y = 0; y < m_nHeight; y++) {
		for (int x = 0; x < m_nWidth; x++) {
			minGauss = GetBGLabel(&buffer[y*m_nWidth*3 + x*3]);
			m_kBG[y*m_nWidth + x] = minGauss;

			if (CV_IMAGE_ELEM(imgInput,BYTE,y,x)!=0 ) {
				minGauss = GetFGLabel(&buffer[y*m_nWidth*3 + x*3]);
				m_kFG[y*m_nWidth + x] = minGauss;
			}
		}
	}

	return 0;
}

int GrabCuter::GetFGLabel(float zn[3])
{
	float minD = 1e30;
	float D;
	float R, G, B;

	int minGauss = 0;

	B = zn[0];
	G = zn[1];
	R = zn[2];

	for (int i = 0; i < N_GAUSS; i++) {
		D = ComputeD(1, i, zn);

		if (D < minD) {
			minD = D;
			minGauss = i;
		}
	}
	return minGauss;
}

int GrabCuter::GetBGLabel(float zn[3])
{
	float minD = 1e30;
	float D;
	float R, G, B;

	int minGauss = 0;

	B = zn[0];
	G = zn[1];
	R = zn[2];

	for (int i = 0; i < N_GAUSS; i++) {
		D = ComputeD(0, i, zn);

		if (D < minD) {
			minD = D;
			minGauss = i;
		}
	}
	return minGauss;
}

float GrabCuter::ComputeD(int alpha, int k, float z[3])
{
	float D;
	if (alpha == 0)
		D = -m_logwBG[k] + 0.5*( log(m_varBG[k][0]*m_varBG[k][1]*m_varBG[k][2]) +
		(z[0]-m_meansBG[k][0])*(z[0]-m_meansBG[k][0])/m_varBG[k][0] +
		(z[1]-m_meansBG[k][1])*(z[1]-m_meansBG[k][1])/m_varBG[k][1] +
		(z[2]-m_meansBG[k][2])*(z[2]-m_meansBG[k][2])/m_varBG[k][2] );
	else
		D = -m_logwFG[k] + 0.5*( log(m_varFG[k][0]*m_varFG[k][1]*m_varFG[k][2]) +
		(z[0]-m_meansFG[k][0])*(z[0]-m_meansFG[k][0])/m_varFG[k][0] +
		(z[1]-m_meansFG[k][1])*(z[1]-m_meansFG[k][1])/m_varFG[k][1] +
		(z[2]-m_meansFG[k][2])*(z[2]-m_meansFG[k][2])/m_varFG[k][2] );

	return D;
}

void GrabCuter::GenImages()
{
	int minGauss;
	int nPitch = imgGMM->widthStep;

	BYTE *pBitsGMM = (BYTE *)imgGMM->imageData;
	BYTE *pBitsFG = (BYTE *)imgForeground->imageData;
	BYTE *pBitsImg = (BYTE *)imgOriginal->imageData;
	BYTE *pBitsOL = (BYTE *)imgOverlay->imageData;

	IplImage* msk = cvCloneImage(imgMask); 
	int w = msk ->width; 
	int h = msk->height; 
	cvZero(msk); 
	for (int y= 0;y<h;y++)
		for ( int x=0;x<w;x++)
		{
				if (m_Alpha[y*m_nWidth + x] == 1) {
					cvSet2D(msk,y,x,cvScalar(255));
				}
		}
	IplImage* mask = cvCloneImage(msk); 
	cvSmooth(msk,mask);
	cvSmooth(mask,msk);

	//cvShowImage("msk", msk);

	for (int y = 0; y < m_nHeight; y++) {
		for (int x = 0; x < m_nWidth; x++) {

		//	if (m_Alpha[y*m_nWidth + x] == 1) {
			if(CV_IMAGE_ELEM(msk,BYTE,y,x)>=100) {
		    	minGauss = m_kFG[y*m_nWidth + x];
				pBitsOL[x*3] = pBitsFG[x*3] = pBitsImg[x*3];
				pBitsOL[x*3 + 1] = pBitsFG[x*3 + 1] = pBitsImg[x*3 + 1];
				pBitsOL[x*3 + 2] = pBitsFG[x*3 + 2] = pBitsImg[x*3 + 2];
				cvSet2D(imgMask, y, x, cvScalar(255));
			}
			else {
				cvSet2D(imgMask, y, x, cvScalar(0));
				minGauss = m_kBG[y*m_nWidth + x];
				pBitsFG[x*3] = 255;
				pBitsFG[x*3 + 1] = 255;
				pBitsFG[x*3 + 2] = 255;

				pBitsOL[x*3] = pBitsImg[x*3] / 3 + 120;
				pBitsOL[x*3 + 1] = pBitsImg[x*3 + 1] / 3 + 120;
				pBitsOL[x*3 + 2] = pBitsImg[x*3 + 2] / 3 + 120;
			}

			pBitsGMM[x*3] = minGauss*60;
			pBitsGMM[x*3 + 1] = minGauss*60;
			pBitsGMM[x*3 + 2] = minGauss*60;
		}
		pBitsGMM += nPitch;
		pBitsFG += nPitch;
		pBitsImg += nPitch;
		pBitsOL += nPitch;
	}

}

float GrabCuter::ComputeV(int alpha1, int alpha2, float z1[3], float z2[3])
{
	if (alpha1 == alpha2)
		return 0;

	float z;
	z = (z1[0]-z2[0])*(z1[0]-z2[0]) + (z1[1]-z2[1])*(z1[1]-z2[1]) +
		(z1[2]-z2[2])*(z1[2]-z2[2]);
	float beta = m_beta * 20;
	return 50 * exp(-beta * z);

}

int GrabCuter::IdxE(int x, int y)
{
	return CV_IMAGE_ELEM(imgUI,int,y,x);
}


void GrabCuter::MinCut()
{
	float *buffer = m_pImgData;
	//BYTE *pFGLabel;
	//int nPitch;
	//int nbpp;

	//if (m_bToFix) {
	//	imgFGLabel.Destroy();
	//	imgFGLabel.Load(_T("fglabel.bmp"));
	//pFGLabel = (BYTE *)imgFGLabel.GetBits();
	//nPitch = imgFGLabel.GetPitch();
	//nbpp = imgFGLabel.GetBPP();
	//}

	float D[2], V;
	Energy::Var *varAlpha;
	Energy *e = new Energy();

	int varSize = UserInput.size();
	varAlpha = new Energy::Var [varSize];
	

	for (int i = 0; i < varSize; i++)
		varAlpha[i] = e -> add_variable();

	int x,y;
	float Ve;
	for (int i=0;i<varSize;i++) 
	{
		x=UserInput[i].x;
		y=UserInput[i].y;

		D[0] = m_weight*ComputeD(0, m_kBG[y*m_nWidth + x], &buffer[y*m_nWidth*3 + x*3]);
		D[1] = m_weight*ComputeD(1, m_kFG[y*m_nWidth + x], &buffer[y*m_nWidth*3 + x*3]);
		if (m_bToFix) {
			if (CV_IMAGE_ELEM( imgFGLabel,BYTE,y, x*3+2) == 255)
				e -> add_term1(varAlpha[i], 200, 0);
			else if (CV_IMAGE_ELEM( imgFGLabel,BYTE,y, x*3) == 255)
				e -> add_term1(varAlpha[i], 0, 200);
			else
				e -> add_term1(varAlpha[i], D[0], D[1]);
		}
		else
			e -> add_term1(varAlpha[i], D[0], D[1]);
		Ve=0.0;
		if (CV_IMAGE_ELEM( imgInput,BYTE,y, x+1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[y*m_nWidth*3 + (x+1)*3]);
		else
		{
			V = ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[y*m_nWidth*3 + (x+1)*3]);
			e -> add_term2(varAlpha[i], varAlpha[IdxE(x+1,y)], 0, V, V, 0);
		}
		if (CV_IMAGE_ELEM( imgInput,BYTE,y+1, x+1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + (x+1)*3]);
		else
		{
			V = ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + (x+1)*3]);
			e -> add_term2(varAlpha[i], varAlpha[IdxE(x+1,y+1)], 0, V, V, 0);
		}
		if (CV_IMAGE_ELEM( imgInput,BYTE,y+1, x) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + x*3]);
		else
		{
			V = ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + x*3]);
			e -> add_term2(varAlpha[i], varAlpha[IdxE(x,y+1)], 0, V, V, 0);
		}
		if (CV_IMAGE_ELEM( imgInput,BYTE,y+1, x-1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + (x-1)*3]);
		else
		{
			V = ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y+1)*m_nWidth*3 + (x-1)*3]);
			e -> add_term2(varAlpha[i], varAlpha[IdxE(x-1,y+1)], 0, V, V, 0);
		}
		if (CV_IMAGE_ELEM( imgInput,BYTE,y, x-1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[y*m_nWidth*3 + (x-1)*3]);
		if (CV_IMAGE_ELEM( imgInput,BYTE,y-1, x-1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y-1)*m_nWidth*3 + (x-1)*3]);
		if (CV_IMAGE_ELEM( imgInput,BYTE,y-1, x) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y-1)*m_nWidth*3 + x*3]);
		if (CV_IMAGE_ELEM( imgInput,BYTE,y-1, x+1) == 0)
			Ve += ComputeV(0, 1, &buffer[y*m_nWidth*3 + x*3], &buffer[(y-1)*m_nWidth*3 + (x+1)*3]);
		if (Ve!=0.0)
			e -> add_term1(varAlpha[i], 0, Ve);

	}
	Energy::TotalValue Emin = e -> minimize();

	for(int i=0;i<varSize;i++) 
	{
		x=UserInput[i].x;
		y=UserInput[i].y;

		m_Alpha[y*m_nWidth + x] = e->get_var(varAlpha[i]);
	}
	

	delete e;
	delete [] varAlpha;
}

void GrabCuter::UpdateGMM()
{
	float *buffer = m_pImgData;
	int bgsize = 0;
	int fgsize = 0;

	memset(m_meansFG, 0, sizeof(float)*15);
	memset(m_varFG, 0, sizeof(float)*15);
	memset(m_logwFG, 0, sizeof(float)*N_GAUSS);

	memset(m_meansBG, 0, sizeof(float)*15);
	memset(m_varBG, 0, sizeof(float)*15);
	memset(m_logwBG, 0, sizeof(float)*N_GAUSS);

	for (int y = 0; y < m_nHeight; y++) {
		for (int x = 0; x < m_nWidth; x++) {

			if (m_Alpha[y*m_nWidth + x] == 1) {
				for (int i = 0; i < 3; i++) {
					m_meansFG[ m_kFG[y*m_nWidth + x] ][i] += buffer[y*m_nWidth*3 + x*3 + i];
					m_varFG[ m_kFG[y*m_nWidth + x] ][i] += buffer[y*m_nWidth*3 + x*3 + i] *
						buffer[y*m_nWidth*3 + x*3 + i];
				}
				m_logwFG[ m_kFG[y*m_nWidth + x] ] += 1;
				fgsize++;
			}
			//else 
			if (m_Alpha[y*m_nWidth + x] == 0)
			{
				for (int i = 0; i < 3; i++) {
					m_meansBG[ m_kBG[y*m_nWidth + x] ][i] += buffer[y*m_nWidth*3 + x*3 + i];
					m_varBG[ m_kBG[y*m_nWidth + x] ][i] += buffer[y*m_nWidth*3 + x*3 + i] *
						buffer[y*m_nWidth*3 + x*3 + i];
				}
				m_logwBG[ m_kBG[y*m_nWidth + x] ] += 1;
				bgsize++;
			}	

		}
	}

	for (int j = 0; j < N_GAUSS; j++) {
		for (int i = 0; i < 3; i++) {
			m_meansBG[j][i] /= m_logwBG[j];
			m_meansFG[j][i] /= m_logwFG[j];
			m_varBG[j][i] /= m_logwBG[j];
			m_varFG[j][i] /= m_logwFG[j];
			m_varBG[j][i] -= m_meansBG[j][i] * m_meansBG[j][i];
			m_varFG[j][i] -= m_meansFG[j][i] * m_meansFG[j][i];
		}
		m_logwBG[j] /= bgsize;
		m_logwFG[j] /= fgsize;
		m_logwBG[j] = log( m_logwBG[j] );
		m_logwFG[j] = log( m_logwFG[j] );
	}

}

void GrabCuter::UserFix(IplImage* inputLabel)
{
	imgFGLabel=inputLabel;
	int x,y;
	for (int i=0;i<UserInput.size();i++) 
	{
		x=UserInput[i].x;
		y=UserInput[i].y;

		if (CV_IMAGE_ELEM( imgFGLabel,BYTE,y, x*3+2)== 255)
			m_Alpha[y*m_nWidth + x] = 1;
		else if (CV_IMAGE_ELEM( imgFGLabel,BYTE,y, x*3) == 255)
			m_Alpha[y*m_nWidth + x] = 0;
	}

	UpdateGMM();

	m_bToFix = TRUE;
	MinCut();
	m_bToFix = FALSE;
	GenImages();

}

IplImage* GrabCuter::RunGC(void)
{
	InitGMM();

	for (int i = 0; i <5; i++) 
	{
		PixelLabel();
		MinCut();
    	UpdateGMM();
	}
	PixelLabel();
	GenImages();
	return imgForeground;
}
