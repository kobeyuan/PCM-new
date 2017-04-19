#pragma once

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <fstream>
using namespace std;

#ifndef  	N_GAUSS
#define	N_GAUSS 5
#endif

class GrabCuter
{
public:
	IplImage* imgOriginal;
	IplImage* imgGMM;
	IplImage* imgForeground;
	IplImage* imgOverlay;
	IplImage* imgFGLabel;
	IplImage* imgInput;
	IplImage* imgUI;
	IplImage* imgMask; 

	BOOL m_bOpen;
	float *m_pImgData;
	BYTE *m_Alpha;
	BYTE *m_kFG;
	BYTE *m_kBG;

	int m_nWidth;
	int m_nHeight;
	float m_beta;
	double m_weight; 

	float m_meansFG[N_GAUSS][3];
	float m_varFG[N_GAUSS][3];
	float m_logwFG[N_GAUSS];

	float m_meansBG[N_GAUSS][3];
	float m_varBG[N_GAUSS][3];
	float m_logwBG[N_GAUSS];
	vector<CvPoint> UserInput;

	BOOL m_bToFix;

public:
	GrabCuter(IplImage* src, IplImage* trimap);
	~GrabCuter(void);

public:
	int InitGMM();
	int PixelLabel();
	void GenImages();
	void MinCut();
	void UpdateGMM();
	void UserFix(IplImage* inputLabel);
	int IdxE(int x, int y);
	IplImage* RunGC(void);

private:
	int GetFGLabel(float zn[3]);
	int GetBGLabel(float zn[3]);
	float ComputeD(int alpha, int k, float z[3]);
private:
	float ComputeV(int alpha1, int alpha2, float z1[3], float z2[3]);
};
