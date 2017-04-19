#include "Bezier.h"
#include "Math/LeastSquaresSparseSolver.h"

CBezier::CBezier(void)
{
}

CBezier::~CBezier(void)
{
	_cp.clear(); _p.clear();  _wp.clear(); _p2c.clear(); 
}

void CBezier::QuadFitting(int s, int e)
{
	// calculate weight
	int n = _p.size(); 
	if(s<0||s>=n||e<0||e>=n||s>e) return; 
	vector<int> seg; 
	if(e-s+1<=3) 
	{ 
	    for (int t=s; t<=e;t++) seg.push_back(t); 
        _segs.push_back(seg); return; 
	}

	vector< vector<float>> w; 
	for (int t=s;t<=e;t++)
	{
		vector<float> wt; 
		float r = 1.0*(t-s)/(e-s); 
		wt.push_back((1-r)*(1-r));  wt.push_back(2*r*(1-r)); wt.push_back(r*r); 
		w.push_back(wt); 
	}

	n = e-s+1; 

	// construct linear system
	const int rows = n;  const int cols = 3;
	LeastSquaresSparseSolver slover; 
	float **b = new float*[2];

	b[0] = new float[rows];	b[1] = new float[rows];	

	slover.Create(rows,cols,2); int row = 0;
	for (int i=0;i<n;i++)
	{
		int t= i+s; 
		slover.AddSysElement(row, 0, w[i][0]); 
		slover.AddSysElement(row, 1, w[i][1]);
		slover.AddSysElement(row, 2, w[i][2]);
		b[0][row] = _p[t].x; b[1][row] = _p[t].y; row++; 
	}

	slover.SetRightHandSideMatrix(b); slover.CholoskyFactorization(); 
	slover.CholoskySolve(0); slover.CholoskySolve(1); 
	
	vector<CvPoint2D32f> cp(3, cvPoint2D32f(0,0)); 
	cp[0].x = slover.GetSolution(0,0); cp[1].x = slover.GetSolution(0,1); cp[2].x = slover.GetSolution(0,2);
	cp[0].y = slover.GetSolution(1,0); cp[1].y = slover.GetSolution(1,1); cp[2].y = slover.GetSolution(1,2);

	// calculate the error
	float error = -99999999; int mid = -1; 
	vector<float> qx, qy; 
	for (int t =0;t<n;t++)
	{
		float x = w[t][0]*cp[0].x + w[t][1]*cp[1].x + w[t][2]*cp[2].x; 
		float y = w[t][0]*cp[0].y + w[t][1]*cp[1].y + w[t][2]*cp[2].y; 
		float d = sqrt((x-_p[s+t].x)*(x-_p[s+t].x)+(y-_p[s+t].y)*(y-_p[s+t].y)); 
		if(d>error) { error = d; mid = t+s; }
		qx.push_back(x); qy.push_back(y); 
	}

	if(error>_thr&&n>=80)
	{
		if(mid==e)
		{
			QuadFitting(s,mid-1); 
			QuadFitting(mid, e); 
		}
		else
		{
			QuadFitting(s,mid); 
			QuadFitting(mid+1, e); 
		}
	}
	else
	{
		seg.clear(); 
		for (int t=s;t<=e;t++) seg.push_back(t); 
		_segs.push_back(seg); 
	}
}

void CBezier::QuadFitting(vector<CvPoint2D32f> p, vector< vector<int>>& segs)

{
	_p = p; int n = _p.size()-1; _segs.clear(); 
	QuadFitting(0, n); segs=_segs; 
}
