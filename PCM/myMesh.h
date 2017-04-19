#pragma once
#include <vector>
#include "Math/mathlib.h"
using namespace std;

struct triNode
{
	float x, y; 
};

struct triFace
{
	int v0,v1,v2;  // 3 vertices's index	
};

struct triEdge
{
	int v0,v1; // 2 end vertices's index
};

struct quadNode
{
	float x, y; 
};

struct quadFace
{
	int v0,v1,v2, v3; 
};

struct quadEdge
{
	int v0,v1; 
};

struct lineSeg
{
	CvPoint2D32f start_;
	CvPoint2D32f middle_;
	CvPoint2D32f end_;
};

class Cquad
{
private:
	vector<Vector2> qd; 

public:
	Cquad(quadNode a, quadNode b, quadNode c, quadNode d)
	{
		Vector2 v; v.x = a.x; v.y =  a.y; qd.push_back(v); 
		v.x = b.x; v.y =  b.y; qd.push_back(v); 
		v.x = c.x; v.y =  c.y; qd.push_back(v); 
		v.x = d.x; v.y =  d.y; qd.push_back(v); 
	} 

	inline void mvcCoord(float x, float y, vector<float>&w)
	{
		w.resize(4,0); 
		Vector2 v; v.x = x; v.y = y; 
		vector<Vector2> Axis;	Axis.resize(4,Vector2(.0,.0));
		w.clear(); w.resize(4,0.0); 
		vector<float> alpha; alpha.resize(4,0.0); 
		for (int i=0; i<4; ++i){
			Axis[i] = qd[i] - v; 
			if( Axis[i].magnitude() < (std::numeric_limits<double>::min)() ){
				w[i] =1.0;				return;
			}
		}

		for (int i=0; i<4; ++i){
			int q_ax((i+4-1)%4), h_ax(i%4);
			double cosi=Axis[q_ax].cos_V2(Axis[h_ax]) ;
			alpha[i] = acos(cosi);	
		}
		for (int i=0; i<4; ++i)
			w[i]=(tan( alpha[i%4]*0.5 )+tan( alpha[(i+1)%4] *0.5 ) ) / Axis[i].magnitude();

		float total_w = 0; 
		for (int i=0;i<4;i++)
		{
			total_w+=w[i]; 
		}

		for (int i=0; i<4; ++i){	
			w[i] = w[i]/total_w;
		}	
	}

	inline bool inQuad(int x, int y)
	{
		Vector2 v; v.x = x; v.y = y; 
		Vector2 v0, v1, v2, v3; 
		v0 = qd[0] - v; v1 = qd[1]-v; 
		v2 = qd[2] -v; v3 = qd[3] -v; 
		double v01 = v0.x*v1.y-v0.y*v1.x;
		double v12 = v1.x*v2.y-v2.x*v1.y; 
		double v23 = v2.x*v3.y-v3.x*v2.y; 
		double v30 = v3.x*v0.y - v3.y*v0.x; 
		double v012 = v01*v12; double v123 = v12*v23; 
		double v230 = v23*v30; double v301 = v30*v01; 
		if(v012<0&&fabs(v012)>(std::numeric_limits<double>::min)()) return false; 
		if(v123<0&&fabs(v123)>(std::numeric_limits<double>::min)()) return false; 
		if(v230<0&&fabs(v230)>(std::numeric_limits<double>::min)()) return false; 
		if(v301<0&&fabs(v301)>(std::numeric_limits<double>::min)()) return false; 

		if(v012>-(std::numeric_limits<double>::min)()&& v123>-(std::numeric_limits<double>::min)()
	       &&v230>-(std::numeric_limits<double>::min)()&&v301>-(std::numeric_limits<double>::min)()) 
		 return true; 

		 return false; 
	}
};

class Ctrimesh 
{
public:
	Ctrimesh(){}
	~Ctrimesh(){ _faces.clear(); _nodes.clear(); _edges.clear(); }

	inline int numoffaces(){ return _faces.size(); }
	inline int numofnodes(){ return _nodes.size(); }
	inline int numofedges(){ return _edges.size(); }

	inline float calArea(int i) // calculate the area of the i_th triangle
	{
		float ax,bx,cx, ay,by,cy; 
		int a,b,c;  a = _faces[i].v0; b = _faces[i].v1; c= _faces[i].v2; 
		ax = _nodes[a].x; ay = _nodes[a].y ; 
		bx = _nodes[b].x; by = _nodes[b].y ; 
		cx = _nodes[c].x; cy = _nodes[c].y ; 
		float abx, aby, acx, acy; 
		abx = bx-ax; aby=by-ay; acx = cx-ax; acy = cy-ay; 
		float area =0.5*fabs(abx*acy-acx*aby); 

		return area; 
	}

public:
	vector<triFace> _faces; 
	vector<triNode> _nodes; 
	vector<triEdge> _edges; 
};

class Cquadmesh
{
private:
	int nw,nh; 
	float dw, dh; 

public:
	Cquadmesh(){}
	~Cquadmesh(){}
    Cquadmesh(int width, int height, int dp)
	{
		_nodes.clear(); _edges.clear(); 
		_faces.clear(); _boundaries.clear();

		nw = width/dp; nh = height/dp; 
		dw = 1.0*width/nw; dh = 1.0*height/nh;  
		float x, y; quadNode nd; 

		//update-should regularize to [0.- 1.]

		IndexType nodeIdx = 0;

		for (int h=0;h<=nh; ++h)
		{
			y = h*dh; 

			for (int w=0;w<=nw;++w)
			{
				x= w*dw; nd.x = x; nd.y = y; 
				_nodes.push_back(nd); 

				if ( h ==0 || h == nh || w == 0 || w == nw)
				{
					_boundaries.push_back(nodeIdx);
				}

				++ nodeIdx;
			} 
		}

	    //  construct nodes
// 		for (int h=0;h<nh; h++)
// 		{
// 			y = h*dh; 
// 			for (int w=0;w<nw;w++)
// 			{
// 				x= w*dw; nd.x = x; nd.y = y; 
// 				_nodes.push_back(nd); 
// 			}
// 			nd.x = width-1; nd. y = y; _nodes.push_back(nd);  
// 		}
// 		y = height-1; 
// 		for (int w=0;w<nw;w++)
// 		{
// 			x= w*dw; nd.x = x; nd.y = y; 
// 			_nodes.push_back(nd); 
// 		}
// 		nd.x = width-1; nd.y = y; _nodes.push_back(nd); 

		// construct corner
		_corner.resize(4); 
		_corner[0] = 0; _corner[1] = nw;
		_corner[2] = nh*(nw+1)+nw;
		_corner[3] = nh*(nw+1);

		// construct edges
		quadEdge eg; 
		for (int h=0;h<nh;h++)
		{
			for (int w=0;w<=nw;w++)
			{
				eg.v0 = h*(nw+1) + w; 
				eg.v1 = eg.v0 + nw +1; 
				_edges.push_back(eg); 
			}
		}

		for (int w=0;w<nw;w++)
		{
			for (int h=0;h<=nh;h++)
			{
				eg.v0 = h*(nw+1) + w; 
				eg.v1 = eg.v0+1; 
				_edges.push_back(eg); 
			}
		}

		// construct faces
		quadFace face; 
		for (int h=0;h<nh;h++)
		{
			for (int w=0;w<nw;w++)
			{
				int a,b,c,d; a = h*(nw+1)+w; b = a+1; 
				d = a+(nw+1); c = d+1; 
				face.v0 = a; face.v1 = b; face.v2 = c; face.v3 = d; 
				_faces.push_back(face);
			}
		}

		//construct faces to faces relationship //right and bottow
		IndexType nFace = _faces.size();
		f2f.clear();

		IndexType fId = 0;
		for (int h=0;h< nh-1; ++h)
		{
			for (int w=0;w< nw-1;++w)
			{
              IndexType curId = h*nw + w;
			  IndexType rId = curId + 1;
			  IndexType bId = curId + nw;

			  if (curId < nFace && rId < nFace && bId < nFace)
			  {
				  std::pair<int,int> f2r(curId,rId);
				  std::pair<int,int> f2b(curId,bId);
				  f2f.push_back(f2r);
				  f2f.push_back(f2b);
			  }

			} 
		}

		//for the last col

		for (int h=0;h< nh-1; ++h)
		{
			IndexType curId = (h + 1) * nw - 1;
			IndexType bId =   (h + 2) * nw - 1;
	        std::pair<int,int> f2b(curId,bId);
		    f2f.push_back(f2b);
		}

	}

public:
	inline void Coord(float x, float y, vector<int>& nb, vector<float>& w)
	{
		nb.clear(); w.clear(); 
		int nx, ny; nx = (int)(x/dw);  ny = (int)(y/dh); 
		int a,b,c,d; a = ny*(nw+1)+nx; b= a+1; 
		d = a+(nw+1); c = d+1; 
		nb.push_back(a); nb.push_back(b); nb.push_back(c); nb.push_back(d); 
		Cquad quad(_nodes[a],_nodes[b],_nodes[c],_nodes[d]); 
        quad.mvcCoord(x,y,w); 
	} 
	inline int numofnodes(){ return _nodes.size(); }
	inline int numoffaces(){ return _faces.size(); }
	inline int numofedges(){ return _edges.size(); }
	int getnw(){ return nw; }
	int getnh(){ return nh; }

public:
	vector<quadNode> _nodes; 
	vector<quadEdge> _edges; 
	vector<quadFace> _faces; 
	vector<int> _corner; 
public:
	vector<int> _boundaries;
	vector<pair<int,int>> f2f;
};


