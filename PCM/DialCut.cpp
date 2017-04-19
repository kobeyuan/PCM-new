#include "DialCut.h"

#include <math.h>

DialCut::DialCut(IndexType _w, IndexType _h, vector<Mat> _imgData, 
				 MatrixXXi& _mask, MatrixXXi _initLabels)
{
	m_width = _w;
	m_height = _h;
	m_imgData = _imgData;
	m_mask = _mask;
	m_labels = m_initLabels = _initLabels;

	m_regular = 1.f;
	m_potts = 1e-6;
	m_inertia = 0.01f;
	m_curSrImgId = 1;

	indeces_a = (PtrImage) imNew(IMAGE_PTR, _w, _h);
	D_a = (DoubleImage) imNew(IMAGE_DOUBLE, _w, _h);
	_size.x = _w;_size.y = _h;

	_strokeDmap = NULL;
	makeInertiaIrrelevant();

	if (_imgData.size() > 1)
	{
		IndexType size = _imgData.size();

		for (int i = 1; i < size; ++i)
		{
			m_newRelevant.push_back(i);
		}
    }
}

void DialCut::run()
{
	Loggger<<"Do nothing!.\n";
}

void DialCut::compute()
{

	//
	vector<RowSpan> spans;
	compactStroke(m_mask,spans);
	makeInertiaRelevant(spans);
	//

	double E, E_old;
	int cutCount = 0;

	E = BVZ_ComputeEnergy();

	MatrixXXi bestLabels;
	bestLabels.resize(m_height,m_width);
	bestLabels.setZero();

	int best = 0;

	vector<IndexType>::reverse_iterator c;

	for (c = m_newRelevant.rbegin(); c!= m_newRelevant.rend(); ++c) 
	{
		E_old = E;

		E = BVZ_Expand(*c, E_old);  
		
		++cutCount;

		assert (E <= E_old);

		if (E<E_old) 
		{
			best = *c;
			bestLabels = m_labels;
		}

		m_labels = m_initLabels;

	 }

	m_labels = bestLabels;
}

double DialCut::BVZ_ComputeEnergy()
{
	double E=0;

	int i,j,k;
	Coord np;

	for (j=0; j< m_height; j++)
	 for (i=0; i< m_width; i++) 
		{
			Coord coord(i,j);

			E += BVZ_data_penalty(coord,m_labels(j,i));

			for (k=0; k<(int)NEIGHBOR_NUM; k++)
			{
				np = coord + NEIGHBORS[k];
				if (np>=Coord(0,0) && np<_size)
					E += BVZ_interaction_penalty(coord,np,m_labels(j,i), m_labels(np.y,np.x) );	
			}

		}

	return E;
}

float DialCut::BVZ_data_penalty(Coord p, IndexType d)
{
	assert(d >= 0);

	if (p.y < 0 || p.y > m_height || p.x >m_width || p.x < 0)
	{
		return A_INFINITY;//the point is out of the size
	}

	bool isUnderStroke = m_mask(p.y,p.x);

	if (isUnderStroke)
	{
		if (d == m_curSrImgId)
		{
			return 0.f;
		}else
		{
			return A_INFINITY;
		}
	}else
	{
		if (_inertiaRelevant)
		{
			if (d == m_labels(p.y,p.x))
			{
				return 0.f;
			}else
			{
				assert(_strokeDmap);

				return m_inertia*.1*_strokeDmap->dist(p.x, p.y);
			}

		}

		return 0.f;
	}

}

float DialCut::BVZ_interaction_penalty(Coord p, Coord np, IndexType l, IndexType nl)
{
	if ( l == nl)
	{
		return 0.f;
	}

	return (m_potts + m_regular * smoothItem(p,np,l,nl) );
}

float DialCut::smoothItem(Coord p, Coord np, IndexType l, IndexType nl)
{
	IndexType imgSize = m_imgData.size();

	assert(l < imgSize && nl < imgSize);

	float a,M =0.;
	int c,k;

	if(l == nl)
	{
	   return 0.f;
	}
	
	Vec3b sColor,tColor;// source--1; targe--0.

	// difference at p pixel
	a=0;
	sColor = m_imgData[1].at<cv::Vec3b>(p.y,p.x);
	tColor = m_imgData[0].at<cv::Vec3b>(p.y,p.x);
 
	for (c=0; c<3; ++c) 
	{
		k = sColor[c] - tColor[c];
		a += k*k;    
	}

	M = sqrt(a);

	// difference at np pixel
	a=0;
	sColor = m_imgData[1].at<cv::Vec3b>(np.y, np.x);
	tColor = m_imgData[0].at<cv::Vec3b>(np.y, np.x);

	for (c=0; c<3; ++c)
	{
		k = sColor[c] - tColor[c];
		a += k*k;    
	}

	M += sqrt(a);


	M /=6.f;

	// gradient denominator
 	bool isEdge = true;
 
 	if(isEdge)
 	{
 		float G;
 		if (p.x!=np.x) 
 		{  // vertical cut, vertical Sobel filter
 			Coord minp(min(p.x,np.x), p.y);
 			if (p.y>0 && p.y<m_height-1) 
 			{
 				G = .5f*(vertGradMag(l,minp) + vertGradMag(nl,minp));
 			}
 			else
 				G = 1.f;
 		}
 		else 
 		{  // horizontal cut, horizontal Sobel filter
 			Coord minp(p.x, min(p.y,np.y));
 			if (p.x>0 && p.x<m_width-1)
 			{
 				G = .5f*(horizGraMag(l,minp) + horizGraMag(nl,minp));
 			}
 			else
 				G = 1.f;
 		}
 
 		if (G==0)
 			M = A_INFINITY;
 		else
 			M /= G;
 	}



	//difference at np pixel

	if(M > A_INFINITY)
		M = A_INFINITY;

	return M;
}

#ifdef BVZ_ALPHA_SINK
#define BVZ_TERM_A Graph::SOURCE
#define BVZ_TERM_B Graph::SINK
#else
#define BVZ_TERM_A Graph::SINK
#define BVZ_TERM_B Graph::SOURCE
#endif

void BVZ_error_function(char *msg)
{
	fprintf(stderr, "%s\n", msg);
	assert(0);
	exit(1);
}

double DialCut::BVZ_Expand(IndexType a, double E_old)
{
	Coord p, np;
	IndexType l, nl;
	Graph *g;
	float delta, P_00, P_0a, P_a0;
	Graph::node_id index, nindex;
	int k, ind=0;
	double E;

    g = new Graph(BVZ_error_function);

	/* initializing */
	E = 0.;
	for (p.y=0; p.y<m_height; p.y++)
		for (p.x=0; p.x<m_width; p.x++, ++ind)
		{
			l = m_labels(p.y,p.x);

			if (a == l) // 和原始的label一样
			{
				IMREF(indeces_a, p) = INDEX_ACTIVE; //返回点p在graph中的节点索引，默认为0
				E += BVZ_data_penalty(p, l);
				continue;
			}

			// 	而void *则不同，任何类型的指针都可以直接赋值给它，无需进行强制类型转换：//void *p1; //int *p2; 		// 	p1 = p2	

			//label 不一样的话

			IMREF(indeces_a, p) = g -> add_node();
			delta = BVZ_data_penalty(p, l);
			IMREF(D_a, p) = BVZ_data_penalty(p, a) - delta;
			E += delta;
		}

		ind=0;
		for (p.y=0; p.y<m_height; p.y++)
			for (p.x=0; p.x< m_width; p.x++, ++ind)
			{
				l = m_labels(p.y,p.x);
				index = (Graph::node_id) IMREF(indeces_a, p);

				/* adding interactions */
				for (k=0; k<(int)NEIGHBOR_NUM; k++)
				{
					np = p + NEIGHBORS[k];
					if ( ! ( np>=Coord(0,0) && np<_size ) ) continue; //HUM
					nl = m_labels(np.y,np.x);
					nindex = (Graph::node_id) IMREF(indeces_a, np);//获取在graph 中node的id

					if (IS_NODE_ID(index))
					{
						if (IS_NODE_ID(nindex))
						{
							P_00 = BVZ_interaction_penalty(p, np, l, nl);
							P_0a = BVZ_interaction_penalty(p, np, l,  a);
							P_a0 = BVZ_interaction_penalty(p, np, a, nl);
							delta = (P_00 <  P_0a) ? P_00 : P_0a;
							if (delta > 0)
							{
								IMREF(D_a, p) -= delta; E += delta;
								P_00 -= delta;
								P_0a -= delta;
							}
							delta = (P_00 < P_a0) ? P_00 : P_a0;
							if (delta > 0)
							{
								IMREF(D_a, np) -= delta; E += delta;
								P_00 -= delta;
								P_a0 -= delta;
							}
							//if (P_00 > 0.0001) { fprintf(_fp, "ERROR: BVZ_interaction_penalty() is non-metric %f!\n",P_00); fflush(_fp); /*assert(0);*/ }
#ifdef BVZ_ALPHA_SINK
							g -> add_edge(index, nindex, P_0a, P_a0);
#else
							g -> add_edge(index, nindex, P_a0, P_0a);
#endif
						}
						else
						{
							delta = BVZ_interaction_penalty(p, np, l, a);
							IMREF(D_a, p) -= delta; E += delta;
						}
					}
					else
					{
						if (IS_NODE_ID(nindex))
						{
							delta = BVZ_interaction_penalty(p, np, a, nl);
							IMREF(D_a, np) -= delta; E += delta;
						}
					}
				}// end for neighbor
			}// end for image-pixel


			/* adding source and sink edges */
			for (p.y=0; p.y<m_height; p.y++)
				for (p.x=0; p.x<m_width; p.x++)
				{
					index = (Graph::node_id) IMREF(indeces_a, p);
					if (IS_NODE_ID(index))
					{
						delta = (float) IMREF(D_a, p);
#ifdef BVZ_ALPHA_SINK
						if (delta > 0) { g -> set_tweights(index, delta, 0); }
						else           { g -> set_tweights(index, 0, -delta); E += delta; }
#else
						if (delta > 0) { g -> set_tweights(index, 0, delta); }
						else           { g -> set_tweights(index, -delta, 0); E += delta; }
#endif
					}
				}

				E += g -> maxflow();

				//fprintf(_fp, "internal E: %f\n",E); fflush(_fp);
				if (E < E_old)
				{
					//fprintf(_fp,"Writing into _labels\n"); fflush(_fp);
					ind=0;
					for (p.y=0; p.y< m_height; p.y++)
						for (p.x=0; p.x<m_width; p.x++, ++ind)
						{
							index = (Graph::node_id) IMREF(indeces_a, p);
							if (IS_NODE_ID(index) && g->what_segment(index)==BVZ_TERM_B)
							{
								m_labels(p.y,p.x) = a; //进行交换操作
							}
						}

						delete g;
						return E;
				}

				delete g;
				return E_old;
}

void DialCut::makeInertiaRelevant(const vector<RowSpan>& spans)
{
	_inertiaRelevant = true;
	if (_strokeDmap) delete _strokeDmap;
	_strokeDmap = new DistanceMap(m_width, m_height, spans);

	//visual distance with gray image

	Mat disMap;
	disMap.create(m_height,m_width,CV_8UC1);

	for (int y=0; y< m_height; ++y)
	{
		for (int x=0; x< m_width; ++x)
		{
			disMap.at<uchar>(y,x) = _strokeDmap->dist(x,y);
		}
	}

	//imshow("disMap",disMap);

}

void DialCut::makeInertiaIrrelevant()
{
	_inertiaRelevant = false;
	if (_strokeDmap) {
		delete _strokeDmap;
		_strokeDmap = NULL;
	}
}

void DialCut::compactStroke(MatrixXXi& _strokesMask,std::vector<RowSpan>& spans) const
{
	int x,y,num;

	for (y=0; y< m_height; ++y)
	{
		x=0;
		while (x < m_width) 
		{
			if (_strokesMask(y,x) ==1) 
			{
				num=0;
				spans.push_back(RowSpan(x,y,0));
				assert(x>=0 && y>=0 &&x<m_width && y< m_height);
				do 
				{
					++x; ++num;
				} while (x<m_width && _strokesMask(y,x) ==1);

				spans[spans.size()-1]._num = num;
			}
			else 
			{
				++x; 
			}

		} // end x<w

	} // end loop over rows
}

float DialCut::vertGradMag(int imgIdx, Coord& pixPos)
{
	float sum = 0, d;

	int y = pixPos.y;
	int x = pixPos.x;

   Vec3b tl = m_imgData[imgIdx].at<Vec3b>(y-1,x);
   Vec3b ml = m_imgData[imgIdx].at<Vec3b>(y,x);
   Vec3b bl = m_imgData[imgIdx].at<Vec3b>(y+ 1,x);

   Vec3b tr = m_imgData[imgIdx].at<Vec3b>(y-1,x+1);
   Vec3b mr = m_imgData[imgIdx].at<Vec3b>(y,x+1);
   Vec3b br = m_imgData[imgIdx].at<Vec3b>(y+1,x+1);

   for (int i = 0; i < 3; ++ i)
   {
	   d = (float)tl[i] + 2.f * (float)ml[i] + (float)bl[i] -
		   (float)tr[i] - 2.f * (float)mr[i] - (float)br[i];
	   
	   d /= 3.f;
	   
	   sum += d*d;
   }

   d = sqrt(sum);

   return d;
}

float DialCut::horizGraMag(int imgIdx, Coord& pixPos)
{
	float sum = 0, d;

	int y = pixPos.y;
	int x = pixPos.x;

	Vec3b tl = m_imgData[imgIdx].at<Vec3b>(y,x-1);
	Vec3b ml = m_imgData[imgIdx].at<Vec3b>(y,x);
	Vec3b bl = m_imgData[imgIdx].at<Vec3b>(y,x+1);

	Vec3b tr = m_imgData[imgIdx].at<Vec3b>(y+1,x-1);
	Vec3b mr = m_imgData[imgIdx].at<Vec3b>(y+1,x);
	Vec3b br = m_imgData[imgIdx].at<Vec3b>(y+1,x+1);

	for (int i = 0; i < 3; ++ i)
	{
		d = (float)tl[i] + 2.f * (float)ml[i] + (float)bl[i] -
			(float)tr[i] - 2.f * (float)mr[i] - (float)br[i];

		d /= 3.f;

		sum += d*d;
	}

	d = sqrt(sum);

	return d;
}