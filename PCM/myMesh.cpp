#include "myMesh.h"
#include <math.h>


float Ctrimesh::calArea(int i)
{
	float a; 
	int v0, v1, v2; 
	v0 = _faces[i].v0; v1 = _faces[i].v1; v2 = _faces[i].v2; 
	float x01 = _nodes[v1].x - _nodes[v0].x;  
	float y01 = _nodes[v1].y - _nodes[v0].y;  
	float x02 = _nodes[v2].x - _nodes[v0].x;  
	float y02 = _nodes[v2].y - _nodes[v0].y;

	a = fabs(x01*y02-x02*y01)/2.0; 

	return a; 
}