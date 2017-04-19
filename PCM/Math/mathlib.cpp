#include "mathlib.h"
#include <limits>

//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix2*//////////////////////////////////
//////////////////////////////////////////////////////////////////////
FTP Matrix2::det() const
{
	return det2x2(&M[0]);
}

FTP Matrix2::Identity_Matrix[] =
{
	1.0, 0.0,
	0.0, 1.0
};

FTP Matrix2::det2x2(FTP a, FTP b, FTP c, FTP d)  {
	return a * d - b * c;
}

FTP Matrix2::det2x2(const FTP *m) 
{
	return m[0]*m[3] - m[1]*m[2];
}

FTP Matrix2::det3x3(FTP a1, FTP a2, FTP a3,
					FTP b1, FTP b2, FTP b3,
					FTP c1, FTP c2, FTP c3) 
{
	return a1*b2*c3 + b1*c2*a3 + c1*a2*b3 - a3*b2*c1 - b3*c2*a1 - c3*a2*b1;
}



//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix2_3*//////////////////////////////////
//////////////////////////////////////////////////////////////////////
FTP Matrix2_3::Identity_Matrix[] =
{
	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
};

FTP Matrix2_3::det2x2(FTP a, FTP b, FTP c, FTP d) {
	return a * d - b * c;
}

FTP Matrix2_3::det3x3(FTP a1, FTP a2, FTP a3,
					  FTP b1, FTP b2, FTP b3,
					  FTP c1, FTP c2, FTP c3) {
						  return a1*b2*c3 + b1*c2*a3 + c1*a2*b3
							  - a3*b2*c1 - b3*c2*a1 - c3*a2*b1;
}

//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix3*//////////////////////////////////
//////////////////////////////////////////////////////////////////////
FTP Matrix3::Identity_Matrix[] =
{
	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0
};


FTP Matrix3::det2x2(FTP a, FTP b, FTP c, FTP d)
{
	return a * d - b * c;
}

FTP Matrix3::det3x3(FTP a1, FTP a2, FTP a3,
                      FTP b1, FTP b2, FTP b3,
                      FTP c1, FTP c2, FTP c3)
{
	return a1*b2*c3 + b1*c2*a3 + c1*a2*b3
		   - a3*b2*c1 - b3*c2*a1 - c3*a2*b1;
}

FTP Matrix3::det3x3(const FTP *m)
{
	return m[0]*m[4]*m[8] + m[1]*m[5]*m[6] + m[2]*m[3]*m[7]
		   - m[6]*m[4]*m[2] - m[7]*m[5]*m[0] - m[8]*m[3]*m[1];
}

Matrix3& Matrix3::adjoint(const Matrix3& other)
{
	FTP a1, a2, a3, b1, b2, b3, c1, c2, c3;
	const FTP *in = other.M;
	FTP *out = M;

	a1 = (in [0 + ( 0 )]);	
	b1 = (in [0 + ( 1 )]);
	c1 = (in [0 + ( 2 )]);
	a2 = (in [3 + ( 0 )]);
	b2 = (in [3 + ( 1 )]);
	c2 = (in [3 + ( 2 )]);
	a3 = (in [6 + ( 0 )]);
	b3 = (in [6 + ( 1 )]);
	c3 = (in [6 + ( 2 )]);

	out[0 + ( 0 )]   =   det2x2(b2, c2, b3, c3);
	out[3 + ( 0 )]   = - det2x2(a2, c2, a3, c3);
	out[6 + ( 0 )]   =   det2x2(a2, b2, a3, b3);
	out[0 + ( 1 )]   = - det2x2(b1, c1, b3, c3);
	out[3 + ( 1 )]   =   det2x2(a1, c1, a3, c3);
	out[6 + ( 1 )]   = - det2x2(a1, b1, a3, b3);
	out[0 + ( 2 )]   =   det2x2(b1, c1, b2, c2);
	out[3 + ( 2 )]   = - det2x2(a1, c1, a2, c2);
	out[6 + ( 2 )]   =   det2x2(a1, b1, a2, b2);

	return *this;
}

Matrix3& Matrix3::invert(const Matrix3& other)
{
	int i;
	adjoint(other);
	FTP d = other.det();
	for (i=0; i<9; i++) {
		M[i] = M[i] / d;
	}
	return *this;
}

Matrix3& Matrix3::transpose(const Matrix3& other)
{
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			M[i * 3 + j] = other.M[j * 3 + i];
		}
	}
	return *this;
}

FTP Matrix3::det() const
{
	return det3x3(&M[0]);
}

FTP Matrix3::trace() const
{
	return (M[0] + M[4] + M[8]);
}

//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix4*//////////////////////////////////
//////////////////////////////////////////////////////////////////////
FTP Matrix4::Identity_Matrix[] =
{
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 1.0
};

FTP Matrix4::Orientation_Switch_Matrix[] =
{
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, -1.0, 0.0,
	0.0, 0.0, 0.0, 1.0
};

FTP Matrix4::Perspective_Matrix[] =
{
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, 1.0, -1.0,
	0.0, 0.0, 0.0, 0.0
};


Matrix4& Matrix4::setrotate(FTP rx, FTP ry, FTP rz)
{
	FTP a = Cos(rx); FTP b = Sin(rx);
	FTP c = Cos(ry); FTP d = Sin(ry);
	FTP e = Cos(rz); FTP f = Sin(rz);
	FTP ad = a * d;
	FTP bd = b * d;

	M[0] =  c * e;
	M[1] = -c * f;
	M[2] = -d;

	M[4] = -bd * e + a * f;
	M[5] =  bd * f + a * e;
	M[6] = -b * c;

	M[8] =  ad * e + b * f;
	M[9] = -ad * f + b * e;
	M[10] =  a * c;

	M[3] = M[7] = M[11] = M[12] = M[13] = M[14] = 0.0;
	M[15] = 1.0;
	return *this;
}

Matrix4& Matrix4::setrotate(FTP angle, FTP x, FTP y, FTP z)
{
	FTP xx, yy, zz, xy, yz, zx, xs, ys, zs, c_complement;
	FTP s = Sin(angle);
	FTP c = Cos(angle);
	FTP magnitude = (FTP)sqrt(x * x + y * y + z * z);
	FTP *data = M;
	if(magnitude == 0.0)
	{
		setidentity();
		return *this;
	}
	x /= magnitude;
	y /= magnitude;
	z /= magnitude;
	xx = x * x;
	yy = y * y;
	zz = z * z;
	xy = x * y;
	yz = y * z;
	zx = z * x;
	xs = x * s;
	ys = y * s;
	zs = z * s;
	c_complement = 1.0F - c;
	data[0] = (c_complement * xx) + c;
	data[4] = (c_complement * xy) - zs;
	data[8] = (c_complement * zx) + ys;
	data[12] = 0.0F;
	data[1] = (c_complement * xy) + zs;
	data[5] = (c_complement * yy) + c;
	data[9] = (c_complement * yz) - xs;
	data[13] = 0.0F;
	data[2] = (c_complement * zx) - ys;
	data[6] = (c_complement * yz) + xs;
	data[10] = (c_complement * zz) + c;
	data[14] = 0.0F;
	data[3] = 0.0F;
	data[7] = 0.0F;
	data[11] = 0.0F;
	data[15] = 1.0F;
	return *this;
}

FTP Matrix4::det() const
{
	FTP a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;
	a1 = (M [(( 0 ) << 2) + (  0 )]) ;
	b1 = (M [(( 0 ) << 2) + (  1 )]) ; 
	c1 = (M [(( 0 ) << 2) + (  2 )]) ;
	d1 = (M [(( 0 ) << 2) + (  3 )]) ;

	a2 = (M [(( 1 ) << 2) + (  0 )]) ;
	b2 = (M [(( 1 ) << 2) + (  1 )]) ; 
	c2 = (M [(( 1 ) << 2) + (  2 )]) ;
	d2 = (M [(( 1 ) << 2) + (  3 )]) ;

	a3 = (M [(( 2 ) << 2) + (  0 )]) ;
	b3 = (M [(( 2 ) << 2) + (  1 )]) ; 
	c3 = (M [(( 2 ) << 2) + (  2 )]) ;
	d3 = (M [(( 2 ) << 2) + (  3 )]) ;

	a4 = (M [(( 3 ) << 2) + (  0 )]) ;
	b4 = (M [(( 3 ) << 2) + (  1 )]) ; 
	c4 = (M [(( 3 ) << 2) + (  2 )]) ;
	d4 = (M [(( 3 ) << 2) + (  3 )]) ;

	return  a1 * det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4) -
			b1 * det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4) +
			c1 * det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4) -
			d1 * det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);
}

Matrix4& Matrix4::adjoint(const Matrix4& other)
{
	FTP a1, a2, a3, a4, b1, b2, b3, b4;
	FTP c1, c2, c3, c4, d1, d2, d3, d4;
	const FTP *in = other.M;
	FTP *out = M;

	a1 = (in [(( 0 ) << 2) + (  0 )]) ; b1 = (in [(( 0 ) << 2) + (  1 )]) ;
	c1 = (in [(( 0 ) << 2) + (  2 )]) ; d1 = (in [(( 0 ) << 2) + (  3 )]) ;
	a2 = (in [(( 1 ) << 2) + (  0 )]) ; b2 = (in [(( 1 ) << 2) + (  1 )]) ;
	c2 = (in [(( 1 ) << 2) + (  2 )]) ; d2 = (in [(( 1 ) << 2) + (  3 )]) ;
	a3 = (in [(( 2 ) << 2) + (  0 )]) ; b3 = (in [(( 2 ) << 2) + (  1 )]) ;
	c3 = (in [(( 2 ) << 2) + (  2 )]) ; d3 = (in [(( 2 ) << 2) + (  3 )]) ;
	a4 = (in [(( 3 ) << 2) + (  0 )]) ; b4 = (in [(( 3 ) << 2) + (  1 )]) ;
	c4 = (in [(( 3 ) << 2) + (  2 )]) ; d4 = (in [(( 3 ) << 2) + (  3 )]) ;

	out[(( 0 ) << 2) + (  0 )]   =   det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4);
	out[(( 1 ) << 2) + (  0 )]   = - det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4);
	out[(( 2 ) << 2) + (  0 )]   =   det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4);
	out[(( 3 ) << 2) + (  0 )]   = - det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);

	out[(( 0 ) << 2) + (  1 )]   = - det3x3( b1, b3, b4, c1, c3, c4, d1, d3, d4);
	out[(( 1 ) << 2) + (  1 )]   =   det3x3( a1, a3, a4, c1, c3, c4, d1, d3, d4);
	out[(( 2 ) << 2) + (  1 )]   = - det3x3( a1, a3, a4, b1, b3, b4, d1, d3, d4);
	out[(( 3 ) << 2) + (  1 )]   =   det3x3( a1, a3, a4, b1, b3, b4, c1, c3, c4);

	out[(( 0 ) << 2) + (  2 )]   =   det3x3( b1, b2, b4, c1, c2, c4, d1, d2, d4);
	out[(( 1 ) << 2) + (  2 )]   = - det3x3( a1, a2, a4, c1, c2, c4, d1, d2, d4);
	out[(( 2 ) << 2) + (  2 )]   =   det3x3( a1, a2, a4, b1, b2, b4, d1, d2, d4);
	out[(( 3 ) << 2) + (  2 )]   = - det3x3( a1, a2, a4, b1, b2, b4, c1, c2, c4);

	out[(( 0 ) << 2) + (  3 )]   = - det3x3( b1, b2, b3, c1, c2, c3, d1, d2, d3);
	out[(( 1 ) << 2) + (  3 )]   =   det3x3( a1, a2, a3, c1, c2, c3, d1, d2, d3);
	out[(( 2 ) << 2) + (  3 )]   = - det3x3( a1, a2, a3, b1, b2, b3, d1, d2, d3);
	out[(( 3 ) << 2) + (  3 )]   =   det3x3( a1, a2, a3, b1, b2, b3, c1, c2, c3);
	return *this;
}

Matrix4& Matrix4::transpose(const Matrix4& other)
{
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++)
			M[i * 4 + j] = other.M[j * 4 + i];
	}
	return *this;
}

Matrix4& Matrix4::invert(const Matrix4& other)
{
	int i;
	adjoint(other);
	FTP d = other.det();
	for (i = 0; i < 16; i++) M[i] = M[i] / d;
	return *this;
}

Matrix4& Matrix4::setprojection(FTP fov, FTP aspect, FTP znear, FTP zfar)
{
	FTP top = znear * Tan(fov);
	FTP bottom = -top;
	FTP left = bottom * aspect;
	FTP right = top * aspect;
	FTP x = (2.0 * znear) / (right - left);
	FTP y = (2.0 * znear) / (top - bottom);
	FTP a = (right + left) / (right - left);
	FTP b = (top + bottom) / (top - bottom);
	FTP c = -(zfar + znear) / (zfar - znear);
	FTP d = -(2.0 * zfar * znear) / (zfar - znear);
	M[0]  = x;     M[1]  = 0.0;  M[2]  = 0.0; M[3]  = 0.0;
	M[4]  = 0.0;  M[5]  = y;     M[6]  = 0.0; M[7]  = 0.0;
	M[8]  = a;     M[9]  = b;     M[10] = c;    M[11] = -1.0;
	M[12] = 0.0;  M[13] = 0.0;  M[14] = d;    M[15] = 0.0;
	return *this;
}

Matrix4& Matrix4::setothogonal(FTP znear, FTP zfar)
{
	FTP x, y, z;
	FTP tx, ty, tz;
	FTP sml = 0.0;
	x = 2.0 / (1.0 + sml);
	y = 2.0 / (1.0 + sml);
	z = -2.0 / (zfar - znear);
	tx = -(1.0 - sml) / (1.0 + sml);
	ty = -(1.0 - sml) / (1.0 + sml);
	tz = -(zfar + znear) / (zfar - znear);
	M[0] = x;    M[4] = 0.0;  M[8]  = 0.0;  M[12] = tx;
	M[1] = 0.0; M[5] = y;     M[9]  = 0.0;  M[13] = ty;
	M[2] = 0.0; M[6] = 0.0;  M[10] = z;     M[14] = tz;
	M[3] = 0.0; M[7] = 0.0;  M[11] = 0.0;  M[15] = 1.0;
	return *this;
}

FTP Matrix4::det2x2(FTP a, FTP b, FTP c, FTP d)
{
	return a * d - b * c;
}

FTP Matrix4::det3x3(FTP a1, FTP a2, FTP a3,
					  FTP b1, FTP b2, FTP b3,
					  FTP c1, FTP c2, FTP c3)
{
	return a1 * det2x2(b2, c2, b3, c3)
		 - b1 * det2x2(a2, c2, a3, c3)
		 + c1 * det2x2(a2, b2, a3, b3);
}

void Solve3x3LinSysDoolittle(FTP a[][3], FTP b[], FTP x[])
{
	FTP A[3][3];
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			A[i][j] = a[i][j];
		}
	}
	// 利用选主元的Doolittle方法求解3x3线性系统
	if (Abs(A[1][0])>Abs(A[2][0]) && Abs(A[1][0])>Abs(A[0][0]))				// the 2nd row is principal row
	{
		if (IsZeroL(A[1][0])) {
//			AfxMessageBox("Solve3x3LinSysDoolittle: no unique solution!");
			exit(1);
		} else {
			// exchange row
			FTP tmp1 = A[0][0];
			FTP tmp2 = A[0][1];
			FTP tmp3 = A[0][2];
			FTP tmp4 = b[0];
			A[0][0] = A[1][0]; A[0][1] = A[1][1]; A[0][2] = A[1][2]; b[0] = b[1];
			A[1][0] = tmp1; A[1][1] = tmp2; A[1][2] = tmp3; b[1] = tmp4;
		}
	}
	else if (Abs(A[2][0])>Abs(A[1][0]) && Abs(A[2][0])>Abs(A[0][0]))		// the 3rd row is principal row
	{
		if (IsZeroL(A[2][0])) {
//			AfxMessageBox("Solve3x3LinSysDoolittle: no unique solution!");
			exit(1);
		} else {
			// exchange row
			FTP tmp1 = A[0][0];
			FTP tmp2 = A[0][1];
			FTP tmp3 = A[0][2];
			FTP tmp4 = b[0];
			A[0][0] = A[2][0]; A[0][1] = A[2][1]; A[0][2] = A[2][2]; b[0] = b[2];
			A[2][0] = tmp1; A[2][1] = tmp2; A[2][2] = tmp3; b[2] = tmp4;
		}
	}
	
	A[1][0] = A[1][0] / A[0][0];
	A[2][0] = A[2][0] / A[0][0];
	// 选主元
	if (Abs(A[2][1] - A[2][0]*A[0][1]) > Abs(A[1][1] - A[1][0]*A[0][1])) {
		if (IsZeroL(Abs(A[2][1] - A[2][0]*A[0][1]))) {
//			AfxMessageBox("Solve3x3LinSysDoolittle: no unique solution!");
			exit(1);
		} else {
			// exchange row
			FTP tmp1 = A[1][0];
			FTP tmp2 = A[1][1];
			FTP tmp3 = A[1][2];
			FTP tmp4 = b[1];
			A[1][0] = A[2][0]; A[1][1] = A[2][1]; A[1][2] = A[2][2]; b[1] = b[2];
			A[2][0] = tmp1; A[2][1] = tmp2; A[2][2] = tmp3; b[2] = tmp4;
		} 
	} else if (IsZeroL(Abs(A[1][1] - A[1][0]*A[0][1]))) {
//		AfxMessageBox("Solve3x3LinSysDoolittle: no unique solution!");
		exit(1);
	}
	A[1][1] = A[1][1] - A[1][0]*A[0][1];
	A[1][2] = A[1][2] - A[1][0]*A[0][2];
	A[2][1] = (A[2][1] - A[2][0]*A[0][1]) / A[1][1];
	A[2][2] = A[2][2] - A[2][0]*A[0][2] - A[2][1]*A[1][2];

	x[0] = b[0];
	x[1] = b[1] - A[1][0]*x[0];
	x[2] = b[2] - A[2][0]*x[0] - A[2][1]*x[1];
	x[2] = x[2] / A[2][2];
	x[1] = (x[1] - A[1][2]*x[2]) / A[1][1];
	x[0] = (x[0] - A[0][1]*x[1] - A[0][2]*x[2]) / A[0][0];
}

void Solve3x3LinSysGaussElim(FTP a[][3], FTP b[], FTP x[])
{
	int r[3];
	FTP s[3];
	FTP c[3];
	FTP A[3][3];
	int i, j, k;

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			A[i][j] = a[i][j];
		}
	}

	for (i=0;i<3;i++)
	{
		// 行指示向量r置初值
		r[i] = i;

		// 每一行中找出s值
		s[i] = fabs(A[i][0]);
		for (j=1;j<3;j++)
		{
			if (s[i] < fabs(A[i][j]))
			{
				s[i] = fabs(A[i][j]);
			}
		}
	}

	for (k=0;k<2;k++)
	{
		// 在第k列中选主元
		c[k] = fabs(A[k][k]) / s[k];
		int R = k;
		int temp;
		for (i=k;i<3;i++)
		{
			if (c[k] < fabs(A[i][k]) / s[k])
			{
				c[k] = fabs(A[i][k]) / s[k];
				R = i;
			}
		}

		if (c[k] == 0.0)
		{
			printf("Error: No unique solution exists!\n");
			exit(-1);
		}

		if (r[R] != r[k])
		{
			temp = r[R];
			r[R] = r[k];
			r[k] = temp;
		}

		for (i=k + 1;i<3;i++)
		{
			FTP m = A[r[i]][k] / A[r[k]][k];
			for (j=k + 1;j<3;j++)
			{
				A[r[i]][j] -= m * A[r[k]][j];
			}
			b[r[i]] -= m * b[r[k]];
		}
	}

	if (A[r[2]][2] == 0.0)
	{
		printf("Error: No unique solution exists!\n");
		exit(-1);
	}

	// 开始回代
	x[2] = b[r[2]] / A[r[2]][2];

	for (i=1;i>=0;i--)
	{
		x[i] = b[r[i]];
		for (j=i + 1;j<3;j++)
		{
			x[i] -= A[r[i]][j] * x[j];
		}
		x[i] /= A[r[i]][i];
	}
}


//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix3_2*//////////////////////////////////
//////////////////////////////////////////////////////////////////////

FTP Matrix3_2::Identity_Matrix[] =
{
	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
};

FTP Matrix3_2::det2x2(FTP a, FTP b, FTP c, FTP d) {
	return a * d - b * c;
}

FTP Matrix3_2::det3x3(FTP a1, FTP a2, FTP a3,
					  FTP b1, FTP b2, FTP b3,
					  FTP c1, FTP c2, FTP c3) {
						  return a1*b2*c3 + b1*c2*a3 + c1*a2*b3
							  - a3*b2*c1 - b3*c2*a1 - c3*a2*b1;
}
//multiply

inline Matrix3 Matrix3_2::mul(const Matrix3_2& m1, const Matrix2_3& m2) const{
	Matrix3 R;
	FTP t(0.0);
	int A_nRow(3),A_nCol(2),B_nRow(2),B_nCol(3),C_nRow(3),C_nCol(3);	//assert(A_nCol==B_nRow);
	for (int i=0; i<C_nRow; ++i)
	for (int j=0; j<C_nCol; ++j){
		t = 0.0;
		for (int k=0; k<A_nCol; ++k)
			t += m1.M[i*A_nCol+k]*m2.M[k*B_nRow+j];
		R.M[i*C_nCol+j] = t;
	}
	return R;
}

inline Matrix3 Matrix3_2::mul(const Matrix2_3& other)  const {
	Matrix3_2 tmp(*this);
	return mul(tmp, other);	
}

inline Matrix3_2& Matrix3_2::mul(const Matrix3_2& m1, const Matrix2& m2) {
	FTP t(0.0);
	int A_nRow(3),A_nCol(2),B_nRow(2),B_nCol(2),C_nRow(3),C_nCol(2);	//assert(A_nCol==B_nRow);
	for (int i=0; i<C_nRow; ++i)
	for (int j=0; j<C_nCol; ++j){
		t = 0.0;
		for (int k=0; k<A_nCol; ++k)
			t += m1.M[i*A_nCol+k]*m2.M[k*B_nRow+j];
		M[i*C_nCol+j] = t;
	}
	return *this;
}

inline Matrix3_2& Matrix3_2::mul(const Matrix2& other) {
	Matrix3_2 tmp(*this);
	mul(tmp,other);
	return *this;
}

inline void Matrix3_2::mul(const Vector2& vi, Vector3& vo) const {  //vo = M * vi : 矩阵左乘向量，无返回值 3_2*2_1 = 3_1
	vo[0] = M[0]*vi[0] + M[1]*vi[1];
	vo[1] = M[2]*vi[0] + M[3]*vi[1];
	vo[2] = M[4]*vi[0] + M[5]*vi[1];
}

inline Vector3 Matrix3_2::mul(const Vector2& v) const { //vo = M * vi : 矩阵左乘向量,返回一向量
	Vector3 tmp;
	tmp[0] = M[0]*v[0] + M[1]*v[1];
	tmp[1] = M[2]*v[0] + M[3]*v[1];
	tmp[2] = M[4]*v[0] + M[5]*v[1];
	return tmp;				  
}

// operators

inline Matrix3_2& Matrix3_2::operator *=(FTP f) {
	M[0] *= f;	M[1] *= f;
	M[2] *= f;	M[3] *= f;
	M[4] *= f;	M[5] *= f;
	return *this;
}

inline Matrix3_2 Matrix3_2::operator *(FTP f) const { 
	Matrix3_2 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Matrix3 Matrix3_2::operator *=(const Matrix2_3& other) const {	
	return mul(*this, other);
}

inline Matrix3 Matrix3_2::operator *(const Matrix2_3& other) const {
	return (*this) *= other;
}

inline Matrix3_2& Matrix3_2::operator *=(const Matrix2& other){	
	Matrix3_2 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix3_2 Matrix3_2::operator *(const Matrix2& other) const {
	Matrix3_2 tmp(*this);
	tmp *= other;
	return tmp;
}

inline Matrix3_2& Matrix3_2::operator +=(const Matrix3_2& v) {
	M[0] += v.M[0]; M[1] += v.M[1]; 
	M[2] += v.M[2]; M[3] += v.M[3]; 
	M[4] += v.M[4]; M[5] += v.M[5];
	return *this;
}

inline Matrix3_2 Matrix3_2::operator +(const Matrix3_2& other) const {
	Matrix3_2 tmp(*this);
	tmp += other;
	return tmp;
}

inline Matrix3_2& Matrix3_2::operator -=(const Matrix3_2& v) {
	M[0] -= v.M[0];	M[1] -= v.M[1];	
	M[2] -= v.M[2];	M[3] -= v.M[3]; 
	M[4] -= v.M[4]; M[5] -= v.M[5];
	return *this;
}

inline Matrix3_2 Matrix3_2::operator -(const Matrix3_2& other) const {
	Matrix3_2 tmp(*this);
	tmp -= other;
	return tmp;
}

inline Matrix3_2& Matrix3_2::operator /=(FTP f) {
	FTP rf = 1 / f;
	M[0] *= rf;	M[1] *= rf;	
	M[2] *= rf;	M[3] *= rf;	
	M[4] *= rf;	M[5] *= rf;
	return *this;
}

inline Matrix3_2 Matrix3_2::operator /(FTP f) const { 
	Matrix3_2 tmp(*this);
	tmp /= f;
	return tmp;
}


inline int Matrix3_2::operator==(const Matrix3_2& m) const { 
	for (int i=0; i<6; i++) {
		if (!IsEqual(M[i], m.M[i]))		return false;
	}
	return true;
}

inline int Matrix3_2::operator!=(const Matrix3_2& m) const { 
	for (int i=0; i<6; i++) {
		if (!IsEqual(M[i], m.M[i]))		return true;
	}
	return false;
}

inline int operator==(const Matrix3_2& m1, const Matrix3_2& m2) {
	for (int i=0; i<6; i++) {
		if (!IsEqual(m1.M[i], m2.M[i]))		return false;
	}
	return true;
}

inline Matrix2_3 Matrix3_2::transpose() const {
	//Matrix3_2 tmp(*this);
	return transpose(*this);
}

inline Matrix2_3 Matrix3_2::transpose(const Matrix3_2& other) const {
	Matrix2_3 R;
	for (int i=0; i<3; i++) {
		for (int j=0; j<2; j++) {
			R.M[j * 3 + i] = other.M[i * 2 + j];
		}
	}
	return R;
}

inline	Matrix3 Vector3::RotationMatrix( FTP degree )
{
	Vector3 axis = *this;
	axis.normalize();
	
	FTP rad = degree * ML_DEG_TO_RAD;

	double fCos=cos( rad );
	double fSin=sin( rad );

	Matrix3 rm;

	rm.m[0][0] = ( axis.x * axis.x ) * ( 1.0f - fCos ) + fCos;
	rm.m[0][1] = ( axis.x * axis.y ) * ( 1.0f - fCos ) - (axis.z * fSin);
	rm.m[0][2] = ( axis.x * axis.z ) * ( 1.0f - fCos ) + (axis.y * fSin);

	rm.m[1][0] = ( axis.y * axis.x ) * ( 1.0f - fCos ) + (axis.z * fSin);
	rm.m[1][1] = ( axis.y * axis.y ) * ( 1.0f - fCos ) + fCos ;
	rm.m[1][2] = ( axis.y * axis.z ) * ( 1.0f - fCos ) - (axis.x * fSin);

	rm.m[2][0] = ( axis.z * axis.x ) * ( 1.0f - fCos ) - (axis.y * fSin);
	rm.m[2][1] = ( axis.z * axis.y ) * ( 1.0f - fCos ) + (axis.x * fSin);
	rm.m[2][2] = ( axis.z * axis.z ) * ( 1.0f - fCos ) + fCos;
	
	return rm;
}


Matrix3 RotationMatrix( const Vector3 &from, const Vector3 &to)
{
	Matrix3 rm;
	Vector3 diff = from-to;
	if( fabs(diff.x)<ML_LOW_TOLERANCE && fabs(diff.x)<ML_LOW_TOLERANCE && fabs(diff.x)<ML_LOW_TOLERANCE )	return rm.setidentity();

	Vector3 axis = from.cross(to).normalize();
	
	FTP ang_rad = acos( from.dot(to)/from.magnitude()/to.magnitude() );

	double fCos=cos( ang_rad );
	double fSin=sin( ang_rad );

	rm.m[0][0] = ( axis.x * axis.x ) * ( 1.0f - fCos ) + fCos;				
	rm.m[0][1] = ( axis.x * axis.y ) * ( 1.0f - fCos ) - (axis.z * fSin);	
	rm.m[0][2] = ( axis.x * axis.z ) * ( 1.0f - fCos ) + (axis.y * fSin);	

	rm.m[1][0] = ( axis.y * axis.x ) * ( 1.0f - fCos ) + (axis.z * fSin);	
	rm.m[1][1] = ( axis.y * axis.y ) * ( 1.0f - fCos ) + fCos ;				
	rm.m[1][2] = ( axis.y * axis.z ) * ( 1.0f - fCos ) - (axis.x * fSin);	

	rm.m[2][0] = ( axis.z * axis.x ) * ( 1.0f - fCos ) - (axis.y * fSin);	
	rm.m[2][1] = ( axis.z * axis.y ) * ( 1.0f - fCos ) + (axis.x * fSin);	
	rm.m[2][2] = ( axis.z * axis.z ) * ( 1.0f - fCos ) + fCos;				
	
	return rm;
}
