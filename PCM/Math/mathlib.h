
// ------------------------------------------------------------
// Math Library 
// ------------------------------------------------------------
// Chris Stephenson 
// ------------------------------------------------------------
// Enriched by ShizheZhou @ 09-8-11 

// Enriched by Xiaoguang Han @ 13-7-1
// ------------------------------------------------------------
// This library is mostly pieced together from other inseficiant 
// free sources I have found (mostly www.flipcode.com code of the day section)
// All angles in degrees.(radians suck!)
// Everything else is basic.
// 
// For the Matrix classes, transfomations dan be writen like this:
// {
//    Matrix4 m;
//    m.settranslate(1, -2, 3).rotate(40, 1, 0.5, 4).scale(0.5, 2, 1.3);
// }
// Instead of this:
// {
//    Matrix4 m;
//    m.settranslate(1, -2, 3)
//    m.rotate(40, 1, 0.5, 4)
//    m.scale(0.5, 2, 1.3);
// }
// ------------------------------------------------------------

#pragma once

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// PI & conversions

#define FTP double

const double ML_PI				= 3.141592653589793238;
const double ML_DEG_TO_RAD		= 0.017453292519943295;
const double ML_RAD_TO_DEG		= 57.29577951308232286;
const double ML_PI_DIV_2		= 1.570796326794896558;
const double ML_PI_DIV_4		= 0.785398163397448279;

// tolerance values

const float ML_LOW_TOLERANCE		= 0.000001f;
const float ML_TOLERANCE			= 0.000100f;
const float ML_HIGH_TOLERANCE		= 0.010000f;

const double ML_D_LOW_TOLERANCE		= 0.0000000001;
const double ML_D_TOLERANCE			= 0.0000000010;
const double ML_D_HIGH_TOLERANCE	= 0.0000000100;

const double RADIAN_TO_ANGLE  = 57.295779513;

// ------------------------------------------------------------
// basic functions
// ------------------------------------------------------------

inline FTP Clamp(FTP f, FTP min, FTP max){ return min > f ? min : (max > f ? f : max);}
inline int Clamp(int f, int min, int max){ return min > f ? min : (max > f ? f : max);}
inline FTP Lerp(FTP f, FTP from, FTP to){ return from + (to - from) * f;}
inline FTP Min(FTP f1, FTP f2){ return f1 < f2 ? f1 : f2;}
inline FTP Max(FTP f1, FTP f2){ return f1 > f2 ? f1 : f2;}
inline FTP Abs(FTP f){return f > 0 ? f : -f;}
inline FTP Sin(FTP f){ return sin(f * ML_DEG_TO_RAD);}
inline FTP Cos(FTP f){ return cos(f * ML_DEG_TO_RAD);}
inline FTP Tan(FTP f){ return tan(f * ML_DEG_TO_RAD);}
inline FTP Asin(FTP f){ return atan(f / sqrt(-f * f + 1)) * ML_RAD_TO_DEG;}			/*[-1,0, 1] --> [-90,0,90]*/
inline FTP Acos(FTP f){ return atan(-f / sqrt(-f * f + 1)) * ML_RAD_TO_DEG + 90.0;}	/*[-1,0, 1] --> [180,90,0]*/
inline FTP AsinR(FTP f){ return atan(f / sqrt(-f * f + 1));} 
inline FTP AcosR(FTP f){ return atan(-f / sqrt(-f * f + 1)) + ML_PI_DIV_2;}
inline FTP Atan(FTP f){ return atan(f) * ML_DEG_TO_RAD;}
inline FTP AtanR(FTP f){ return atan(f);}
inline FTP Degree(FTP f){ return f * ML_DEG_TO_RAD;}
inline FTP Radians(FTP f){ return f * ML_RAD_TO_DEG;}
inline FTP Random(){ return (FTP)(rand()) / (FTP)(RAND_MAX);}
inline FTP Random(FTP f){ return Random() * f;}
inline FTP Random(FTP f1, FTP f2){ return Random() * (f2 - f1) + f1;}
inline void SeedRand(unsigned int seed){srand(seed);};

inline bool IsZeroL(FTP f){ return fabs(f) < ML_LOW_TOLERANCE;}
inline bool IsZero(FTP f){ return fabs(f) < ML_TOLERANCE;}
inline bool IsZeroH(FTP f){ return fabs(f) < ML_HIGH_TOLERANCE;}
inline bool IsEqualL(FTP f1, FTP f2){ return fabs(f1 - f2) < ML_LOW_TOLERANCE;}
inline bool IsEqual(FTP f1, FTP f2){ return fabs(f1 - f2) < ML_TOLERANCE;}
inline bool IsEqualH(FTP f1, FTP f2){ return fabs(f1 - f2) < ML_HIGH_TOLERANCE;}

inline bool ValidRange(FTP a, FTP low, FTP up){ return (a>=low&&a<=up);	}
inline bool ValidRange(int a, int low, int up){ return (a>=low&&a<=up);	}

void Solve3x3LinSysDoolittle(FTP a[][3], FTP b[], FTP x[]);
void Solve3x3LinSysGaussElim(FTP a[][3], FTP b[], FTP x[]);


inline int AccumSum(int f1, int f2){  /*sum: f1 + (f1-1) + (f1-2) + ... + f2*/
	int sum = 0;
	if(f1==f2) return f1;
	else if(f1>f2)
		for (int i=f1; i>=f2; --i)		sum+=i;
	else if(f1<f2)
		for (int i=f1; i<=f2; ++i)		sum+=i;
	return sum;
}

inline int MinIdx(FTP f0, FTP f1, FTP f2){
	if(f0<f1){
		if(f0<f2)	return 0;		else		return 2;			
	}
	else{
		if(f1<f2)	return 1;		else		return 2;			
	}
}

inline int MaxIdx(FTP f0, FTP f1, FTP f2){
	if(f0>f1){
		if(f0>f2)	return 0;		else		return 2;			
	}
	else{
		if(f1>f2)	return 1;		else		return 2;			
	}
}

// ------------------------------------------------------------
// classes
// ------------------------------------------------------------

class Vector2;
class Vector3;
class Matrix2;
class Matrix2_3;
class Matrix3_2;
class Matrix3;
class Matrix4;
class Plane;

// ------------------------------------------------------------
// Vector2
// ------------------------------------------------------------

class Vector2
{
public:

	//members

	union 
	{
		struct { FTP x, y; };
		struct { FTP v[2]; };
	};

	// constructors

	Vector2();
	Vector2(const FTP* args);
	Vector2(FTP _x, FTP _y);
	Vector2(const Vector2& v);

	// set

	void set(FTP _x, FTP _y);
	void set(const Vector2& v);
	void set(FTP* args);

	// Data access using indices
	FTP&       operator[](int i)       { return (v[i]); }
	const FTP& operator[](int i) const { return (v[i]); }

	// operators

	Vector2 operator *(FTP f) const;
	Vector2 operator /(FTP f) const;
	Vector2 operator +(const Vector2& v) const;
	Vector2 operator -(const Vector2& v) const;
	Vector2 operator -() const;
	Vector2& operator *=(FTP f);
	Vector2& operator /=(FTP f);
	Vector2& operator +=(const Vector2& v);
	Vector2& operator -=(const Vector2& v);
	int operator==(const Vector2& v);
	int operator!=(const Vector2& v);
	Vector2& operator=(const Vector2& v);
	Vector2&  operator&(const Vector2& v); // reverse copy
	Matrix2_3 operator*(const Vector3& v) const;
	
	//////////////////三角函数///////////////
	// sin of real angle between the 2 vector 
	inline double sin_V2(const Vector2 &a){
		return fabs(this->x*a.y - this->y*a.x) / (magnitude() * a.magnitude() );
	}	
	// cos of real angle between the 2 vector 
	inline double cos_V2(const Vector2 &a){
		return dot(a) / (magnitude() * a.magnitude() );
	}	
	// ctg of real angle between the 2 vector
	inline double ctg_V2(const Vector2 &a)	{
		return  dot(a)/ fabs(this->x*a.y - this->y*a.x);
	}
	// tg of real angle between the 2 vector
	inline double tan_V2(const Vector2 &a){
		return fabs(this->x*a.y - this->y*a.x)/dot(a);
	}

	// linear algebra
	FTP sqauremagnitude() const;
	FTP magnitude() const;
	FTP sqauredistance(const Vector2& v) const;
	FTP distance(const Vector2& v) const;
	void normalize(); 
	void normalize(const Vector2& v);
	FTP dot(const Vector2& v) const;
};

// ------------------------------------------------------------
// Vector3
// ------------------------------------------------------------

class Vector3
{
public:

	// members

	union 
	{
		struct { FTP x, y, z; };
		struct { FTP v[3]; };
	};

	// constructors

	Vector3();
	Vector3(const FTP* args);
	Vector3(FTP _x, FTP _y, FTP _z);
	Vector3(const Vector3& v);

	// set

	void set(const FTP* args);
	void set(FTP _x, FTP _y, FTP _z);
	void set(const Vector3& v);
	void set(const Vector3* v);
	// Per coordinate (explicit inline functions)
	void setx(FTP newX) { v[0] = newX; }
	void sety(FTP newY) { v[1] = newY; }
	void setz(FTP newZ) { v[2] = newZ; }

	// Data access using indices
	FTP&       operator[](int i)       { return (v[i]); }
	const FTP& operator[](int i) const { return (v[i]); }

	// operators

	Vector3 operator *(FTP f) const;
	Vector3 operator /(FTP f) const;
	Vector3 operator +(const Vector3& v) const;
	Vector3 operator -(const Vector3& v) const;
	Vector3 operator -() const;
	Vector3& operator *=(FTP f);
	Vector3& operator /=(FTP f);
	Vector3& operator +=(const Vector3& v);
	Vector3& operator -=(const Vector3& v);
	int operator==(const Vector3& v);
	int operator!=(const Vector3& v);
	friend int operator==(const Vector3& v1, const Vector3& v2);
	friend int operator!=(const Vector3& v1, const Vector3& v2) { return !(v1 == v2); }
	Vector3& operator=(const Vector3& v);
	Vector3& operator&(const Vector3& v); // reverse copy
	Matrix3 operator *(const Vector3& v)const;

	// linear algebra
	FTP sqauremagnitude() const;
	FTP magnitude() const;
	FTP sqauredistance(const Vector3& v) const;
	FTP distance(const Vector3& v) const;
	Vector3& normalize(); 
	Vector3 normalize() const; 
	void normalize(const Vector3& v);
	FTP dot(const Vector3& v) const;
	Vector3 cross(const Vector3& v) const;
	Matrix3 square() const;
	Matrix3 RotationMatrix( FTP degree );

	/*template <class charT, class traits>
	friend std::basic_ostream<charT,traits>& operator<<(std::basic_ostream<charT,traits>& os, const Vector3 &v)
	{
		os << " ";
		for(int i = 0; i < 3; ++i) {		
			os << v[i];
			if(i < 2)	os << " ";
		}
		os << " ";
		return os;
	}

	template <class charT, class traits>
	friend std::basic_istream<charT,traits>& operator>>(std::basic_istream<charT,traits>& os, Vector3 &v)
	{
		for(int i = 0; i < 3; ++i) {
			os >> v[i];
		}
		return os;
	}
	*/
};

// ------------------------------------------------------------
// Matrix2
// 0 1 
// 2 3 
// ------------------------------------------------------------

class Matrix2
{
public:

	// members

	union
	{
		FTP M[4];
		FTP m[2][2];
	};

	static FTP Identity_Matrix[];

	// constructors

	Matrix2();
	Matrix2(const FTP* other);
	Matrix2(const Matrix2& other);

	// set

	Matrix2& operator=(const Matrix2& other);
	void set(const FTP* other);
	void set(const Matrix2& other);
	void setcols(const FTP* other);
	void setdiag(const FTP* diag);
	void setdiag(FTP d1, FTP d2);
	void setorthocol(int c1);
	Matrix2& setidentity();
	Matrix2& setzero();

	// simple methods

	bool iszero();
	bool isrotate();

	// this *= other
	Matrix2& mul(const Matrix2& other);
	// this = m1 * m2
	Matrix2& mul(const Matrix2& m1, const Matrix2& m2);
	// column c *= f
	Matrix2& mulcol(int c, FTP f);
	// this = other * f
	Matrix2& mul(const Matrix2& other, FTP f);
	// vo = this * vi
	void mul(const Vector2& vi, Vector2& vo);
	Vector2 mul(const Vector2& v) const;
	// operators
	Matrix2_3	operator*(const Matrix2_3& other) const;
	Vector2		operator*(const Vector2& other) const;
	Matrix2		operator*(const Matrix2& other) const;
	Matrix2&	operator*=(const Matrix2& other);
	Matrix2		operator*(FTP f) const;
	Matrix2&	operator*=(FTP f);
	Matrix2		operator/(FTP f) const;
	Matrix2&	operator/=(FTP f);
	Matrix2		operator +(const Matrix2& v) const;
	Matrix2&	operator+=(const Matrix2& other);
	Matrix2		operator -(const Matrix2& v) const;
	Matrix2&	operator-=(const Matrix2& other);
	int operator==(const Matrix2& m);
	int operator!=(const Matrix2& m);
	friend int operator==(const Matrix2& m1, const Matrix2& m2);
	friend int operator!=(const Matrix2& m1, const Matrix2& m2) { return !(m1 == m2); }

	// other methods
	void squarecolsymm(FTP *symm);
	void squareupper(Matrix2& out);
	Matrix2& adjoint();//求伴随
	Matrix2& adjoint(const Matrix2& other);
	Matrix2& invert(const Matrix2& other);
	Matrix2& invert_exp();
	Matrix2& transpose();
	Matrix2& transpose(const Matrix2& other);
	
	static FTP det2x2(FTP a, FTP b, FTP c, FTP d);
	static FTP det2x2(const FTP *m);
	static FTP det3x3(FTP a1, FTP a2, FTP a3,
		FTP b1, FTP b2, FTP b3,
		FTP c1, FTP c2, FTP c3);
	static FTP det3x3(const FTP *m);

	FTP det() const;
	FTP trace() const;
	/*
	template <class charT, class traits>
	friend std::basic_ostream<charT,traits>& operator<<(std::basic_ostream<charT,traits>& os, const Matrix2 &mtx)
	{
		os << " ";
		for(int i = 0; i < 4; ++i) 
			os << mtx.M[i]<< " ";
		return os;
	}*/
};

// ------------------------------------------------------------
// Matrix3
// 0 1 2
// 3 4 5
// 6 7 8
// ------------------------------------------------------------

class Matrix3
{
public:

	// members

	union
	{
		FTP M[9];
		FTP m[3][3];
	};

	static FTP Identity_Matrix[];

	// constructors

	Matrix3();
	Matrix3(const FTP* other);
	Matrix3(const Matrix3& other);
	Matrix3(const Vector3& r1,const Vector3& r2,const Vector3& r3);
	// set

	Matrix3& operator=(const Matrix3& other);
	void set(const FTP* other);
	void setcols(const FTP* other);
	void setorthocol(int c);
	void setorthocol(int c1, int c2);
	void setdiag(const FTP* diag);
	void setdiag(FTP d1, FTP d2, FTP d3);
	void set(const Matrix3& other);

	// simple methods

	Matrix3& setidentity();
	Matrix3& setzero();
	bool iszero();

	// this *= other
	Matrix3& mul(const Matrix3& other);
	// this = m1 * m2
	Matrix3& mul(const Matrix3& m1, const Matrix3& m2);
	// column c *= f
	Matrix3& mulcol(int c, FTP f);
	// this = other * f
	Matrix3& mul(const Matrix3& other, FTP f);
	// vo = this * vi
	void mul(const Vector3& vi, Vector3& vo);
	Vector3 mul(const Vector3& v) const;

	// operators
	Vector3 operator *(const Vector3& v) const;
	Matrix3 operator*(const Matrix3& other) const;
	Matrix3& operator*=(const Matrix3& other);
	Matrix3 operator*(FTP f) const;
	Matrix3& operator*=(FTP f);
	Matrix3 operator/(FTP f) const;
	Matrix3& operator/=(FTP f);
	Matrix3 operator +(const Matrix3& v) const;
	Matrix3& operator+=(const Matrix3& other);
	Matrix3 operator -(const Matrix3& v) const;
	Matrix3& operator-=(const Matrix3& other);
	int operator==(const Matrix3& m);
	int operator!=(const Matrix3& m);
	friend int operator==(const Matrix3& m1, const Matrix3& m2);
	friend int operator!=(const Matrix3& m1, const Matrix3& m2) { return !(m1 == m2); }
	
	// transformation methods

	Matrix3& translate(FTP tx, FTP ty);
	Matrix3& settranslate(FTP tx, FTP ty);
	Matrix3& rotate(FTP r);
	Matrix3& setrotate(FTP r);
	Matrix3& scale(FTP sx, FTP sy);
	Matrix3& setscale(FTP sx, FTP sy);
	bool isrotate();
	// Actions with Vector2

	Vector2 transform(const Vector2& v);

	static FTP det2x2(FTP a, FTP b, FTP c, FTP d);
	static FTP det3x3(FTP a1, FTP a2, FTP a3,
						FTP b1, FTP b2, FTP b3,
						FTP c1, FTP c2, FTP c3);
	static FTP det3x3(const FTP *m);

	Matrix3& adjoint();
	Matrix3& adjoint(const Matrix3& other);
	Matrix3& transpose();
	Matrix3& transpose(const Matrix3& other);
	Matrix3& invert_exp();
	Matrix3& invert_imp1();
	Matrix3& invert_imp2();
	Matrix3& invert(const Matrix3& other);
	void squarecolsymm(FTP *symm);
	void squareupper(Matrix3& out);

	FTP det() const;
	FTP trace() const;
	/*
	template <class charT, class traits>
	friend std::basic_ostream<charT,traits>& operator<<(std::basic_ostream<charT,traits>& os, const Matrix3 &mtx)
	{
		os << " ";
		for(int i=0; i<9; ++i) 
			os << mtx.M[i]<< " ";
		return os;
	}

	template <class charT, class traits>
	friend std::basic_istream<charT,traits>& operator>>(std::basic_istream<charT, traits>& is, Matrix3 &mtx)
	{	
		for(int i=0; i<9; ++i)	is >> mtx.M[i];
		return is;
	}*/
};

Matrix3 RotationMatrix(const Vector3 &from, const Vector3 &to);

// ------------------------------------------------------------
// Matrix2_3
//  0  1  2 
//  3  4  5
// ------------------------------------------------------------
	
class Matrix2_3
{
public:
	// members
	union
	{ 
		FTP M[6];  
		FTP m[2][3];
	};
	// static
	static FTP Identity_Matrix[];
	static FTP det2x2(FTP a, FTP b, FTP c, FTP d);
	static FTP det3x3(FTP a1, FTP a2, FTP a3,
		FTP b1, FTP b2, FTP b3,
		FTP c1, FTP c2, FTP c3);

public:
	Matrix2_3(){};		/* all zero by default */
	Matrix2_3(const FTP* other);
	Matrix2_3(const Matrix2_3& other){
		for (int i=0; i<6; ++i)		M[i] = other.M[i];
	};

	// set

	Matrix2_3& operator=(const Matrix2_3& other);
	void set(const FTP* other);
	void set(const Matrix2_3& other);
	
	Matrix2_3& setidentity();
	Matrix2_3& setzero();
	Matrix2_3& setperspective();
	Matrix2_3& setswitchorientation();

	//mul 
	
	Matrix2 mul(const Matrix2_3& m1, const Matrix3_2& m2) const;
	Matrix2 mul(const Matrix3_2& other) const;	//2_2  =  2_3 * 3_2
	Matrix2_3& mul(const Matrix2_3& m1, const Matrix3& m2);
	Matrix2_3& mul(const Matrix3& other);	//2_3  =  2_3 * 3_32
	void	mul(const Vector3& vi, Vector2& vo) const;  //vo = M * vi : 矩阵左乘向量，无返回值 2_3*3_1 = 2_1
	Vector2 mul(const Vector3& v) const; //vo = M * vi : 矩阵左乘向量,返回一向量
	
	// Operators
	Matrix2_3& operator *=(FTP f);
	Matrix2_3  operator *(FTP f) const;
	Vector2	   operator *(const Vector3& v)const;  /* 2_1 = 2_3 * 3*1 */
	Matrix2_3& operator +=(const Matrix2_3& v);
	Matrix2_3  operator +(const Matrix2_3& other) const;
	Matrix2_3& operator -=(const Matrix2_3& v);
	Matrix2_3  operator -(const Matrix2_3& other) const;
	Matrix2_3& operator /=(FTP f);
	Matrix2_3  operator /(FTP f) const;
	int operator==(const Matrix2_3& m) const;
	int operator!=(const Matrix2_3& m) const;
	friend int operator==(const Matrix2_3& m1, const Matrix2_3& m2);
	
	Matrix2		operator *=(const Matrix3_2& other);
	Matrix2		operator *(const Matrix3_2& other) const;
	Matrix2_3& operator *=(const Matrix3& other);
	Matrix2_3  operator *(const Matrix3& other) const;
	

	// Transpose
	Matrix3_2 transpose() const;
	Matrix3_2 transpose(const Matrix2_3& other) const;

/*functions need to call MatLab */
	void svd(Matrix2 &c_U, double* c_D,Matrix3 &c_V )  /*mwArray.getData return by colume,so this->M = c_U*c_D*c_V */
	{

	}


};

// ------------------------------------------------------------
// Matrix3_2
//  0  1
//  2  3
//	4  5
// ------------------------------------------------------------

class Matrix3_2
{
public:
	// members
	union
	{ 
		FTP M[6];  
		FTP m[3][2];
	};
	
	//static
	static FTP Identity_Matrix[];
	static FTP det2x2(FTP a, FTP b, FTP c, FTP d);
	static FTP det3x3(FTP a1, FTP a2, FTP a3,
		FTP b1, FTP b2, FTP b3,
		FTP c1, FTP c2, FTP c3);
public:

	// constructors

	Matrix3_2();
	Matrix3_2(const FTP* other);
	Matrix3_2(const Matrix3_2& other);

	// set

	Matrix3_2& operator=(const Matrix3_2& other);
	void set(const FTP* other);
	void set(const Matrix3_2& other);

	// Simple methods

	Matrix3_2& setidentity();
	Matrix3_2& setzero();
	Matrix3_2& setperspective();
	Matrix3_2& setswitchorientation();

	//mul : 2_22  =  2_3 * 3_2
	
	Matrix3 mul(const Matrix2_3& other) const ;
	Matrix3 mul(const Matrix3_2& m1, const Matrix2_3& m2) const ;
	Matrix3_2& mul(const Matrix2& other) ;
	Matrix3_2& mul(const Matrix3_2& m1, const Matrix2& m2) ;
	Vector3 mul(const Vector2& v) const ;  
	void mul(const Vector2& vi, Vector3& vo) const;
	// Operators
	
	Matrix3 operator*=(const Matrix2_3& other)const;
	Matrix3 operator*(const Matrix2_3& other) const;
	Matrix3_2& operator *=(const Matrix2& other);
	Matrix3_2 operator *(const Matrix2& other) const ;
	Matrix3_2& operator*=(FTP f);
	Matrix3_2 operator*(FTP f) const;
	Matrix3_2& operator/=(FTP f);
	Matrix3_2 operator/(FTP f) const;
	Matrix3_2& operator +=(const Matrix3_2& v) ;
	Matrix3_2 operator +(const Matrix3_2& other) const ; 
	Matrix3_2& operator -=(const Matrix3_2& v) ;
	Matrix3_2 operator -(const Matrix3_2& other) const ; 
	inline int operator==(const Matrix3_2& m) const ;
	inline int operator!=(const Matrix3_2& m) const;
	friend int operator==(const Matrix3_2& m1, const Matrix3_2& m2);

	// Transpose
	Matrix2_3 transpose() const;
	Matrix2_3 transpose(const Matrix3_2& other) const;
};

// ------------------------------------------------------------
// Matrix4
//  0  1  2  3
//  4  5  6  7
//  8  9 10 11
// 12 13 14 15
// ------------------------------------------------------------

class Matrix4
{
public:

	// members

	union
	{
		FTP M[16];
		FTP m[4][4];
	};

	static FTP Identity_Matrix[];
	static FTP Orientation_Switch_Matrix[];
	static FTP Perspective_Matrix[];

public:

	// constructors

	Matrix4();
	Matrix4(const FTP* other);
	Matrix4(const Matrix4& other);

	// set

	Matrix4& operator=(const Matrix4& other);
	void set(const FTP* other);
	void set(const Matrix4& other);

	// Simple methods

	Matrix4& setidentity();
	Matrix4& setzero();
	Matrix4& setperspective();
	Matrix4& setswitchorientation();

	// this *= other
	Matrix4& mul(const Matrix4& other);
	// this = m1 * m2
	Matrix4& mul(const Matrix4& m1, const Matrix4& m2);

	// Operators

	Matrix4& operator*=(const Matrix4& other);
	Matrix4 operator*(const Matrix4& other) const;
	Matrix4 operator*(FTP f) const;
	Matrix4& operator*=(FTP f);
	Matrix4 operator/(FTP f) const;
	Matrix4& operator/=(FTP f);

	// Transformation methods

	Matrix4& settranslate(FTP tx, FTP ty, FTP tz);
	Matrix4& translate(FTP tx, FTP ty, FTP tz);
	Matrix4& setscale(FTP sx, FTP sy, FTP sz);
	Matrix4& scale(FTP sx, FTP sy, FTP sz);

	Matrix4& setrotate(const Matrix3& matrix);
	void getrotate(Matrix3 &matrix);
	// rotation around three euler-angles
	Matrix4& setrotate(const Vector3& r);
	Matrix4& setrotate(FTP rx, FTP ry, FTP rz);
	Matrix4& rotate(const Vector3& r);
	Matrix4& rotate(FTP rx, FTP ry, FTP rz);

	// rotation euler-angle around axis
	Matrix4& setrotate(FTP angle, const Vector3& r);
	Matrix4& setrotate(FTP angle, FTP x, FTP y, FTP z);
	Matrix4& rotate(FTP angle, const Vector3& r);
	Matrix4& rotate(FTP angle, FTP x, FTP y, FTP z);

	// Invert/Transpose

	Matrix4& adjoint();
	Matrix4& adjoint(const Matrix4& other);

	Matrix4& transpose();
	Matrix4& transpose(const Matrix4& other);

	Matrix4& invert();
	Matrix4& invert(const Matrix4& other);

	FTP det() const;

	// Perpsective

	Matrix4& setprojection(FTP fov, FTP aspect, FTP znear, FTP zfar);
	Matrix4& setothogonal(FTP znear, FTP zfar);

	// static

	static FTP det2x2(FTP a, FTP b, FTP c, FTP d);
	static FTP det3x3(FTP a1, FTP a2, FTP a3,
		FTP b1, FTP b2, FTP b3,
		FTP c1, FTP c2, FTP c3);

	// Actions with Vector3

	Vector3 transform(const Vector3& v);
	inline void reform_1(Matrix3& outM, Vector3& outV);  //there will be reform_2,reform_3 in the future
};

// ------------------------------------------------------------
// Plane
// ------------------------------------------------------------

class Plane
{
public:

	// members 

	union 
	{
		struct { FTP a, b, c, d; };
		struct { Vector3 n; FTP d; };
	};

	// constructors

	Plane();
	Plane(const Vector3& _n, FTP _d);
	Plane(FTP _a, FTP _b, FTP _c, FTP _d);
	Plane(FTP* args);
	Plane(const Plane& p);

	// set

	void set(const Vector3& _n, FTP _d);
	void set(FTP _a, FTP _b, FTP _c, FTP _d);
	void set(FTP* args);
	void set(const Plane& p);

	// operators

	Plane& operator=(const Plane& p);

	// interactor width vector3

	FTP distance(const Vector3& v) const;
	Vector3 reflect(const Vector3& v);
	Vector3 intersect(const Vector3& start, const Vector3& end);
};

template<class REAL = double>
class Quaternion //normalized quaternion for representing rotations
{
public:
	//constructors
	Quaternion() : r(1.) { } //initialize to identity
	Quaternion(const Quaternion &q) : r(q.r), v(q.v) {} //copy constructor
	template<class R> Quaternion(const Quaternion<R> &q) : r(q.r), v(q.v) {} //convert quaternions of other types
	//axis angle constructor:
	template<class R> Quaternion(const Vector3 &axis, const R &angle) : r(cos(double(angle * 0.5))), v( axis.normalize() * sin(double(angle * 0.5)) ) {}
	//minimum rotation constructor:
	template<class R> Quaternion(const Vector3 &from, const Vector3 &to) : r(1.)
	{
		R fromLenSq = from.sqauremagnitude(), toLenSq = to.sqauremagnitude();
		if(fromLenSq < toLenSq) {
			if(fromLenSq < R(1e-16))
				return;
			Vector3 mid = from * sqrt(toLenSq / fromLenSq) + to;
			R fac = 1. / sqrt(mid.sqauremagnitude() * toLenSq);
			r = (mid * to) * fac;
			v = (mid % to) * fac;
		}
		else {
			if(toLenSq < R(1e-16))
				return;
			Vector3 mid = from + to * sqrt(fromLenSq / toLenSq);
			R fac = 1. / sqrt(mid.sqauremagnitude() * fromLenSq);
			r = (from * mid) * fac;
			v = (from % mid) * fac;
		}
	}

	//quaternion multiplication
	Quaternion operator*(const Quaternion &q) const { return Quaternion(r * q.r - v.dot(q.v),  q.v*r + v * q.r + v.cross(q.v) ); }

	//transforming a vector
	Vector3 operator*(const Vector3 &p) const
	{
		Vector3 v2 = v + v;
		Vector3 vsq2 = Vector3(v[0]*v2[0],  v[1]*v2[1], v[2]*v2[2]);
		Vector3 rv2 = v2*r;
		Vector3 vv2(v[1] * v2[2], v[0] * v2[2], v[0] * v2[1]);
		return Vector3(	p[0] * (REAL(1.) - vsq2[1] - vsq2[2]) + p[1] * (vv2[2] - rv2[2]) + p[2] * (vv2[1] + rv2[1]),
						p[1] * (REAL(1.) - vsq2[2] - vsq2[0]) + p[2] * (vv2[0] - rv2[0]) + p[0] * (vv2[2] + rv2[2]),
						p[2] * (REAL(1.) - vsq2[0] - vsq2[1]) + p[0] * (vv2[1] - rv2[1]) + p[1] * (vv2[0] + rv2[0]));
	}

	//equality
	template<class R> bool operator==(const Quaternion<R> &oth) const
	{
		return (r == oth.r && v == oth.v) || (r == -oth.r && v == -oth.v);
	}

	Quaternion inverse() const { return Quaternion(-r, v); }

	REAL getAngle() const { return REAL(2.) * atan2(v.magnitude(), r); }
	Vector3 getAxis() const { return v.normalize(); }

	const REAL &operator[](int i) const { return (i == 0) ? r : v[i - 1]; }
	void set(const REAL &inR, const Vector3 &inV) {
		REAL ratio = REAL(1.) / sqrt(inR * inR + inV.sqauremagnitude()); 
		r = inR * ratio; v = inV * ratio; //normalize
	}
	/*
	template <class charT, class traits>
	friend std::basic_ostream<charT,traits>& operator<<(std::basic_ostream<charT,traits>& os, const Quaternion &q)
	{
		os<<q.getAngle()<<q.getAxis();
		return os;
	};
	template <class charT, class traits>
	friend std::basic_istream<charT,traits>& operator>>(std::basic_istream<charT,traits>& is,const Quaternion &q)
	{
		REAL deg; Vector3 axis;
		is>>deg>>axis;
		q = Quaternion(axis, deg);
		return is;
	};*/

private:
	Quaternion(const REAL &inR, const Vector3 &inV) : r(inR), v(inV) {}

	REAL r;
	Vector3 v;
};


/* transform : T(v) = (rot * v * scale) + trans */
template<class FT = double>
class Transform { 
public:
	typedef Vector3 Vec;

	Transform() : scale(1.) {}
	explicit Transform(const FT &inScale) : scale(inScale) {}
	explicit Transform(const Vec &inTrans) : scale(1.), trans(inTrans) {}
	Transform(const Quaternion<FT> &inRot, FT inScale = FT(1.), Vec inTrans = Vec()) : rot(inRot), scale(inScale), trans(inTrans) {}
	Transform(const Transform &t) : rot(t.rot), scale(t.scale), trans(t.trans) {}

	Transform operator*(const Transform &t) const { return Transform(rot * t.rot, t.scale * scale, trans + rot * (t.trans * scale )); }
	Vec operator*(const Vec &v) const { return rot * (v * scale) + trans; }

	Transform inverse() const { return Transform(rot.inverse(), 1. / scale, rot.inverse() * -trans * (1. / scale)); }

	Transform linearComponent() const { return Transform(rot, scale); }
	Vec mult3(const Vec &v) const { return rot * (v * scale); }

	FT getScale() const { return scale; }
	Vec getTrans() const { return trans; }
	Quaternion<FT> getRot() const { return rot; }
	/*
	template <class charT, class traits>
	friend std::basic_ostream<charT,traits>& operator<<(std::basic_ostream<charT,traits>& os, const Transform &t)
	{
		os<<t.getRot()<<t.getTrans()<<t.getScale();
		return os;
	};

	template<class charT, class traits>
	friend void operator>>(std::basic_istream<charT, traits>& is, const Transform &t)
	{
		is>>t.rot>>t.trans>>t.scale;		
	}
	*/
private:
	Quaternion<FT> rot;
	FT scale;
	Vec trans;
};


/*****************/
/*Implementations*/
/*****************/
// ------------------------------------------------------------
// Vector2 
// ------------------------------------------------------------

// constructors

inline Vector2::Vector2()
{
	x = y = 0.0f;
}

inline Vector2::Vector2(const FTP* args)
{
	x = args[0];
	y = args[1];
}

inline Vector2::Vector2(FTP _x, FTP _y) 
{
	x = _x; 
	y = _y; 
}

inline Vector2::Vector2(const Vector2& v)
{
	x = v.x;
	y = v.y;
}

// set

inline void Vector2::set(FTP _x, FTP _y)
{
	x = _x; 
	y = _y; 
}

inline void Vector2::set(const Vector2& v)
{
	x = v.x;
	y = v.y;
}

inline void Vector2::set(FTP* args)
{
	x = args[0];
	y = args[1];
}

// operators

inline Vector2 Vector2::operator *(FTP f) const
{ 
	Vector2 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Vector2 Vector2::operator /(FTP f) const
{ 
	Vector2 tmp(*this);
	tmp /= f;
	return tmp;
}

inline Vector2 Vector2::operator +(const Vector2& v) const
{
	Vector2 tmp(*this);
	tmp += v;
	return tmp;
}

inline Vector2 Vector2::operator -(const Vector2& v) const
{
	Vector2 tmp(*this);
	tmp -= v;
	return tmp;
}

inline Vector2 Vector2::operator -() const
{
	Vector2 tmp(*this);
	tmp.x = -tmp.x;
	tmp.y = -tmp.y;
	return tmp;
}

inline Vector2& Vector2::operator *=(FTP f) 
{ 
	x *= f; 
	y *= f; 
	return *this;
}

inline Vector2& Vector2::operator /=(FTP f) 
{ 
	FTP d = 1.0f / f;
	x *= d; 
	y *= d; 
	return *this;
}

inline Vector2& Vector2::operator +=(const Vector2& v) 
{
	x += v.x; 
	y += v.y; 
	return *this;
}

inline Vector2& Vector2::operator -=(const Vector2& v) 
{
	x -= v.x;
	y -= v.y;
	return *this;
}

inline int Vector2::operator==(const Vector2& v) 
{ 
	return ((x == v.x) && (y == v.y));
}

inline int Vector2::operator!=(const Vector2& v) 
{ 
	return ((x != v.x) || (y != v.y));
}

inline Vector2& Vector2::operator=(const Vector2& v)
{
	x = v.x;
	y = v.y;
	return *this;
}

inline Vector2& Vector2::operator&(const Vector2& v)
{
	x = -v.x;
	y = -v.y;
	return *this;
}

inline Matrix2_3 Vector2::operator *(const Vector3& v)const  /* 2_3 = 2_1 * 1*3 */
{ 
	Matrix2_3 m;
	m.M[0] = x*v.x;	m.M[1] = x*v.y;	m.M[2] = x*v.z;
	m.M[3] = y*v.x;	m.M[4] = y*v.y;	m.M[5] = y*v.z;
	return m;
}

// linear algebra

inline FTP Vector2::sqauremagnitude() const
{
	return x * x + y * y;
}

inline FTP Vector2::magnitude() const
{
	return sqrt(sqauremagnitude());
}

inline FTP Vector2::sqauredistance(const Vector2& v) const
{
	Vector2 tmp = Vector2(v.x - x, v.y - y);
	return tmp.sqauremagnitude();
}

inline FTP Vector2::distance(const Vector2& v) const
{
	Vector2 tmp = Vector2(v.x - x, v.y - y);
	return tmp.magnitude();
}

inline void Vector2::normalize()
{
	FTP m = magnitude();
	if(IsZero(m)) return;
	*this /= m;
}

inline void Vector2::normalize(const Vector2& v)
{
	*this = v;
	normalize();
}

inline FTP Vector2::dot(const Vector2& v) const
{ 
	return x * v.x + y * v.y; 
}



// ------------------------------------------------------------
// Matrix2
// ------------------------------------------------------------

// constructors

inline Matrix2::Matrix2(){
}

inline Matrix2::Matrix2(const FTP* other){
	for(int i = 0; i < 4; i++) M[i] = other[i];
}

inline Matrix2::Matrix2(const Matrix2& other){
	for(int i = 0; i < 4; i++) M[i] = other.M[i];
}

inline Matrix2& Matrix2::operator=(const Matrix2& other){
	for(int i = 0; i < 4; i++) M[i] = other.M[i];
	return *this;
}

// set

inline void Matrix2::set(const Matrix2& other){
	for(int i = 0; i < 4; i++) M[i] = other.M[i];
}

inline void Matrix2::set(const FTP* other){
	for(int i = 0; i < 4; i++) M[i] = other[i];
}

inline void Matrix2::setcols(const FTP* other){
	M[0] = other[0];
	M[2] = other[1];
	M[1] = other[2];
	M[3] = other[3];
}

inline bool Matrix2::isrotate(){
	Vector2 c1(m[0][0], m[1][0]);
	Vector2 c2(m[0][1], m[1][1]);
	
	if (!IsZero(c1.dot(c2))) {
		return false;
	}
	if (!IsEqual(c1.dot(c1), 1.0)) {
		return false;
	}
	if (!IsEqual(c2.dot(c2), 1.0)) {
		return false;
	}
	return true;
}

inline void Matrix2::setorthocol(int c1){
	int c2;
	if (c1 == 0) {
		c2 = 1;
	}
	else if (c1 == 1) {
		c2 = 0;
	} 
	else
		return;

	Vector2 v1(m[0][c1], m[1][c1]);
	// get a vector v2 orthogonal to v1
	Vector2 v2;
	if (!IsZero(v1[0])) {
		v2.set(-v1[1]/v1[0], 1.0);
	} else if (!IsZero(v1[1])) {
		v2.set(1.0, -v1[0]/v1[1]);
	} else {
		setzero();
		return;
	}

	m[0][c2] = v2.x;
	m[1][c2] = v2.y;
}

inline void Matrix2::setdiag(const FTP* diag){
	setzero();
	M[0] = diag[0];
	M[3] = diag[1];
}

inline void Matrix2::setdiag(FTP d1, FTP d2){
	setzero();
	M[0] = d1;
	M[3] = d2;
}

// simple methods

inline Matrix2& Matrix2::setidentity(){
	for(int i = 0; i < 4; i++) M[i] = Matrix2::Identity_Matrix[i];
	return *this;
}

inline Matrix2& Matrix2::setzero(){
	for(int i = 0; i < 4; i++) M[i] = 0.0f;
	return *this;
}

inline bool Matrix2::iszero(){
	for (int i=0; i<4; i++) {
		if (!IsZero(M[i]))		return false;
	}
	return true;
}

inline Matrix2& Matrix2::mul(const Matrix2& other){
	Matrix2 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix2& Matrix2::mul(const Matrix2& m1, const Matrix2& m2){
	M[0] = m1.M[0]*m2.M[0] + m1.M[1]*m2.M[2];
	M[1] = m1.M[0]*m2.M[1] + m1.M[1]*m2.M[3];
	M[2] = m1.M[2]*m2.M[0] + m1.M[3]*m2.M[2];
	M[3] = m1.M[2]*m2.M[1] + m1.M[3]*m2.M[3];
	return *this;
}

inline Matrix2& Matrix2::mulcol(int c, FTP f){
	m[0][c] *= f;
	m[1][c] *= f;
	return *this;
}

inline Matrix2& Matrix2::mul(const Matrix2& other, FTP f){
	const FTP* m = other.M;
	for(int i = 0; i < 4; i++) M[i] = m[i] * f;
	return *this;
}

inline void Matrix2::mul(const Vector2& vi, Vector2& vo){  //vo = M * vi : 矩阵左乘向量，无返回值
	vo[0] = M[0]*vi[0] + M[1]*vi[1] ;
	vo[1] = M[2]*vi[0] + M[3]*vi[1] ;	
}

inline Vector2 Matrix2::mul(const Vector2& v) const { //vo = M * vi : 矩阵左乘向量,返回一向量
	Vector2 tmp;
	tmp[0] = M[0]*v[0] + M[1]*v[1];
	tmp[1] = M[2]*v[0] + M[3]*v[1];
	return tmp;
}
// operators

inline Matrix2_3 Matrix2::operator*(const Matrix2_3& other) const{
	Matrix2_3 tmp;
	tmp.M[0] = M[0]*other.M[0] + M[1]*other.M[3];
	tmp.M[1] = M[0]*other.M[1] + M[1]*other.M[4];
	tmp.M[2] = M[0]*other.M[2] + M[1]*other.M[5];
	tmp.M[3] = M[2]*other.M[0] + M[3]*other.M[3];
	tmp.M[4] = M[2]*other.M[1] + M[3]*other.M[4];
	tmp.M[5] = M[2]*other.M[2] + M[3]*other.M[5];
	return tmp;
}

inline Vector2 Matrix2::operator*( const Vector2& other ) const
{
	Vector2 tmp;
	tmp.x = M[0]*other.x + M[1]*other.y;
	tmp.y = M[2]*other.x + M[3]*other.y;
	return tmp;
}

inline Matrix2 Matrix2::operator *(const Matrix2& other) const {
	Matrix2 tmp(*this);
	tmp *= other;
	return tmp;
}

inline Matrix2 Matrix2::operator +(const Matrix2& other) const {
	Matrix2 tmp(*this);
	tmp += other;
	return tmp;
}

inline Matrix2 Matrix2::operator -(const Matrix2& other) const {
	Matrix2 tmp(*this);
	tmp -= other;
	return tmp;
}

inline Matrix2& Matrix2::operator *=(const Matrix2& other){	
	Matrix2 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix2& Matrix2::operator +=(const Matrix2& v) {
	M[0] += v.M[0];
	M[1] += v.M[1];
	M[2] += v.M[2];
	M[3] += v.M[3];
	return *this;
}

inline Matrix2& Matrix2::operator -=(const Matrix2& v) {
	M[0] -= v.M[0];
	M[1] -= v.M[1];
	M[2] -= v.M[2];
	M[3] -= v.M[3];
	return *this;
}

inline Matrix2 Matrix2::operator *(FTP f) const { 
	Matrix2 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Matrix2& Matrix2::operator *=(FTP f) {
	M[0] *= f;
	M[1] *= f;
	M[2] *= f;
	M[3] *= f;
	return *this;
}

inline Matrix2 Matrix2::operator /(FTP f) const { 
	Matrix2 tmp(*this);
	tmp /= f;
	return tmp;
}

inline Matrix2& Matrix2::operator /=(FTP f) {
	FTP rf = 1 / f;
	M[0] *= rf;
	M[1] *= rf;
	M[2] *= rf;
	M[3] *= rf;
	return *this;
}

inline int Matrix2::operator==(const Matrix2& m) { 
	for (int i=0; i<4; i++) {
		if (!IsEqual(M[i], m.M[i]))		return false;
	}
	return true;
}

inline int Matrix2::operator!=(const Matrix2& m) { 
	for (int i=0; i<4; i++) {
		if (!IsEqual(M[i], m.M[i]))		return true;
	}
	return false;
}

inline int operator==(const Matrix2& m1, const Matrix2& m2){
	for (int i=0; i<4; i++) {
		if (!IsEqual(m1.M[i], m2.M[i]))		return false;
	}
	return true;
}

inline void Matrix2::squarecolsymm(FTP *symm){
	// symm: Compute square M^t * M. The upper triangle of the result symmetric matrix 
	//		 stored in column symmetric storage: [m11, m12, m22]
	//		 It must be allocated by caller.
	symm[0] = M[0]*M[0] + M[2]*M[2];
	symm[1] = M[0]*M[1] + M[2]*M[3];
	symm[2] = M[1]*M[1] + M[3]*M[3];
}

inline void Matrix2::squareupper(Matrix2& out) {
	out.M[0] = M[0]*M[0] + M[2]*M[2];
	out.M[1] = M[0]*M[1] + M[2]*M[3];
	out.M[3] = M[1]*M[1] + M[3]*M[3];
}

// Actions with Vector2
inline Matrix2& Matrix2::adjoint() {
	Matrix2 tmp(*this);
	adjoint(tmp);
	return *this;
}


inline Matrix2& Matrix2::adjoint(const Matrix2& other) {
	FTP a1, a2, b1, b2;
	const FTP *in = other.M;
	FTP *out = M;

	a1 = (in [0 + ( 0 )]);
	b1 = (in [0 + ( 1 )]);
	a2 = (in [2 + ( 0 )]);
	b2 = (in [2 + ( 1 )]);

	out[0 + ( 0 )]   =   b2;
	out[2 + ( 0 )]   = - a2;
	out[0 + ( 1 )]   = - b1;
	out[2 + ( 1 )]   =	 a1;
	return *this;
}

inline Matrix2& Matrix2::invert(const Matrix2& other) {
	int i;
	adjoint(other);
	FTP d = other.det();
	for (i=0; i<4; i++) {
		M[i] = M[i] / d;
	}
	return *this;
}


inline Matrix2& Matrix2::invert_exp() {
	Matrix2 tmp(*this);
	invert(tmp);
	return *this;
}

inline Matrix2& Matrix2::transpose() {
	Matrix2 tmp(*this);
	transpose(tmp);
	return *this;
}

inline Matrix2& Matrix2::transpose(const Matrix2& other) {
	for (int i=0; i<2; i++) {
		for (int j=0; j<2; j++) {
			M[i * 2 + j] = other.M[j * 2 + i];
		}
	}
	return *this;
}

// ------------------------------------------------------------
// Matrix3
// ------------------------------------------------------------

// constructors

inline Matrix3::Matrix3()
{

}

inline Matrix3::Matrix3(const FTP* other)
{
	for(int i = 0; i < 9; i++) M[i] = other[i];
}

inline Matrix3::Matrix3(const Matrix3& other)
{
	for(int i = 0; i < 9; i++) M[i] = other.M[i];
}

inline Matrix3::Matrix3(const Vector3& r1,const Vector3& r2,const Vector3& r3)
{
	M[0]=r1.x; M[1]=r1.y; M[2]=r1.z;
	M[3]=r2.x; M[4]=r2.y; M[5]=r2.z;
	M[6]=r3.x; M[7]=r3.y; M[8]=r3.z;
}

inline Matrix3& Matrix3::operator=(const Matrix3& other)
{
	for(int i = 0; i < 9; i++) M[i] = other.M[i];
	return *this;
}

// set

inline void Matrix3::set(const Matrix3& other)
{
	for(int i = 0; i < 9; i++) M[i] = other.M[i];
}

inline void Matrix3::set(const FTP* other)
{
	for(int i = 0; i < 9; i++) M[i] = other[i];
}

inline void Matrix3::setcols(const FTP* other) 
{
	M[0] = other[0];
	M[3] = other[1];
	M[6] = other[2];
	M[1] = other[3];
	M[4] = other[4];
	M[7] = other[5];
	M[2] = other[6];
	M[5] = other[7];
	M[8] = other[8];
}

inline bool Matrix3::isrotate()
{
	Vector3 c1(m[0][0], m[1][0], m[2][0]);
	Vector3 c2(m[0][1], m[1][1], m[2][1]);
	Vector3 c3(m[0][2], m[1][2], m[2][2]);
	FTP d12 = c1.dot(c2);
	FTP d13 = c1.dot(c3);
	FTP d23 = c2.dot(c3);
	FTP d11 = c1.dot(c1);
	FTP d22 = c2.dot(c2);
	FTP d33 = c3.dot(c3);

	if (!IsZero(c1.dot(c2))) {
		return false;
	}
	if (!IsZero(c1.dot(c3))) {
		return false;
	}
	if (!IsZero(c2.dot(c3))) {
		return false;
	}
	if (!IsEqual(c1.dot(c1), 1.0)) {
		return false;
	}
	if (!IsEqual(c2.dot(c2), 1.0)) {
		return false;
	}
	if (!IsEqual(c3.dot(c3), 1.0)) {
		return false;
	}
	return true;
}

inline void Matrix3::setorthocol(int c1)
{
	int c2, c3;
	if (c1 == 0) {
		c2 = 1;
		c3 = 2;
	} else if (c1 == 1) {
		c2 = 0;
		c3 = 2;
	} else if (c1 == 2) {
		c2 = 0;
		c3 = 1;
	} else {
		return;
	}
	Vector3 v1(m[0][c1], m[1][c1], m[2][c1]);
	// get a vector v2 orthogonal to v1
	Vector3 v2;
	if (!IsZero(v1[0])) {
		v2.set(-v1[1]/v1[0], 1.0, 0.0);
	} else if (!IsZero(v1[1])) {
		v2.set(1.0, -v1[0]/v1[1], 0.0);
	} else if (!IsZero(v1[2])) {
		v2.set(1.0, 0.0, -v1[0]/v1[2]);
	} else {
		setzero();
		return;
	}
	// get the 3rd vector orthogonal to v1 and v2
	Vector3 v3 = v1.cross(v2);
	m[0][c2] = v2.x;
	m[1][c2] = v2.y;
	m[2][c2] = v2.z;
	m[0][c3] = v3.x;
	m[1][c3] = v3.y;
	m[2][c3] = v3.z;
}

inline void Matrix3::setorthocol(int c1, int c2)
{
	if (c1<0 || c1>2 || c2<0 || c2>2 || c1==c2)
		return;
	int c3 = 3 - c1 - c2;
	Vector3 v1(m[0][c1], m[1][c1], m[2][c1]);
	Vector3 v2(m[0][c2], m[1][c2], m[2][c2]);
	Vector3 v3 = v1.cross(v2);
	m[0][c3] = v3.x;
	m[1][c3] = v3.y;
	m[2][c3] = v3.z;
}

inline void Matrix3::setdiag(const FTP* diag)
{
	setzero();
	M[0] = diag[0];
	M[4] = diag[1];
	M[8] = diag[2];
}

inline void Matrix3::setdiag(FTP d1, FTP d2, FTP d3)
{
	setzero();
	M[0] = d1;
	M[4] = d2;
	M[8] = d3;
}

// simple methods

inline Matrix3& Matrix3::setidentity()
{
	for(int i = 0; i < 9; i++) M[i] = Matrix3::Identity_Matrix[i];
	return *this;
}

inline Matrix3& Matrix3::setzero()
{
	for(int i = 0; i < 9; i++) M[i] = 0.0f;
	return *this;
}

inline bool Matrix3::iszero()
{
	for (int i=0; i<9; i++) {
		if (!IsZero(M[i])) {
			return false;
		}
	}
	return true;
}

inline Matrix3& Matrix3::mul(const Matrix3& other)
{
	Matrix3 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix3& Matrix3::mul(const Matrix3& m1, const Matrix3& m2)
{
	M[0] = m1.M[0]*m2.M[0] + m1.M[1]*m2.M[3] + m1.M[2]*m2.M[6];
	M[1] = m1.M[0]*m2.M[1] + m1.M[1]*m2.M[4] + m1.M[2]*m2.M[7];
	M[2] = m1.M[0]*m2.M[2] + m1.M[1]*m2.M[5] + m1.M[2]*m2.M[8];
	M[3] = m1.M[3]*m2.M[0] + m1.M[4]*m2.M[3] + m1.M[5]*m2.M[6];
	M[4] = m1.M[3]*m2.M[1] + m1.M[4]*m2.M[4] + m1.M[5]*m2.M[7];
	M[5] = m1.M[3]*m2.M[2] + m1.M[4]*m2.M[5] + m1.M[5]*m2.M[8];
	M[6] = m1.M[6]*m2.M[0] + m1.M[7]*m2.M[3] + m1.M[8]*m2.M[6];
	M[7] = m1.M[6]*m2.M[1] + m1.M[7]*m2.M[4] + m1.M[8]*m2.M[7];
	M[8] = m1.M[6]*m2.M[2] + m1.M[7]*m2.M[5] + m1.M[8]*m2.M[8];
	return *this;
}

inline Matrix3& Matrix3::mulcol(int c, FTP f)
{
	m[0][c] *= f;
	m[1][c] *= f;
	m[2][c] *= f;
	return *this;
}

inline Matrix3& Matrix3::mul(const Matrix3& other, FTP f)
{
	const FTP* m = other.M;
	for(int i = 0; i < 9; i++) M[i] = m[i] * f;
	return *this;
}

inline void Matrix3::mul(const Vector3& vi, Vector3& vo)
{
	vo[0] = M[0]*vi[0] + M[1]*vi[1] + M[2]*vi[2];
	vo[1] = M[3]*vi[0] + M[4]*vi[1] + M[5]*vi[2];
	vo[2] = M[6]*vi[0] + M[7]*vi[1] + M[8]*vi[2];
}

inline Vector3 Matrix3::mul(const Vector3& v) const
{
	Vector3 tmp;
	tmp[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
	tmp[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
	tmp[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
	return tmp;
}
// operators

inline Vector3 Matrix3::operator *(const Vector3& v) const
{
	return this->mul(v);
}


inline Matrix3 Matrix3::operator *(const Matrix3& other) const
{
	Matrix3 tmp(*this);
	tmp *= other;
	return tmp;
}

inline Matrix3 Matrix3::operator +(const Matrix3& other) const
{
	Matrix3 tmp(*this);
	tmp += other;
	return tmp;
}

inline Matrix3 Matrix3::operator -(const Matrix3& other) const
{
	Matrix3 tmp(*this);
	tmp -= other;
	return tmp;
}

inline Matrix3& Matrix3::operator *=(const Matrix3& other)
{
	Matrix3 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix3& Matrix3::operator +=(const Matrix3& v) 
{
	M[0] += v.M[0];
	M[1] += v.M[1];
	M[2] += v.M[2];
	M[3] += v.M[3];
	M[4] += v.M[4];
	M[5] += v.M[5];
	M[6] += v.M[6];
	M[7] += v.M[7];
	M[8] += v.M[8];
	return *this;
}

inline Matrix3& Matrix3::operator -=(const Matrix3& v) 
{
	M[0] -= v.M[0];
	M[1] -= v.M[1];
	M[2] -= v.M[2];
	M[3] -= v.M[3];
	M[4] -= v.M[4];
	M[5] -= v.M[5];
	M[6] -= v.M[6];
	M[7] -= v.M[7];
	M[8] -= v.M[8];
	return *this;
}

inline Matrix3 Matrix3::operator *(FTP f) const
{ 
	Matrix3 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Matrix3& Matrix3::operator *=(FTP f) 
{
	M[0] *= f;
	M[1] *= f;
	M[2] *= f;
	M[3] *= f;
	M[4] *= f;
	M[5] *= f;
	M[6] *= f;
	M[7] *= f;
	M[8] *= f;
	return *this;
}

inline Matrix3 Matrix3::operator /(FTP f) const
{ 
	Matrix3 tmp(*this);
	tmp /= f;
	return tmp;
}

inline Matrix3& Matrix3::operator /=(FTP f) 
{
	FTP rf = 1 / f;
	M[0] *= rf;
	M[1] *= rf;
	M[2] *= rf;
	M[3] *= rf;
	M[4] *= rf;
	M[5] *= rf;
	M[6] *= rf;
	M[7] *= rf;
	M[8] *= rf;
	return *this;
}

inline int Matrix3::operator==(const Matrix3& m) 
{ 
	for (int i=0; i<9; i++) {
		if (!IsEqual(M[i], m.M[i]))
			return false;
	}
	return true;
}

inline int Matrix3::operator!=(const Matrix3& m) 
{ 
	for (int i=0; i<9; i++) {
		if (!IsEqual(M[i], m.M[i]))
			return true;
	}
	return false;
}

// inline int Matrix3::operator==(const Matrix3& m1, const Matrix3& m2){
// 	for (int i=0; i<9; i++) {
// 		if (!IsEqual(m1.M[i], m2.M[i]))
// 			return false;
// 	}
// 	return true;
// }

inline void Matrix3::squarecolsymm(FTP *symm)
{
	// symm: Compute square M^t * M. The upper triangle of the result symmetric matrix 
	//		 stored in column symmetric storage: [m11, m12, m22, m13, m23, m33]
	//		 It must be allocated by caller.
	symm[0] = M[0]*M[0] + M[3]*M[3] + M[6]*M[6];
	symm[1] = M[0]*M[1] + M[3]*M[4] + M[6]*M[7];
	symm[2] = M[1]*M[1] + M[4]*M[4] + M[7]*M[7];
	symm[3] = M[0]*M[2] + M[3]*M[5] + M[6]*M[8];
	symm[4] = M[1]*M[2] + M[4]*M[5] + M[7]*M[8];
	symm[5] = M[2]*M[2] + M[5]*M[5] + M[8]*M[8];
}

inline void Matrix3::squareupper(Matrix3& out)
{
	out.M[0] = M[0]*M[0] + M[3]*M[3] + M[6]*M[6];
	out.M[1] = M[0]*M[1] + M[3]*M[4] + M[6]*M[7];
	out.M[2] = M[0]*M[2] + M[3]*M[5] + M[6]*M[8];
	out.M[4] = M[1]*M[1] + M[4]*M[4] + M[7]*M[7];
	out.M[5] = M[1]*M[2] + M[4]*M[5] + M[7]*M[8];
	out.M[8] = M[2]*M[2] + M[5]*M[5] + M[8]*M[8];
}

// transformation methods

inline Matrix3& Matrix3::translate(FTP tx, FTP ty)
{
	M[6] = M[0] * tx + M[3] * ty + M[6];
	M[7] = M[1] * tx + M[4] * ty + M[7];
	M[8] = M[2] * tx + M[5] * ty + M[8];
	return *this;
}

inline Matrix3& Matrix3::settranslate(FTP tx, FTP ty)
{
	setidentity();
	M[6] = tx;
	M[7] = ty;
	return *this;
}

inline Matrix3& Matrix3::rotate(FTP r)
{
	FTP c = Cos(r), s = Sin(r);
	Matrix3 tmp;
	tmp.setidentity();
	tmp.M[0] = c;
	tmp.M[1] = -s;
	tmp.M[3] = s;
	tmp.M[4] = c;
	*this *= tmp;
	return *this;
}

inline Matrix3& Matrix3::setrotate(FTP r)
{
	FTP c = Cos(r), s = Sin(r);
	Matrix3 tmp;
	setidentity();
	M[0] = c;
	M[1] = -s;
	M[3] = s;
	M[4] = c;
	return *this;
}

inline Matrix3& Matrix3::scale(FTP sx, FTP sy)
{
	M[0] *= sx; M[3] *= sy;
	M[1] *= sx; M[4] *= sy;
	M[2] *= sx; M[5] *= sy;
	return *this;
}

inline Matrix3& Matrix3::setscale(FTP sx, FTP sy)
{
	M[0] = sx; M[4] = sy; M[8] = 1.0f;
	M[1] = M[2] = M[3] = M[5] = M[6] = M[7] = 0.0f;
	return *this;
}

// Actions with Vector2

inline Vector2 Matrix3::transform(const Vector2& v)
{
	return Vector2(v.x * M[0] + v.y * M[3] + M[6], v.x * M[1] + v.y * M[4] + M[7]);
}

inline Matrix3& Matrix3::adjoint()
{
	Matrix3 tmp(*this);
	adjoint(tmp);
	return *this;
}

inline Matrix3& Matrix3::invert_exp()
{
	Matrix3 tmp(*this);
	invert(tmp);
	return *this;
}

inline Matrix3& Matrix3::invert_imp1()
{
	FTP b[3];
	FTP a[3][3];
	Matrix3 x;

	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			a[i][j] = m[i][j];
		}
	}
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			b[j] = 0.0;
		}
		b[i] = 1.0;
		Solve3x3LinSysGaussElim(a, b, m[i]);
	}
	return *this;
}

inline Matrix3& Matrix3::invert_imp2()
{
	FTP b[3];
	FTP a[3][3];
	Matrix3 x;

	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			a[i][j] = m[i][j];
		}
	}
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			b[j] = 0.0;
		}
		b[i] = 1.0;
		Solve3x3LinSysDoolittle(a, b, m[i]);
	}
	return *this;
}

inline Matrix3& Matrix3::transpose()
{
	Matrix3 tmp(*this);
	transpose(tmp);
	return *this;
}

// ------------------------------------------------------------
// Vector3 functions
// ------------------------------------------------------------

// constructors

inline Vector3::Vector3()
{
	x = y = z = 0.0f;
}

inline Vector3::Vector3(const FTP* args)
{
	x = args[0];
	y = args[1];
	z = args[2];
}

inline Vector3::Vector3(FTP _x, FTP _y, FTP _z) 
{
	x = _x; 
	y = _y; 
	z = _z;
}

inline Vector3::Vector3(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

// set

inline void Vector3::set(const FTP* args)
{
	x = args[0];
	y = args[1];
	z = args[2];
}

inline void Vector3::set(FTP _x, FTP _y, FTP _z) 
{
	x = _x; 
	y = _y; 
	z = _z;
}

inline void Vector3::set(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

inline void Vector3::set(const Vector3* v)
{
	x = v->x;
	y = v->y;
	z = v->z;
}

// operators

inline Vector3 Vector3::operator *(FTP f) const
{ 
	Vector3 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Vector3 Vector3::operator /(FTP f) const
{ 
	Vector3 tmp(*this);
	tmp /= f;
	return tmp;
}

inline Vector3 Vector3::operator +(const Vector3& v) const
{
	Vector3 tmp(*this);
	tmp += v;
	return tmp;
}

inline Vector3 Vector3::operator -(const Vector3& v) const
{
	Vector3 tmp(*this);
	tmp -= v;
	return tmp;
}

inline Vector3 Vector3::operator -() const
{
	Vector3 tmp(*this);
	tmp.x = -tmp.x;
	tmp.y = -tmp.y;
	tmp.z = -tmp.z;
	return tmp;
}

inline Vector3& Vector3::operator *=(FTP f) 
{ 
	x *= f; 
	y *= f; 
	z *= f; 
	return *this;
}

inline Vector3& Vector3::operator /=(FTP f) 
{ 
	FTP d = 1.0f / f;
	x *= d; 
	y *= d; 
	z *= d; 
	return *this;
}

inline Vector3& Vector3::operator +=(const Vector3& v) 
{
	x += v.x; 
	y += v.y; 
	z += v.z; 
	return *this;
}

inline Vector3& Vector3::operator -=(const Vector3& v) 
{
	x -= v.x; 
	y -= v.y; 
	z -= v.z; 
	return *this;
}

inline int Vector3::operator==(const Vector3& v) 
{ 
	return ((x == v.x) && (y == v.y) && (z == v.z));
}

inline int Vector3::operator!=(const Vector3& v) 
{ 
	return ((x != v.x) || (y != v.y) || (z != v.z));
}

inline int operator==(const Vector3& v1, const Vector3& v2)
{
  return (v1.v[0] == v2.v[0] && v1.v[1] == v2.v[1] && v1.v[2] == v2.v[2]);
}

inline Vector3& Vector3::operator=(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

inline Vector3& Vector3::operator&(const Vector3& v)
{
	x = -v.x;
	y = -v.y;
	z = -v.z;
	return *this;
}


inline Matrix3 Vector3::operator *(const Vector3& v)const  /* 2_3 = 2_1 * 1*3 */
{ 
	Matrix3 m;
	m.M[0] = x*v.x;	m.M[1] = x*v.y;	m.M[2] = x*v.z;
	m.M[3] = y*v.x;	m.M[4] = y*v.y;	m.M[5] = y*v.z;
	m.M[6] = z*v.x;	m.M[7] = z*v.y;	m.M[8] = z*v.z;
	return m;
}

// linear algebra

inline FTP Vector3::sqauremagnitude() const
{
	return x * x + y * y + z * z;
}

inline FTP Vector3::magnitude() const
{
	return sqrt(sqauremagnitude());
}

inline FTP Vector3::sqauredistance(const Vector3& v) const
{
	Vector3 tmp = Vector3(v.x - x, v.y - y, v.z - z);
	return tmp.sqauremagnitude();
}

inline FTP Vector3::distance(const Vector3& v) const
{
	Vector3 tmp = Vector3(v.x - x, v.y - y, v.z - z);
	return tmp.magnitude();
}

inline Vector3& Vector3::normalize() //normalize it self
{
	FTP m = magnitude();
	if(IsZero(m)) return *this;
	*this /= m;
	return *this;
}

inline Vector3 Vector3::normalize() const  //const call, only return a normalized value, no change to itself
{
	FTP m = magnitude();
	if(IsZero(m)) return *this;
	return *this/m;
}

inline void Vector3::normalize(const Vector3& v)
{
	*this = v;
	normalize();
}
inline FTP Vector3::dot(const Vector3& v) const 
{ 
	return x * v.x + y * v.y + z * v.z; 
}

inline Vector3 Vector3::cross(const Vector3& v) const
{ 
	return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); 
}

inline Matrix3 Vector3::square() const
{
	Matrix3 tmp;
	tmp.M[0] = v[0]*v[0];
	tmp.M[1] = tmp.M[3] = v[0]*v[1];
	tmp.M[2] = tmp.M[6] = v[0]*v[2];
	tmp.M[4] = v[1]*v[1];
	tmp.M[5] = tmp.M[7] = v[1]*v[2];
	tmp.M[8] = v[2]*v[2];
	return tmp;
}

// ------------------------------------------------------------
// Matrix4
// ------------------------------------------------------------

// constructors

inline Matrix4::Matrix4()
{

}

inline Matrix4::Matrix4(const Matrix4& other)
{
	for(int i = 0; i < 16; i++) M[i] = other.M[i];
}

inline Matrix4::Matrix4(const FTP* other)
{
	for(int i = 0; i < 16; i++) M[i] = other[i];
}

// set

inline Matrix4& Matrix4::operator=(const Matrix4& other)
{
	for(int i = 0; i < 16; i++) M[i] = other.M[i];
	return *this;
}

inline void Matrix4::set(const FTP* other)
{
	for(int i = 0; i < 16; i++) M[i] = other[i];
}

inline void Matrix4::set(const Matrix4& other)
{
	for(int i = 0; i < 16; i++) M[i] = other.M[i];
}

// Simple methods

inline Matrix4& Matrix4::setidentity()
{
	for(int i = 0; i < 16; i++) M[i] = Matrix4::Identity_Matrix[i];
	return *this;
}

inline Matrix4& Matrix4::setzero()
{
	for(int i = 0; i < 16; i++) M[i] = 0.0f;
	return *this;
}

inline Matrix4& Matrix4::setswitchorientation()
{
	for(int i = 0; i < 16; i++) M[i] = Matrix4::Orientation_Switch_Matrix[i];
	return *this;
}

inline Matrix4& Matrix4::setperspective()
{
	for(int i = 0; i < 16; i++) M[i] = Matrix4::Perspective_Matrix[i];
	return *this;
}

inline Matrix4& Matrix4::mul(const Matrix4& other)
{
	Matrix4 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix4& Matrix4::mul(const Matrix4& m1, const Matrix4& m2)
{
	const FTP *b = m1.M;
	const FTP *a = m2.M;
	FTP *prod = M;

	for(int i = 0; i < 4; i++)
	{
		FTP a0 = a[i];
		FTP a1 = a[i + 4];
		FTP a2 = a[i + 8];
		FTP a3 = a[i + 12];
		prod[i] = a0 * b[0] + a1 * b[1] + a2 * b[2] + a3 * b[3];
		prod[i + 4] = a0 * b[4] + a1 * b[5] + a2 * b[6] + a3 * b[7];
		prod[i + 8] = a0 * b[8] + a1 * b[9] + a2 * b[10] + a3 * b[11];
		prod[i + 12] = a0 * b[12] + a1 * b[13] + a2 * b[14] + a3 * b[15];
	}
	return *this;
}

inline Matrix4 Matrix4::operator *(FTP f) const
{ 
	Matrix4 tmp(*this);
	tmp *= f;
	return tmp;
}

inline Matrix4& Matrix4::operator *=(FTP f) 
{
	for (int i=0; i<16; i++) {
		M[i] *= f;
	}
	return *this;
}

inline Matrix4 Matrix4::operator /(FTP f) const
{ 
	Matrix4 tmp(*this);
	tmp /= f;
	return tmp;
}

inline Matrix4& Matrix4::operator /=(FTP f) 
{
	FTP rf = 1 / f;
	for (int i=0; i<16; i++) {
		M[i] *= rf;
	}
	return *this;
}

// Operators

inline Matrix4& Matrix4::operator*=(const Matrix4& other)
{
	Matrix4 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix4 Matrix4::operator*(const Matrix4& other) const
{
	Matrix4 tmp;
	tmp.mul(*this, other);
	return tmp;
}

// Transformation methods

inline Matrix4& Matrix4::translate(FTP tx, FTP ty, FTP tz)
{
	M[12] = M[0] * tx + M[4] * ty + M[8]  * tz + M[12];
	M[13] = M[1] * tx + M[5] * ty + M[9]  * tz + M[13];
	M[14] = M[2] * tx + M[6] * ty + M[10] * tz + M[14];
	M[15] = M[3] * tx + M[7] * ty + M[11] * tz + M[15];
	return *this;
}

inline Matrix4& Matrix4::settranslate(FTP tx, FTP ty, FTP tz)
{
	setidentity();
	return translate(tx, ty, tz);
}

inline Matrix4& Matrix4::scale(FTP sx, FTP sy, FTP sz)
{
	M[0] *= sx; M[4] *= sy; M[8]  *= sz;
	M[1] *= sx; M[5] *= sy; M[9]  *= sz;
	M[2] *= sx; M[6] *= sy; M[10] *= sz;
	M[3] *= sx; M[7] *= sy; M[11] *= sz;
	return *this;
}

inline Matrix4& Matrix4::setscale(FTP sx, FTP sy, FTP sz)
{
	setidentity();
	return scale(sx, sy, sz);
}

inline Matrix4& Matrix4::setrotate(const Matrix3 &matrix)
{
	M[0] = matrix.M[0];
	M[1] = matrix.M[3];
	M[2] = matrix.M[6];
	M[3] = 0.0;
	M[4] = matrix.M[1];
	M[5] = matrix.M[4];
	M[6] = matrix.M[7];
	M[7] = 0.0;
	M[8] = matrix.M[2];
	M[9] = matrix.M[5];
	M[10] = matrix.M[8];
	M[11] = 0.0;
	M[12] = 0.0;
	M[13] = 0.0;
	M[14] = 0.0;
	M[15] = 1.0;
	return *this;
}

inline void Matrix4::getrotate(Matrix3 &matrix)
{
	matrix.M[0] = M[0];
	matrix.M[3] = M[1];
	matrix.M[6] = M[2];
	matrix.M[1] = M[4];
	matrix.M[4] = M[5];
	matrix.M[7] = M[6];
	matrix.M[2] = M[8];
	matrix.M[5] = M[9];
	matrix.M[8] = M[10];
}

// rotation around three euler-angles

inline Matrix4& Matrix4::setrotate(const Vector3& r)
{
	return setrotate(r.x, r.y, r.z);
}

inline Matrix4& Matrix4::rotate(const Vector3& r)
{
	return rotate(r.x, r.y, r.z);
}

inline Matrix4& Matrix4::rotate(FTP rx, FTP ry, FTP rz)
{
	Matrix4 tmp;
	tmp.setrotate(rx, ry, rz);
	mul(tmp);
	return *this;
}

// rotation euler-angle around axis

inline Matrix4& Matrix4::setrotate(FTP angle, const Vector3& r)
{
	return setrotate(angle, r.x, r.y, r.z);
}

inline Matrix4& Matrix4::rotate(FTP angle, const Vector3& r)
{
	return rotate(angle, r.x, r.y, r.z);
}

inline Matrix4& Matrix4::rotate(FTP angle, FTP x, FTP y, FTP z)
{
	Matrix4 tmp;
	tmp.setrotate(angle, x, y, z);
	mul(tmp);
	return *this;
}

// Invert/Transpose
inline Matrix4& Matrix4::adjoint()
{
	Matrix4 tmp(*this);
	adjoint(tmp);
	return *this;
}

inline Matrix4& Matrix4::transpose()
{
	Matrix4 tmp(*this);
	transpose(tmp);
	return *this;
}

inline Matrix4& Matrix4::invert()
{
	Matrix4 tmp(*this);
	invert(tmp);
	return *this;
}

// Actions with Vector3

inline Vector3 Matrix4::transform(const Vector3& v)
{
	return Vector3(v.x * M[0] + v.y * M[4] + v.z * M[8] + M[12],
				   v.x * M[1] + v.y * M[5] + v.z * M[9] + M[13],
				   v.x * M[2] + v.y * M[6] + v.z * M[10] + M[14]);
}

inline void Matrix4::reform_1( Matrix3& outM, Vector3& outV )
{
	outM.M[0] = M[0];	outM.M[1] = M[4];	outM.M[2] = M[8];				
	outM.M[3] = M[1];	outM.M[4] = M[5];	outM.M[5] = M[9];				
	outM.M[6] = M[2];	outM.M[7] = M[6];	outM.M[8] = M[10];		
	
	outV.x = M[12];	outV.y = M[13];	outV.z = M[14];
}
// ------------------------------------------------------------
// Plane functions
// ------------------------------------------------------------

// constructors

inline Plane::Plane()
{
	a = b = c = d = 0.0f;
}

inline Plane::Plane(const Vector3& _n, FTP _d)
{
	a = _n.x; 
	b = _n.y; 
	c = _n.z; 
	d = _d;
}

inline Plane::Plane(FTP _a, FTP _b, FTP _c, FTP _d)
{
	a = _a; 
	b = _b; 
	c = _c; 
	d = _d;
}

inline Plane::Plane(FTP* args)
{
	a = args[0]; 
	b = args[1]; 
	c = args[2]; 
	d = args[3];
}

inline Plane::Plane(const Plane& p)
{
	a = p.a; 
	b = p.b; 
	c = p.c; 
	d = p.d;
}

inline void Plane::set(const Vector3& _n, FTP _d)
{
	a = _n.x; 
	b = _n.y; 
	c = _n.z; 
	d = _d;
}

// set

inline void Plane::set(FTP _a, FTP _b, FTP _c, FTP _d)
{
	a = _a; 
	b = _b; 
	c = _c; 
	d = _d;
}

inline void Plane::set(FTP* args)
{
	a = args[0]; 
	b = args[1]; 
	c = args[2]; 
	d = args[3];
}

inline void Plane::set(const Plane& p)
{
	a = p.a; 
	b = p.b; 
	c = p.c; 
	d = p.d;
}

// operators

inline Plane& Plane::operator=(const Plane& p)
{
	a = p.a; b = p.b; c = p.c; d = p.d;
	return *this;
}

// interactor width vector3

inline FTP Plane::distance(const Vector3& v) const
{
	return n.dot(v) - d;
}

inline Vector3 Plane::reflect(const Vector3& v)
{
	return v - n * 2 * distance(v);
}

inline Vector3 Plane::intersect(const Vector3& start, const Vector3& end)
{
	FTP s = distance(start);
	FTP e = distance(end);

	if(IsZero(e - s))return Vector3();

	FTP ws = -s / (e - s);
	FTP we = e / (e - s);

	return start * ws + end * we;
}

//////////////////////////////////////////////////////////////////////
/////////////////////////*matrix2_3*//////////////////////////////////
//////////////////////////////////////////////////////////////////////
inline Matrix2_3& Matrix2_3::setzero(){
	for (int i=0; i<6; ++i)		M[i] =0;
	return *this;
}

inline Matrix2_3& Matrix2_3::setidentity(){
	for (int i=0; i<6; ++i)		M[i] = Identity_Matrix[i];
	return *this;
}

inline Matrix2_3& Matrix2_3::operator=(const Matrix2_3& other){
	for (int i=0; i<6; ++i)		M[i] = other.M[i];
	return *this;
}

//multiply
inline Matrix2 Matrix2_3::mul(const Matrix2_3& m1, const Matrix3_2& m2) const {
	Matrix2 R;
	R.M[0] = m1.M[0]*m2.M[0] + m1.M[1]*m2.M[2] + m1.M[2]*m2.M[4];
	R.M[1] = m1.M[0]*m2.M[1] + m1.M[1]*m2.M[3] + m1.M[2]*m2.M[5];
	R.M[2] = m1.M[3]*m2.M[0] + m1.M[4]*m2.M[2] + m1.M[5]*m2.M[4];
	R.M[3] = m1.M[3]*m2.M[1] + m1.M[4]*m2.M[3] + m1.M[5]*m2.M[5];
	return R;
}

inline Matrix2 Matrix2_3::mul(const Matrix3_2& other)  const {
	return mul(*this, other);	
}

inline Matrix2_3& Matrix2_3::mul(const Matrix2_3& m1, const Matrix3& m2) {
	M[0] = m1.M[0]*m2.M[0] + m1.M[1]*m2.M[3] + m1.M[2]*m2.M[6];
	M[1] = m1.M[0]*m2.M[1] + m1.M[1]*m2.M[4] + m1.M[2]*m2.M[7];
	M[2] = m1.M[0]*m2.M[2] + m1.M[1]*m2.M[5] + m1.M[2]*m2.M[8];
	M[3] = m1.M[3]*m2.M[0] + m1.M[4]*m2.M[3] + m1.M[5]*m2.M[6];
	M[4] = m1.M[3]*m2.M[1] + m1.M[4]*m2.M[4] + m1.M[5]*m2.M[7];
	M[5] = m1.M[3]*m2.M[2] + m1.M[4]*m2.M[5] + m1.M[5]*m2.M[8];
	return *this;
}

inline Matrix2_3& Matrix2_3::mul(const Matrix3& other){
	Matrix2_3 tmp(*this);
	mul(tmp,other);
	return *this;
}

inline void Matrix2_3::mul(const Vector3& vi, Vector2& vo) const{  //vo = M * vi : 矩阵左乘向量，无返回值 2_3*3_1 = 2_1
	vo[0] = M[0]*vi[0] + M[1]*vi[1] + M[2]*vi[2];
	vo[1] = M[3]*vi[0] + M[4]*vi[1] + M[5]*vi[2];	
}

inline Vector2 Matrix2_3::mul(const Vector3& v) const { //vo = M * vi : 矩阵左乘向量,返回一向量
	Vector2 tmp;
	tmp[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
	tmp[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];	
	return tmp;
}


// operators

inline Matrix2_3& Matrix2_3::operator *=(FTP f) {
	M[0] *= f;	M[1] *= f;	M[2] *= f;
	M[3] *= f;	M[4] *= f;	M[5] *= f;
	return *this;
}

inline Matrix2_3 Matrix2_3::operator *(FTP f) const { 
	Matrix2_3 tmp(*this);
	tmp *= f;
	return tmp;
}
////////////

inline Matrix2 Matrix2_3::operator *=(const Matrix3_2& other){	
	return mul(*this, other);
}

inline Matrix2 Matrix2_3::operator *(const Matrix3_2& other) const {
	Matrix2_3 tmp(*this);
	return tmp *= other;
}

inline Matrix2_3& Matrix2_3::operator *=(const Matrix3& other){	
	Matrix2_3 tmp(*this);
	mul(tmp, other);
	return *this;
}

inline Matrix2_3 Matrix2_3::operator *(const Matrix3& other) const {
	Matrix2_3 tmp(*this);
	tmp *= other;
	return tmp;
}

/////////

inline Matrix2_3& Matrix2_3::operator +=(const Matrix2_3& v) {
	M[0] += v.M[0];	M[1] += v.M[1];	M[2] += v.M[2];
	M[3] += v.M[3]; M[4] += v.M[4]; M[5] += v.M[5];
	return *this;
}

inline Matrix2_3 Matrix2_3::operator +(const Matrix2_3& other) const {
	Matrix2_3 tmp(*this);
	tmp += other;
	return tmp;
}

inline Matrix2_3& Matrix2_3::operator -=(const Matrix2_3& v) {
	M[0] -= v.M[0]; M[1] -= v.M[1];	M[2] -= v.M[2];
	M[3] -= v.M[3]; M[4] -= v.M[4]; M[5] -= v.M[5];
	return *this;
}

inline Matrix2_3 Matrix2_3::operator -(const Matrix2_3& other) const {
	Matrix2_3 tmp(*this);
	tmp -= other;
	return tmp;
}

inline Matrix2_3& Matrix2_3::operator /=(FTP f) {
	FTP rf = 1 / f;
	M[0] *= rf;	M[1] *= rf;	M[2] *= rf;
	M[3] *= rf;	M[4] *= rf;	M[5] *= rf;
	return *this;
}

inline Matrix2_3 Matrix2_3::operator /(FTP f) const { 
	Matrix2_3 tmp(*this);
	tmp /= f;
	return tmp;
}


inline int Matrix2_3::operator==(const Matrix2_3& m) const { 
	for (int i=0; i<4; i++) {
		if (!IsEqual(M[i], m.M[i]))		return false;
	}
	return true;
}

inline int Matrix2_3::operator!=(const Matrix2_3& m) const { 
	for (int i=0; i<4; i++) {
		if (!IsEqual(M[i], m.M[i]))		return true;
	}
	return false;
}

inline int operator==(const Matrix2_3& m1, const Matrix2_3& m2){
	for (int i=0; i<4; i++) {
		if (!IsEqual(m1.M[i], m2.M[i]))		return false;
	}
	return true;
}

inline Matrix3_2 Matrix2_3::transpose() const{
	return transpose(*this);
}

inline Matrix3_2 Matrix2_3::transpose(const Matrix2_3& other) const{
	Matrix3_2 R;
	int A_nRow(2),A_nCol(3);
	for (int i=0; i<A_nRow; i++) {
		for (int j=0; j<A_nCol; j++) {
			R.M[j * A_nRow + i] = other.M[i * A_nCol + j];
		}
	}
	return R;
}

inline Vector2 Matrix2_3::operator *(const Vector3& v)const  /* 2_3 = 2_1 * 1*3 */
{ 
	return Vector2(M[0]*v.x + M[1]*v.y+ M[2]*v.z ,
			   	   M[3]*v.x + M[4]*v.y+ M[5]*v.z );
}	


/////////////math utility functions/////////
//vector2  CMP_RULES
inline bool X_lager_v2(Vector2 &a,Vector2 &b){	return	a.x<b.x;}
inline bool Y_lager_v2(Vector2 &a,Vector2 &b){	return	a.y<b.y;}

//vector3  CMP_RULES
inline bool X_lager_v3(Vector3 &a,Vector3 &b){	return	a.x<b.x;}
inline bool Y_lager_v3(Vector3 &a,Vector3 &b){	return	a.y<b.y;}
inline bool Z_lager_v3(Vector3 &a,Vector3 &b){	return	a.z<b.z;}

