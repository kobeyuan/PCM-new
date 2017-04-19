#pragma   warning(disable:   4786)

#ifndef NUMC_CSRMATRIX_H
#define NUMC_CSRMATRIX_H

#include <cmath>
#include <map>
#include <vector>
#include <limits>
#include <cassert>
#include <algorithm>

using std::vector;
using std::map;

#ifndef _NUMC_BEGIN
	#define _NUMC_BEGIN	namespace numc {
	#define _NUMC_END	}
#endif

_NUMC_BEGIN

template <typename T> class CSRMatrix;
template <typename T> struct RowMat;
template <typename T>
void CreateCSRMatrixFromRowMap(CSRMatrix<T> &matC, const RowMat<T>& rowC);

template <typename T>
class CSRMatrix
{
public:
	enum MatType{ RealStrucSymm=1, RealSymmPosDef=2, RealSymmIndef=-2, 
		CompStrucSymm=3, CompHermPosDef=4, CompHermIndef=-4, 
		CompSymm=6, RealUnSymm=11, CompUnSymm=13};
	int _nCol, _nRow;

	MatType _mtype;
	
	bool _onebase;
	std::vector<T> _v;
	std::vector<int> _ai, _aj;
	CSRMatrix<T>():_nCol(0),_nRow(0),_mtype(RealUnSymm),_onebase(false) {};
	CSRMatrix<T>(const RowMat<T> &rm):_mtype(RealUnSymm)	{ CreateCSRMatrixFromRowMap(*this, rm); }

public:

	bool issymm() const{
		return (_mtype == RealSymmIndef || _mtype == RealSymmPosDef || _mtype == CompSymm )?true:false;
	}

	inline void clear() {
		_nCol = 0; _nRow = 0; _mtype = RealUnSymm; _onebase = false; _v.clear(); _ai.clear(); _aj.clear();
	}

	T getElementV(int x, int y) const{
		double *p = const_cast<CSRMatrix*>(this)->getElementP(x, y);
		return (NULL==p)?0:*p;	
	}

	T* getElementP(int x, int y) {
		if (x<0||y<0)	return NULL;
		if ( issymm() && x>y) swap(x, y);
		int off = _onebase;
		std::vector<int>::const_iterator it = lower_bound(_aj.begin()+_ai[x]-off, _aj.begin()+(_ai[x+1]-off), y+off);
		if (it==_aj.begin()+(_ai[x+1]-off) || *it!=(y+off) ) return NULL;

		return &_v.front() + ( it-_aj.begin() );
	}

	void MultiVect(T* in, T *out) const;
	
	bool ChangeBase(bool oneBase) {
		if(_onebase == oneBase)	return true;
		int ii;
		if (oneBase) {
			assert(this->_ai[0] == 0);
			if (_ai[0] != 0)
				return false;

			for (ii=0; ii<_ai.back(); ii++) {
				_aj[ii]++;
			}
			for (ii=0; ii<_nCol+1; ii++) {
				_ai[ii]++;
			}
		}
		else {
			assert(_ai[0] == 1);
			if (_ai[0] != 1)
				return false;

			for (ii=0; ii<_ai.back()-1; ii++) {
				_aj[ii]--;
			}
			for (ii=0; ii<_nCol+1; ii++) {
				_ai[ii]--;
			}
		}
		_onebase = oneBase;
		return true;
	}


	void GetSubMat(const std::vector<int> &xi, const std::vector<int> &yi, CSRMatrix& mat){
		mat.resize( yi.size() );
		for(int j=0; j<yi.size(); j++){
			for(int i=0; i<xi.size(); i++){

			}
		}
		
	}
};

template<typename T>
struct RowMat:public vector<map<int, T> >
{
	int _nCol;

	RowMat<T>(int nRow=0, int nCol=0):vector<map<int, T> >(nRow),_nCol(nCol)	{	}
	~RowMat<T>()	{ }

	RowMat<T>(const CSRMatrix<T> &mat) {
		clear();
		resize(mat._nRow, mat._nCol);

		int off = mat._onebase?1:0;
		for(unsigned int i=0; i+1<mat._ai.size(); i++)
			for(int j=mat._ai[i]; j<mat._ai[i+1]; j++)		
				(*this)[i][ mat._aj[j-off]-off ] = mat._v[ j-off ];
	}

	void resize(int nRow, int nCol=0) { vector<map<int, T> >::resize(nRow); _nCol = nCol>0?nCol:_nCol; }

	int nRow()	const { return vector<map<int, T> >::size(); }
	int nCol()	const { return _nCol; }

	int operator+=(const RowMat<T>& r){ /* 2 rowmatrix plus*/
		const int h = nRow();
		if( r.nRow() != h )	return -1;

		for(int i=0; i<h; i++){
			const map<int, T>& rrow = r[i];
			map<int, T>& lrow = (*this)[i];
			for(map<int,T>::const_iterator it=rrow.begin(); it!=rrow.end(); ++it){
				int col = it->first;
				T val = it->second;

				map<int,T>::iterator f = lrow.find(col);
				if( f != lrow.end() ){
					f->second += val;
				}
				else{
					lrow.insert( make_pair<int, T>(col, val) );
				}
			}
		}
		return 0;
	}

	template<typename _aT>  
	RowMat<T>& operator*=(_aT s){
		const int h = nRow();
		for(int i=0; i<h; i++){
			map<int, T>& lrow = (*this)[i];
			for(map<int,T>::iterator it=lrow.begin(); it!=lrow.end(); ++it)
				it->second*=s;
		}
		return *this;
	}
};



//////////////////////////////////////////////////////////////////////////
/// Get SubMatrix from rowmap
//////////////////////////////////////////////////////////////////////////
template <typename T>
void GetSubRowMap(const RowMat<T>& src,
				  const std::vector<int> &xi, const std::vector<int> &yi,
				  RowMat<T>& dst)
{
	assert( xi.size()>0 && yi.size()>0 );
	assert( yi.back() < (int)src.size() );
	if( xi.size()==0 || yi.size()==0 || yi.back() >= (int)src.size() )	return;
	

	typedef std::map<int, T> row;
	dst.resize( yi.size() );

	for(unsigned int j=0; j<yi.size(); j++){
		const row &srow = src[ yi[j] ];
		row &drow = dst[j];

		const int* pxi = &xi.front();

		for(row::const_iterator it=srow.begin(); it!=srow.end(); ++it ){
			pxi = std::lower_bound(pxi, &xi.back(), it->first);
			
			if(*pxi != it->first){			// done
				continue;
			}

			int col = pxi - &xi.front();
			drow[col] = it->second;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/// Create CSR matrix from vector of rowmap
//////////////////////////////////////////////////////////////////////////
template <typename T>
void CreateCSRMatrixFromRowMap(CSRMatrix<T> &matC, const RowMat<T>& rowC)
{
	if(rowC._nCol <= 0){
		fprintf(stderr, "\nnumber of column not specified in the RowMat, exit!");
		return;
	}
	matC.clear();
	matC._nCol = rowC.size();
	matC._nRow = rowC._nCol;
	matC._onebase = false;

	matC._ai.reserve(matC._nCol+1);
	int i,nnz=0;
	for (i=0; i<matC._nCol; i++) {
		nnz += rowC[i].size();
	}
	matC._aj.reserve(nnz);
	matC._v.reserve(nnz);

	// copy rows into matC
	matC._ai.push_back(0);
	for (i=0; i<matC._nCol; i++) {
		matC._ai.push_back(matC._ai.back());
		for (std::map<int, T>::const_iterator it=rowC[i].begin(); it!=rowC[i].end(); it++) {
			if(fabs(it->second) > std::numeric_limits<double>::epsilon()){
				matC._ai.back()++;
				matC._aj.push_back(it->first);
				matC._v.push_back(it->second);
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////
/// Computes the transpose of a matrix.
template <typename T>
void CSRMatrixTranspose(const CSRMatrix<T> &matA, CSRMatrix<T> &matAT)
{
	if (matA.issymm()) {		// symmetric - just copy the matrix
		matAT = matA;
		return;
	}

	matAT._nCol = matA._nRow;
	matAT._nRow = matA._nCol;
	matAT._mtype = matA._mtype;

	// non-symmetric matrix -> need to build data structure.
	// we'll go over the columns and build the rows
	int off = matA._onebase?1:0;
	RowMat<T> rowC(matA._nRow, matA._nCol);
	for (int i=0; i<matA._nCol; i++) {
		for (int j=matA._ai[i]; j<matA._ai[i+1]; j++) {
			rowC[matA._aj[j-off]-off][i] = matA._v[j-off];
		}
	}

	CreateCSRMatrixFromRowMap(matAT, rowC);
}


//////////////////////////////////////////////////////////////////////////
// multiplication of sparse matrix
// Assuming nothing about the result (the result is not stored symmetric)
//////////////////////////////////////////////////////////////////////////
template <typename T>
bool Mul2Matrices(const CSRMatrix<T> &matA, const CSRMatrix<T> &matB,
				  RowMat<T> &rowsC)
{
	if(matA._onebase || matB._onebase){
		fprintf(stderr, "\nmatrix saved in 1-based format, pleased converted it to 0-based and try again!");
		return false;
	}
	// Compatibility of dimensions
	if (matA._nRow != matB._nCol)
		return false;

	// (m x n)*(n x k) = (m x k)
	const int m=matA._nCol;
	const int n=matA._nRow;
	const int k=matB._nRow;
	rowsC.resize(m);

	T aiv, valB;
	int colInd, colB;

	for (int i=0; i<m; ++i) {					// creating row i of C
		std::map<int, T> &mapRow2Val = rowsC[i];
		for (int iAi = matA._ai[i]; iAi < matA._ai[i+1]; ++iAi) {			// travel on ai
			colInd = matA._aj[iAi];
			aiv = matA._v[iAi];
			// make aiv*b_{rowInd} and insert into mapRow2Val
			for (int iB=matB._ai[colInd]; iB<matB._ai[colInd+1]; ++iB) {
				colB=matB._aj[iB];
				valB=matB._v[iB];
				// insert valA*aiv into map
				std::map<int, T>::iterator it = mapRow2Val.find(colB);
				if (it == mapRow2Val.end()) {		// first time
					mapRow2Val[colB] = valB*aiv;
				}
				else {
					it->second = it->second + valB*aiv;
				}
			}
		}
	}// now column i is created

	return true;
}

template <typename T>
bool Mul2Matrices(const CSRMatrix<T> &matA, const CSRMatrix<T> &matB, CSRMatrix<T> &matC)
{
	RowMat<T> rowsC;
	if( !Mul2Matrices(matA, matB, rowsC) ) return false;

	const int k=matB._nRow;
	rowsC._nCol = k;
	CreateCSRMatrixFromRowMap(matC, rowsC);						// modified by jianwei hu @ 16/09/07
	return true;
}

//////////////////////////////////////////////////////////////////////////
// multiplication of sparse matrix
// The result is symmetric
//////////////////////////////////////////////////////////////////////////
template <typename T>
bool Mul2MatricesSymmResult(const CSRMatrix<T> &matA, const CSRMatrix<T> &matB,
							RowMat<T> &rowsC)
{
	if(matA._onebase || matB._onebase){
		fprintf(stderr, "\nmatrix saved in 1-based format, pleased converted it to 0-based and try again!");
		return false;
	}
	// Compatibility of dimensions
	if (matA._nRow != matB._nCol || matA._nCol != matB._nRow)	return false;
	
	// (m x n)*(n x m) = (m x m)
	const int m=matA._nCol;
	const int n=matA._nRow;
	
	rowsC.resize(m);

	T aiv, valB;
	int colInd, colB;

	for (int i=0; i<m; ++i) {					// creating row i of C
		std::map<int, T> &mapRow2Val = rowsC[i];
		for (int iAi = matA._ai[i]; iAi < matA._ai[i+1]; ++iAi) {			// travel on ai
			colInd = matA._aj[iAi];
			aiv = matA._v[iAi];
			// make aiv*b_{colInd} and insert into mapRow2Val
			for (int iB=matB._ai[colInd]; iB<matB._ai[colInd+1]; ++iB) {
				colB=matB._aj[iB];
				if (colB >= i) {
					valB=matB._v[iB];
					// insert valA*aiv into map
					std::map<int, T>::iterator it = mapRow2Val.find(colB);
					if (it == mapRow2Val.end()) {		// first time
						mapRow2Val[colB] = valB*aiv;
					}
					else {
						it->second = it->second + valB*aiv;
					}
				}
			}
		}
	}// now column i is created
	
	return true;
}

template <typename T>
bool Mul2MatricesSymmResult(const CSRMatrix<T> &matA, const CSRMatrix<T> &matB, CSRMatrix<T> &matC)
{
	RowMat<T> rowsC;
	if( !Mul2MatricesSymmResult(matA, matB, rowsC) )	return false;
	
	rowsC._nCol = matA._nCol;
	CreateCSRMatrixFromRowMap(matC, rowsC);
	matC._mtype = CSRMatrix<T>::RealSymmIndef;
	return true;
}


template<typename T>
void DebugShowRowMatrix(const std::vector<std::map<int,T> > &rowA)
{
	printf("\n");
	for(unsigned int i=0; i<rowA.size(); i++){
		printf("\n%3d#", i);
		std::map<int,T>::const_iterator it=rowA[i].begin();
		int j=0;
		for(; it!=rowA[i].end(); ++it){
			for(int k=j; k<it->first; k++){
				printf("\t%3.2f", 0);
			}
			
			j=it->first+1;
			printf("\t%3.2f", it->second);
		}
	}
}


_NUMC_END

#endif		// NUMC_CSRMATRIX_H