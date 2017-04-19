#ifndef NUMC_MKL_ADDON_H_
#define NUMC_MKL_ADDON_H_

#include "spmatrix.h"
#include <mkl.h>
#include <mkl_spblas.h>

#ifdef P4
#undef P4
#endif


_NUMC_BEGIN

template<typename T>
bool PseudoInverseMatrix_BAD(const T *A, int nR, int nC, T *Ainv){
	int   bExcu;       
	T alpha = 1.0;
	T beta = 0.0;
	int   lwork(26*nC);
	T *workMtx = new T[lwork];		
	int *pipiv = new int[lwork];         

	T *AtA = new T[nC*nC];
	std::cout<<"****PseudoInverse : MKL_dsyrk..."; 
	/*AtA = At * A;*/
	dsyrk("U", "N", &nC, &nR, &alpha,A, &nC, &beta, AtA, &nC);
	/*factor*/
	dsytrf ("U", &nC, AtA, &nC, pipiv, workMtx, &lwork, &bExcu);
	/*inverse*/
	dsytri ("U", &nC, AtA, &nC, pipiv, workMtx, &bExcu);
	std::cout<<"done.\n"; 
	if(bExcu!=0){
		std::cout<<"PseudoInverse: factorization is wrong.\n"; 		return false;
	}
	//Ainv = AtAinv * At;
	for(int i=0; i<nC; ++i)		
	for(int j=0; j<nR; ++j){
		T s(0.0);
		for(int k=0; k<i; ++k)
			s+= AtA[i*nC+k] * A[k+nC*j];
		for(int k=i; k<nC; ++k)
			s+= AtA[k*nC+i] * A[k+nC*j];
		Ainv[i*nR+j] = s;		
	}
	SAFEDELETES(AtA);
	SAFEDELETES(pipiv);
	SAFEDELETES(workMtx);
	return true;
}

template<typename T>
bool MulMatrix(const T *A, const T *B,int nRA, int nCA_RB, int nCB, T *AB,
			   const char* MODEA="N",const char* MODEB="N"){  /*MODE: "N": A; "T": A'; "C": conjg(A').*/
    
	T alpha = 1.0, beta = 0.0;
	if(MODEA!="N"&&MODEA!="T"&&MODEA!="C"){
		MSG_BOX_INFO("mkl:gemm, Mode A is not right");	return false;	
	}
	if(MODEB!="N"&&MODEB!="T"&&MODEB!="C"){
		MSG_BOX_INFO("mkl:gemm, Mode B is not right");	return false;	
	}
		/*AB = A*B;*/
	if(sizeof(T) != sizeof(double)){
		MSG_BOX_INFO("change the mkl:gemm to float version, please");
		return false;	
	}
	//std::cout<<"****MulMatrix : MKL_dgemm..."; 
	DGEMM(MODEB, MODEA, &nCB, &nRA, &nCA_RB, &alpha, B, &nCB, A, &nCA_RB,	&beta, AB, &nCB);
	//std::cout<<"done\n"; 
	return true;
}

typedef double mklReal;

bool MklSolveSparseSystem(CSRMatrix<mklReal> &matA, double *const b, double *const x, int nRhs=1);

bool LeastSquareSolver(const CSRMatrix<mklReal> &matA, double *const b, double *const x, int nRhs=1);

inline void MulMatrix2Vector(const CSRMatrix<mklReal> &matA, const double *const b, double *const Ab, int nVect, bool transMatA=false){
	//Ab = A x b
	int n = matA._nCol;

	for(int i=0; i<nVect; i++){
		if( matA.issymm() ){
			// symmetric matrix lower triangle non-zero are saved
			mkl_dcsrsymv("U", &n,
				const_cast<mklReal*>(&matA._v.front()), 
				const_cast<int*>(&matA._ai.front()),
				const_cast<int*>(&matA._aj.front()),
				const_cast<mklReal*>(b+n*i), Ab+n*i);
			return;
		}
		
		mkl_dcsrgemv(transMatA?"T":"N", &n,
			const_cast<mklReal*>(&matA._v.front()), 
			const_cast<int*>(&matA._ai.front()), 
			const_cast<int*>(&matA._aj.front()), 
			const_cast<mklReal*>(b), 
			Ab);

	}
	
}


class CSparseSolver
{
public:
	CSparseSolver(const CSRMatrix<mklReal> *pMatA=NULL);
	~CSparseSolver();
	void reset();
	bool solve(double *b, double *x, int nRhs=1);
	bool init(const CSRMatrix<mklReal> *pMatA=NULL);
	inline CSRMatrix<mklReal>* getMatA()	{ return &_matA; }

private:
	CSRMatrix<mklReal> _matA;
	bool _IsInitiated;

	mklReal *_pa;
	int *_pia;
	int *_pja;

	std::vector<void*> _ppt;		// Internal solver memory pointer
	std::vector<int> _piparm;		// Pardiso control parameters
	int _pmtype;					// Real unsymmetric matrix
	int _pmaxfct, _pmnum, _pphase, _perror;
	/* Auxiliary variables. */
	double _pddum;				// Double dummy
	int _pidum;				// Integer dummy
	int _pn;
	int _pRhs;					// nRhs: Number of right hand sides.

public:
	static int _pmsglvl;
};

class CLeastSquareSpareSolver
{
public:
	CLeastSquareSpareSolver()	{}
	~CLeastSquareSpareSolver()	{}

	inline bool solve(double *b, double *x, int nRhs=1)
	{
		/// right side updating
		int m = _matAT._nRow;
		int n = _matAT._nCol;
		std::vector<mklReal> ATb(m*nRhs, 0);									//ATb = A' x b
		for (int i=0; i<nRhs; i++) {
			mkl_dcsrgemv("N", &n, &_matAT._v.front(), &_matAT._ai.front(), &_matAT._aj.front(), b+i*m, &ATb.front()+i*n);
		}

		/// solver starts
		return _sparseSolver.solve(&ATb.front(), x, nRhs);
	}

	bool init(const CSRMatrix<mklReal> &matA);

private:
	CSRMatrix<mklReal> _matAT;
	CSparseSolver _sparseSolver;
};

typedef std::map<int,double> CSRRow;


// inline void AddCoeff2CSRRow(CSRRow &row, int idx, double val){
// 	if( fabs(val)<std::numeric_limits<double>::epsilon() ) return;
// 	CSRRow::iterator it = row.find(idx);
// 	if(it == row.end()) row.insert( std::make_pair(idx, val) );
// 	else it->second += val;
// }
// 
// inline void Add2RowMat(std::vector<CSRRow>& a, const std::vector<CSRRow>& b, float ca, float cb){
// 	int nrow = a.size();
// 	assert(nrow == (int)b.size());
// 	for(int i=0; i<nrow; i++){
// 		for(CSRRow::iterator ait=a[i].begin(); ait!=a[i].end(); ++ait){
// 			ait->second *= ca;
// 		}
// 		
// 		for(CSRRow::const_iterator bit=b[i].begin(); bit!=b[i].end(); ++bit){
// 			AddCoeff2CSRRow(a[i], bit->first, bit->second*cb);
// 		}
// 	}
// }

_NUMC_END

#endif		//#def NUMC_MKL_ADDON_H