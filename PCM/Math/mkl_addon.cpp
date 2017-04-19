#include "mkl_addon.h"
#include <algorithm>
#include <ctime>
#include <cstdio>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

_NUMC_BEGIN

bool LeastSquareSolver(const CSRMatrix<mklReal> &matA, mklReal *const b, mklReal *const x, int nRhs)
{
	CLeastSquareSpareSolver solver;
	if(solver.init(matA)){
		return solver.solve(b, x, nRhs);
	}
	else
		return false;
}

bool MklSolveSparseSystem(CSRMatrix<mklReal> &matA, mklReal *const b, mklReal *const x, int nRhs)
{
	CSparseSolver solver(&matA);

	if(solver.init())	return solver.solve(b, x, nRhs);

	return false;
}



// Multiplies matA by x and stores the result in b. Assumes all memory has been allocated
// and the sizes match; assumes matA is not symmetric!!
// void MulNonSymmMatrixVector(const CSRMatrix<mklReal> &matA, const mklReal *const x, mklReal *const b)
// {
//
// }


CSparseSolver::CSparseSolver(const CSRMatrix<mklReal> *pMatA)
:_IsInitiated(false)
{
	reset();
	if(pMatA) _matA = *pMatA;
}

CSparseSolver::~CSparseSolver()
{
	reset();
}

void CSparseSolver::reset()
{
	if( _IsInitiated )	{
		//////////////////////////////////////////////////////////////////////////
		// .. Termination and release of memory
		//////////////////////////////////////////////////////////////////////////
		_pphase = -1; /* Release internal memory. */
		_perror = 0;
		PARDISO (&_ppt.front(), &_pmaxfct, &_pmnum, &_pmtype, &_pphase, &_pn, &_pddum, _pia, _pja,
				 &_pidum, &_pRhs, &_piparm.front(), &_pmsglvl, &_pddum, &_pddum, &_perror);
	}

	_ppt.swap( std::vector<void*>(64, (void*)NULL) );
	_piparm.swap( std::vector<int>(64, 0) );
	_pa = NULL;
	_pia = NULL;
	_pja = NULL;
	_pRhs = 1;
	_IsInitiated = false;
}

bool CSparseSolver::init(const CSRMatrix<mklReal> *pMatA)
{
	if(_IsInitiated)	reset();

	if(pMatA)	_matA = *pMatA;

	_matA.ChangeBase(true);
	_pn = _matA._ai.size() - 1;
	const int nnz = _matA._ai.back() - 1;
	if ( (int)_matA._aj.size()!=nnz || (int)_matA._v.size()!=nnz)
		return false;	

	_pa = &_matA._v.front();
	_pia = &_matA._ai.front();
	_pja = &_matA._aj.front();

	_pmtype = _matA._mtype;	// Real unsymmetric matrix
	_piparm[0] = 0;			// No solver default					// revised by jie @ 14/05/2007
// 	_piparm[1] = 2;			// Fill-in reordering from METIS */
// 	_piparm[2] = 1;			// omp_get_max_threads();	/* Numbers of processors, value of OMP_NUM_THREADS */
// 	_piparm[7] = 2;			// Max numbers of iterative refinement steps
// 	_piparm[9] = 13;		// Perturb the pivot elements with 1E-13
// 	_piparm[10] = 1;		// Use nonsymmetric permutation and scaling MPS
// 	_piparm[17] = -1;		// Output: Number of nonzeros in the factor LU
// 	_piparm[18] = -1;		// Output: Mflops for LU factorization
// 	_piparm[19] = 0;		// Output: Numbers of CG Iterations
	_pmaxfct = 1;			// Maximum number of numerical factorizations
	_pmnum = 1;				// Which factorization to use
	//_pmsglvl = 0;			// Print statistical information in file		// changed to static member
	_perror = 0;			// Initialize error flag

	//////////////////////////////////////////////////////////////////////////
	// .. Reordering and Symbolic Factorization. This step also allocates
	// all memory that is necessary for the factorization. */
	//////////////////////////////////////////////////////////////////////////
	_pphase = 11;
	PARDISO (&_ppt.front(), &_pmaxfct, &_pmnum, &_pmtype, &_pphase, &_pn, _pa, _pia, _pja,
	         &_pidum, &_pRhs, &_piparm.front(), &_pmsglvl, &_pddum, &_pddum, &_perror);

	if (_perror != 0) {
		printf("\nERROR during symbolic factorization: %d", _perror);
		return false;
	}
	//////////////////////////////////////////////////////////////////////////
	// .. Numerical factorization
	//////////////////////////////////////////////////////////////////////////
	_pphase = 22;
	PARDISO (&_ppt.front(), &_pmaxfct, &_pmnum, &_pmtype, &_pphase, &_pn, _pa, _pia, _pja,
	         &_pidum, &_pRhs, &_piparm.front(), &_pmsglvl, &_pddum, &_pddum, &_perror);
	// !!!!!!!!! don't use random value for _pRhs

	if (_perror != 0) {
		printf("\nERROR during numerical factorization: %d", _perror);
		return false;
	}

	_IsInitiated = true;
	return true;
}

bool CSparseSolver::solve(double *b, double *x, int nRhs)
{
	//////////////////////////////////////////////////////////////////////////
	// .. Back substitution and iterative refinement
	//////////////////////////////////////////////////////////////////////////
	_pphase = 33;
	_perror = 0;
	_pRhs = nRhs;
	PARDISO (&_ppt.front(), &_pmaxfct, &_pmnum, &_pmtype, &_pphase, &_pn, _pa, _pia, _pja,
	         &_pidum, &_pRhs, &_piparm.front(), &_pmsglvl, b, x, &_perror);
	if (_perror != 0) {
		printf("\nERROR during solution: %d", _perror);
		return false;
	}

  	std::vector<mklReal> y(_pn, 0);
  	if(_matA.issymm())	mkl_dcsrsymv("U", &_pn, _pa, _pia, _pja, x, &y.front());
  	else mkl_dcsrgemv("N", &_pn, _pa, _pia, _pja, x, &y.front());
 	for(int i=0; i<_pn; i++){ y[i] -= b[i];	}

	return true;
}



//////////////////////////////////////////////////////////////////////////
// Least Square Solver
//////////////////////////////////////////////////////////////////////////
bool CLeastSquareSpareSolver::init(const CSRMatrix<mklReal> &matA)
{
	CSRMatrix<mklReal> &matATA = *_sparseSolver.getMatA();
	CSRMatrix<mklReal> &matAT = _matAT;

//	printf("\n\n=====\tLeast square solver initiating ...");
	clock_t t1 = clock();

	/// matrix transposing
//	printf("\n\tMatrix transposing ...");
	clock_t t0 = clock();
	CSRMatrixTranspose(matA, matAT);
//	printf(" finished in %d ms", clock()-t0);

	/// matrix multiplication
//	printf("\n\tMatrix multiplication ...");
	t0 = clock();
	Mul2MatricesSymmResult(matAT, matA, matATA);			//ATA = A' x A
//	printf(" finished in %d ms", clock()-t0);
	_sparseSolver.init();

	/// updating index for matrix A': for rightside updating A'B
//	printf("\n\tSparse matrix index updating for MKL ...");
	t0 = clock();
	const int m = matA._nCol;
	const int n = matA._nRow;
	matAT.ChangeBase(true);
//	printf(" finished in %d ms", clock()-t0);

	return true;
}


int CSparseSolver::_pmsglvl = 0;

_NUMC_END