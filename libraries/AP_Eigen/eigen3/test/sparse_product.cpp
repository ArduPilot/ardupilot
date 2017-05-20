// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "sparse.h"

template<typename SparseMatrixType> void sparse_product()
{
  typedef typename SparseMatrixType::StorageIndex StorageIndex;
  Index n = 100;
  const Index rows  = internal::random<Index>(1,n);
  const Index cols  = internal::random<Index>(1,n);
  const Index depth = internal::random<Index>(1,n);
  typedef typename SparseMatrixType::Scalar Scalar;
  enum { Flags = SparseMatrixType::Flags };

  double density = (std::max)(8./(rows*cols), 0.2);
  typedef Matrix<Scalar,Dynamic,Dynamic> DenseMatrix;
  typedef Matrix<Scalar,Dynamic,1> DenseVector;
  typedef Matrix<Scalar,1,Dynamic> RowDenseVector;
  typedef SparseVector<Scalar,0,StorageIndex> ColSpVector;
  typedef SparseVector<Scalar,RowMajor,StorageIndex> RowSpVector;

  Scalar s1 = internal::random<Scalar>();
  Scalar s2 = internal::random<Scalar>();

  // test matrix-matrix product
  {
    DenseMatrix refMat2  = DenseMatrix::Zero(rows, depth);
    DenseMatrix refMat2t = DenseMatrix::Zero(depth, rows);
    DenseMatrix refMat3  = DenseMatrix::Zero(depth, cols);
    DenseMatrix refMat3t = DenseMatrix::Zero(cols, depth);
    DenseMatrix refMat4  = DenseMatrix::Zero(rows, cols);
    DenseMatrix refMat4t = DenseMatrix::Zero(cols, rows);
    DenseMatrix refMat5  = DenseMatrix::Random(depth, cols);
    DenseMatrix refMat6  = DenseMatrix::Random(rows, rows);
    DenseMatrix dm4 = DenseMatrix::Zero(rows, rows);
//     DenseVector dv1 = DenseVector::Random(rows);
    SparseMatrixType m2 (rows, depth);
    SparseMatrixType m2t(depth, rows);
    SparseMatrixType m3 (depth, cols);
    SparseMatrixType m3t(cols, depth);
    SparseMatrixType m4 (rows, cols);
    SparseMatrixType m4t(cols, rows);
    SparseMatrixType m6(rows, rows);
    initSparse(density, refMat2,  m2);
    initSparse(density, refMat2t, m2t);
    initSparse(density, refMat3,  m3);
    initSparse(density, refMat3t, m3t);
    initSparse(density, refMat4,  m4);
    initSparse(density, refMat4t, m4t);
    initSparse(density, refMat6, m6);

//     int c = internal::random<int>(0,depth-1);

    // sparse * sparse
    VERIFY_IS_APPROX(m4=m2*m3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(m4=m2t.transpose()*m3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(m4=m2t.transpose()*m3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());
    VERIFY_IS_APPROX(m4=m2*m3t.transpose(), refMat4=refMat2*refMat3t.transpose());

    VERIFY_IS_APPROX(m4 = m2*m3/s1, refMat4 = refMat2*refMat3/s1);
    VERIFY_IS_APPROX(m4 = m2*m3*s1, refMat4 = refMat2*refMat3*s1);
    VERIFY_IS_APPROX(m4 = s2*m2*m3*s1, refMat4 = s2*refMat2*refMat3*s1);
    VERIFY_IS_APPROX(m4 = (m2+m2)*m3, refMat4 = (refMat2+refMat2)*refMat3);
    VERIFY_IS_APPROX(m4 = m2*m3.leftCols(cols/2), refMat4 = refMat2*refMat3.leftCols(cols/2));
    VERIFY_IS_APPROX(m4 = m2*(m3+m3).leftCols(cols/2), refMat4 = refMat2*(refMat3+refMat3).leftCols(cols/2));

    VERIFY_IS_APPROX(m4=(m2*m3).pruned(0), refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(m4=(m2t.transpose()*m3).pruned(0), refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(m4=(m2t.transpose()*m3t.transpose()).pruned(0), refMat4=refMat2t.transpose()*refMat3t.transpose());
    VERIFY_IS_APPROX(m4=(m2*m3t.transpose()).pruned(0), refMat4=refMat2*refMat3t.transpose());

    // test aliasing
    m4 = m2; refMat4 = refMat2;
    VERIFY_IS_APPROX(m4=m4*m3, refMat4=refMat4*refMat3);

    // sparse * dense matrix
    VERIFY_IS_APPROX(dm4=m2*refMat3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=m2*refMat3t.transpose(), refMat4=refMat2*refMat3t.transpose());
    VERIFY_IS_APPROX(dm4=m2t.transpose()*refMat3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(dm4=m2t.transpose()*refMat3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());

    VERIFY_IS_APPROX(dm4=m2*refMat3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=dm4+m2*refMat3, refMat4=refMat4+refMat2*refMat3);
    VERIFY_IS_APPROX(dm4+=m2*refMat3, refMat4+=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4-=m2*refMat3, refMat4-=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4.noalias()+=m2*refMat3, refMat4+=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4.noalias()-=m2*refMat3, refMat4-=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=m2*(refMat3+refMat3), refMat4=refMat2*(refMat3+refMat3));
    VERIFY_IS_APPROX(dm4=m2t.transpose()*(refMat3+refMat5)*0.5, refMat4=refMat2t.transpose()*(refMat3+refMat5)*0.5);
    
    // sparse * dense vector
    VERIFY_IS_APPROX(dm4.col(0)=m2*refMat3.col(0), refMat4.col(0)=refMat2*refMat3.col(0));
    VERIFY_IS_APPROX(dm4.col(0)=m2*refMat3t.transpose().col(0), refMat4.col(0)=refMat2*refMat3t.transpose().col(0));
    VERIFY_IS_APPROX(dm4.col(0)=m2t.transpose()*refMat3.col(0), refMat4.col(0)=refMat2t.transpose()*refMat3.col(0));
    VERIFY_IS_APPROX(dm4.col(0)=m2t.transpose()*refMat3t.transpose().col(0), refMat4.col(0)=refMat2t.transpose()*refMat3t.transpose().col(0));

    // dense * sparse
    VERIFY_IS_APPROX(dm4=refMat2*m3, refMat4=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=dm4+refMat2*m3, refMat4=refMat4+refMat2*refMat3);
    VERIFY_IS_APPROX(dm4+=refMat2*m3, refMat4+=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4-=refMat2*m3, refMat4-=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4.noalias()+=refMat2*m3, refMat4+=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4.noalias()-=refMat2*m3, refMat4-=refMat2*refMat3);
    VERIFY_IS_APPROX(dm4=refMat2*m3t.transpose(), refMat4=refMat2*refMat3t.transpose());
    VERIFY_IS_APPROX(dm4=refMat2t.transpose()*m3, refMat4=refMat2t.transpose()*refMat3);
    VERIFY_IS_APPROX(dm4=refMat2t.transpose()*m3t.transpose(), refMat4=refMat2t.transpose()*refMat3t.transpose());

    // sparse * dense and dense * sparse outer product
    {
      Index c  = internal::random<Index>(0,depth-1);
      Index r  = internal::random<Index>(0,rows-1);
      Index c1 = internal::random<Index>(0,cols-1);
      Index r1 = internal::random<Index>(0,depth-1);
      DenseMatrix dm5  = DenseMatrix::Random(depth, cols);

      VERIFY_IS_APPROX( m4=m2.col(c)*dm5.col(c1).transpose(), refMat4=refMat2.col(c)*dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX( m4=m2.middleCols(c,1)*dm5.col(c1).transpose(), refMat4=refMat2.col(c)*dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=m2.col(c)*dm5.col(c1).transpose(), refMat4=refMat2.col(c)*dm5.col(c1).transpose());
      
      VERIFY_IS_APPROX(m4=dm5.col(c1)*m2.col(c).transpose(), refMat4=dm5.col(c1)*refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(m4=dm5.col(c1)*m2.middleCols(c,1).transpose(), refMat4=dm5.col(c1)*refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=dm5.col(c1)*m2.col(c).transpose(), refMat4=dm5.col(c1)*refMat2.col(c).transpose());

      VERIFY_IS_APPROX( m4=dm5.row(r1).transpose()*m2.col(c).transpose(), refMat4=dm5.row(r1).transpose()*refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=dm5.row(r1).transpose()*m2.col(c).transpose(), refMat4=dm5.row(r1).transpose()*refMat2.col(c).transpose());

      VERIFY_IS_APPROX( m4=m2.row(r).transpose()*dm5.col(c1).transpose(), refMat4=refMat2.row(r).transpose()*dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX( m4=m2.middleRows(r,1).transpose()*dm5.col(c1).transpose(), refMat4=refMat2.row(r).transpose()*dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=m2.row(r).transpose()*dm5.col(c1).transpose(), refMat4=refMat2.row(r).transpose()*dm5.col(c1).transpose());

      VERIFY_IS_APPROX( m4=dm5.col(c1)*m2.row(r), refMat4=dm5.col(c1)*refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX( m4=dm5.col(c1)*m2.middleRows(r,1), refMat4=dm5.col(c1)*refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=dm5.col(c1)*m2.row(r), refMat4=dm5.col(c1)*refMat2.row(r));

      VERIFY_IS_APPROX( m4=dm5.row(r1).transpose()*m2.row(r), refMat4=dm5.row(r1).transpose()*refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array()!=0).count());
      VERIFY_IS_APPROX(dm4=dm5.row(r1).transpose()*m2.row(r), refMat4=dm5.row(r1).transpose()*refMat2.row(r));
    }

    VERIFY_IS_APPROX(m6=m6*m6, refMat6=refMat6*refMat6);
    
    // sparse matrix * sparse vector
    ColSpVector cv0(cols), cv1;
    DenseVector dcv0(cols), dcv1;
    initSparse(2*density,dcv0, cv0);
    
    RowSpVector rv0(depth), rv1;
    RowDenseVector drv0(depth), drv1(rv1);
    initSparse(2*density,drv0, rv0);

    VERIFY_IS_APPROX(cv1=m3*cv0, dcv1=refMat3*dcv0);    
    VERIFY_IS_APPROX(rv1=rv0*m3, drv1=drv0*refMat3);
    VERIFY_IS_APPROX(cv1=m3t.adjoint()*cv0, dcv1=refMat3t.adjoint()*dcv0);
    VERIFY_IS_APPROX(cv1=rv0*m3, dcv1=drv0*refMat3);
    VERIFY_IS_APPROX(rv1=m3*cv0, drv1=refMat3*dcv0);
  }
  
  // test matrix - diagonal product
  {
    DenseMatrix refM2 = DenseMatrix::Zero(rows, cols);
    DenseMatrix refM3 = DenseMatrix::Zero(rows, cols);
    DenseMatrix d3 = DenseMatrix::Zero(rows, cols);
    DiagonalMatrix<Scalar,Dynamic> d1(DenseVector::Random(cols));
    DiagonalMatrix<Scalar,Dynamic> d2(DenseVector::Random(rows));
    SparseMatrixType m2(rows, cols);
    SparseMatrixType m3(rows, cols);
    initSparse<Scalar>(density, refM2, m2);
    initSparse<Scalar>(density, refM3, m3);
    VERIFY_IS_APPROX(m3=m2*d1, refM3=refM2*d1);
    VERIFY_IS_APPROX(m3=m2.transpose()*d2, refM3=refM2.transpose()*d2);
    VERIFY_IS_APPROX(m3=d2*m2, refM3=d2*refM2);
    VERIFY_IS_APPROX(m3=d1*m2.transpose(), refM3=d1*refM2.transpose());
    
    // also check with a SparseWrapper:
    DenseVector v1 = DenseVector::Random(cols);
    DenseVector v2 = DenseVector::Random(rows);
    VERIFY_IS_APPROX(m3=m2*v1.asDiagonal(), refM3=refM2*v1.asDiagonal());
    VERIFY_IS_APPROX(m3=m2.transpose()*v2.asDiagonal(), refM3=refM2.transpose()*v2.asDiagonal());
    VERIFY_IS_APPROX(m3=v2.asDiagonal()*m2, refM3=v2.asDiagonal()*refM2);
    VERIFY_IS_APPROX(m3=v1.asDiagonal()*m2.transpose(), refM3=v1.asDiagonal()*refM2.transpose());
    
    VERIFY_IS_APPROX(m3=v2.asDiagonal()*m2*v1.asDiagonal(), refM3=v2.asDiagonal()*refM2*v1.asDiagonal());
    
    // evaluate to a dense matrix to check the .row() and .col() iterator functions
    VERIFY_IS_APPROX(d3=m2*d1, refM3=refM2*d1);
    VERIFY_IS_APPROX(d3=m2.transpose()*d2, refM3=refM2.transpose()*d2);
    VERIFY_IS_APPROX(d3=d2*m2, refM3=d2*refM2);
    VERIFY_IS_APPROX(d3=d1*m2.transpose(), refM3=d1*refM2.transpose());
  }

  // test self-adjoint and triangular-view products
  {
    DenseMatrix b = DenseMatrix::Random(rows, rows);
    DenseMatrix x = DenseMatrix::Random(rows, rows);
    DenseMatrix refX = DenseMatrix::Random(rows, rows);
    DenseMatrix refUp = DenseMatrix::Zero(rows, rows);
    DenseMatrix refLo = DenseMatrix::Zero(rows, rows);
    DenseMatrix refS = DenseMatrix::Zero(rows, rows);
    DenseMatrix refA = DenseMatrix::Zero(rows, rows);
    SparseMatrixType mUp(rows, rows);
    SparseMatrixType mLo(rows, rows);
    SparseMatrixType mS(rows, rows);
    SparseMatrixType mA(rows, rows);
    initSparse<Scalar>(density, refA, mA);
    do {
      initSparse<Scalar>(density, refUp, mUp, ForceRealDiag|/*ForceNonZeroDiag|*/MakeUpperTriangular);
    } while (refUp.isZero());
    refLo = refUp.adjoint();
    mLo = mUp.adjoint();
    refS = refUp + refLo;
    refS.diagonal() *= 0.5;
    mS = mUp + mLo;
    // TODO be able to address the diagonal....
    for (int k=0; k<mS.outerSize(); ++k)
      for (typename SparseMatrixType::InnerIterator it(mS,k); it; ++it)
        if (it.index() == k)
          it.valueRef() *= 0.5;

    VERIFY_IS_APPROX(refS.adjoint(), refS);
    VERIFY_IS_APPROX(mS.adjoint(), mS);
    VERIFY_IS_APPROX(mS, refS);
    VERIFY_IS_APPROX(x=mS*b, refX=refS*b);

    // sparse selfadjointView with dense matrices
    VERIFY_IS_APPROX(x=mUp.template selfadjointView<Upper>()*b, refX=refS*b);
    VERIFY_IS_APPROX(x=mLo.template selfadjointView<Lower>()*b, refX=refS*b);
    VERIFY_IS_APPROX(x=mS.template selfadjointView<Upper|Lower>()*b, refX=refS*b);
    
    // sparse selfadjointView with sparse matrices
    SparseMatrixType mSres(rows,rows);
    VERIFY_IS_APPROX(mSres = mLo.template selfadjointView<Lower>()*mS,
                     refX = refLo.template selfadjointView<Lower>()*refS);
    VERIFY_IS_APPROX(mSres = mS * mLo.template selfadjointView<Lower>(),
                     refX = refS * refLo.template selfadjointView<Lower>());
    
    // sparse triangularView with dense matrices
    VERIFY_IS_APPROX(x=mA.template triangularView<Upper>()*b, refX=refA.template triangularView<Upper>()*b);
    VERIFY_IS_APPROX(x=mA.template triangularView<Lower>()*b, refX=refA.template triangularView<Lower>()*b);
    VERIFY_IS_APPROX(x=b*mA.template triangularView<Upper>(), refX=b*refA.template triangularView<Upper>());
    VERIFY_IS_APPROX(x=b*mA.template triangularView<Lower>(), refX=b*refA.template triangularView<Lower>());
    
    // sparse triangularView with sparse matrices
    VERIFY_IS_APPROX(mSres = mA.template triangularView<Lower>()*mS,   refX = refA.template triangularView<Lower>()*refS);
    VERIFY_IS_APPROX(mSres = mS * mA.template triangularView<Lower>(), refX = refS * refA.template triangularView<Lower>());
    VERIFY_IS_APPROX(mSres = mA.template triangularView<Upper>()*mS,   refX = refA.template triangularView<Upper>()*refS);
    VERIFY_IS_APPROX(mSres = mS * mA.template triangularView<Upper>(), refX = refS * refA.template triangularView<Upper>());
  }
}

// New test for Bug in SparseTimeDenseProduct
template<typename SparseMatrixType, typename DenseMatrixType> void sparse_product_regression_test()
{
  // This code does not compile with afflicted versions of the bug
  SparseMatrixType sm1(3,2);
  DenseMatrixType m2(2,2);
  sm1.setZero();
  m2.setZero();

  DenseMatrixType m3 = sm1*m2;


  // This code produces a segfault with afflicted versions of another SparseTimeDenseProduct
  // bug

  SparseMatrixType sm2(20000,2);
  sm2.setZero();
  DenseMatrixType m4(sm2*m2);

  VERIFY_IS_APPROX( m4(0,0), 0.0 );
}

template<typename Scalar>
void bug_942()
{
  typedef Matrix<Scalar, Dynamic, 1>     Vector;
  typedef SparseMatrix<Scalar, ColMajor> ColSpMat;
  typedef SparseMatrix<Scalar, RowMajor> RowSpMat;
  ColSpMat cmA(1,1);
  cmA.insert(0,0) = 1;

  RowSpMat rmA(1,1);
  rmA.insert(0,0) = 1;

  Vector d(1);
  d[0] = 2;
  
  double res = 2;
  
  VERIFY_IS_APPROX( ( cmA*d.asDiagonal() ).eval().coeff(0,0), res );
  VERIFY_IS_APPROX( ( d.asDiagonal()*rmA ).eval().coeff(0,0), res );
  VERIFY_IS_APPROX( ( rmA*d.asDiagonal() ).eval().coeff(0,0), res );
  VERIFY_IS_APPROX( ( d.asDiagonal()*cmA ).eval().coeff(0,0), res );
}

void test_sparse_product()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1( (sparse_product<SparseMatrix<double,ColMajor> >()) );
    CALL_SUBTEST_1( (sparse_product<SparseMatrix<double,RowMajor> >()) );
    CALL_SUBTEST_1( (bug_942<double>()) );
    CALL_SUBTEST_2( (sparse_product<SparseMatrix<std::complex<double>, ColMajor > >()) );
    CALL_SUBTEST_2( (sparse_product<SparseMatrix<std::complex<double>, RowMajor > >()) );
    CALL_SUBTEST_3( (sparse_product<SparseMatrix<float,ColMajor,long int> >()) );
    CALL_SUBTEST_4( (sparse_product_regression_test<SparseMatrix<double,RowMajor>, Matrix<double, Dynamic, Dynamic, RowMajor> >()) );
  }
}
