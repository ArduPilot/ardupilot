// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Désiré Nuentsa-Wakam <desire.nuentsa_wakam@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.


// SparseLU solve does not accept column major matrices for the destination.
// However, as expected, the generic check_sparse_square_solving routines produces row-major
// rhs and destination matrices when compiled with EIGEN_DEFAULT_TO_ROW_MAJOR

#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#undef EIGEN_DEFAULT_TO_ROW_MAJOR
#endif

#include "sparse_solver.h"
#include <Eigen/SparseLU>
#include <unsupported/Eigen/SparseExtra>

template<typename T> void test_sparselu_T()
{
  SparseLU<SparseMatrix<T, ColMajor> /*, COLAMDOrdering<int>*/ > sparselu_colamd; // COLAMDOrdering is the default
  SparseLU<SparseMatrix<T, ColMajor>, AMDOrdering<int> > sparselu_amd; 
  SparseLU<SparseMatrix<T, ColMajor, long int>, NaturalOrdering<long int> > sparselu_natural;
  
  check_sparse_square_solving(sparselu_colamd,  300, 100000, true); 
  check_sparse_square_solving(sparselu_amd,     300,  10000, true);
  check_sparse_square_solving(sparselu_natural, 300,   2000, true);
  
  check_sparse_square_abs_determinant(sparselu_colamd);
  check_sparse_square_abs_determinant(sparselu_amd);
  
  check_sparse_square_determinant(sparselu_colamd);
  check_sparse_square_determinant(sparselu_amd);
}

void test_sparselu()
{
  CALL_SUBTEST_1(test_sparselu_T<float>()); 
  CALL_SUBTEST_2(test_sparselu_T<double>());
  CALL_SUBTEST_3(test_sparselu_T<std::complex<float> >()); 
  CALL_SUBTEST_4(test_sparselu_T<std::complex<double> >());
}
