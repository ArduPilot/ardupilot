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
#include "sparse_solver.h"
#include <Eigen/SparseLU>
#include <Eigen/MetisSupport>
#include <unsupported/Eigen/SparseExtra>

template<typename T> void test_metis_T()
{
  SparseLU<SparseMatrix<T, ColMajor>, MetisOrdering<int> > sparselu_metis;
  
  check_sparse_square_solving(sparselu_metis); 
}

void test_metis_support()
{
  CALL_SUBTEST_1(test_metis_T<double>());
}
