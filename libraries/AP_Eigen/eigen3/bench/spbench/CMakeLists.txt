

set(BLAS_FOUND TRUE)
set(LAPACK_FOUND TRUE)
set(BLAS_LIBRARIES eigen_blas_static)
set(LAPACK_LIBRARIES eigen_lapack_static)

set(SPARSE_LIBS "")

# find_library(PARDISO_LIBRARIES pardiso412-GNU450-X86-64)
# if(PARDISO_LIBRARIES)
#   add_definitions("-DEIGEN_PARDISO_SUPPORT")
#   set(SPARSE_LIBS ${SPARSE_LIBS} ${PARDISO_LIBRARIES})
# endif(PARDISO_LIBRARIES)

find_package(Cholmod)
if(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
  include_directories(${CHOLMOD_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${CHOLMOD_LIBRARIES} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
  set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
endif()

find_package(Umfpack)
if(UMFPACK_FOUND AND BLAS_FOUND)
  add_definitions("-DEIGEN_UMFPACK_SUPPORT")
  include_directories(${UMFPACK_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${UMFPACK_LIBRARIES} ${BLAS_LIBRARIES})
  set(UMFPACK_ALL_LIBS ${UMFPACK_LIBRARIES} ${BLAS_LIBRARIES})
endif()

find_package(SuperLU)
if(SUPERLU_FOUND AND BLAS_FOUND)
  add_definitions("-DEIGEN_SUPERLU_SUPPORT")
  include_directories(${SUPERLU_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${SUPERLU_LIBRARIES} ${BLAS_LIBRARIES})
  set(SUPERLU_ALL_LIBS ${SUPERLU_LIBRARIES} ${BLAS_LIBRARIES})
endif()


find_package(Pastix)
find_package(Scotch)
find_package(Metis)
if(PASTIX_FOUND AND BLAS_FOUND)
  add_definitions("-DEIGEN_PASTIX_SUPPORT")
  include_directories(${PASTIX_INCLUDES})
  if(SCOTCH_FOUND)
    include_directories(${SCOTCH_INCLUDES})
    set(PASTIX_LIBRARIES ${PASTIX_LIBRARIES} ${SCOTCH_LIBRARIES})
  elseif(METIS_FOUND)
    include_directories(${METIS_INCLUDES})
    set(PASTIX_LIBRARIES ${PASTIX_LIBRARIES} ${METIS_LIBRARIES})  
  endif(SCOTCH_FOUND)
  set(SPARSE_LIBS ${SPARSE_LIBS} ${PASTIX_LIBRARIES} ${ORDERING_LIBRARIES} ${BLAS_LIBRARIES})
  set(PASTIX_ALL_LIBS ${PASTIX_LIBRARIES} ${BLAS_LIBRARIES})
endif(PASTIX_FOUND AND BLAS_FOUND)

if(METIS_FOUND)
  include_directories(${METIS_INCLUDES})
  set (SPARSE_LIBS ${SPARSE_LIBS} ${METIS_LIBRARIES})
  add_definitions("-DEIGEN_METIS_SUPPORT")
endif(METIS_FOUND)

find_library(RT_LIBRARY rt)
if(RT_LIBRARY)
  set(SPARSE_LIBS ${SPARSE_LIBS} ${RT_LIBRARY})
endif(RT_LIBRARY)

add_executable(spbenchsolver spbenchsolver.cpp)
target_link_libraries (spbenchsolver ${SPARSE_LIBS})

add_executable(spsolver sp_solver.cpp)
target_link_libraries (spsolver ${SPARSE_LIBS})


add_executable(test_sparseLU test_sparseLU.cpp)
target_link_libraries (test_sparseLU ${SPARSE_LIBS})

