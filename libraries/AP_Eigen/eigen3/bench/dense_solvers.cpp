#include <iostream>
#include "BenchTimer.h"
#include <Eigen/Dense>
#include <map>
#include <string>
using namespace Eigen;

std::map<std::string,Array<float,1,4> > results;

template<typename Scalar,int Size>
void bench(int id, int size = Size)
{
  typedef Matrix<Scalar,Size,Size> Mat;
  Mat A(size,size);
  A.setRandom();
  A = A*A.adjoint();
  BenchTimer t_llt, t_ldlt, t_lu, t_fplu, t_qr, t_cpqr, t_fpqr, t_jsvd;
  
  int tries = 3;
  int rep = 1000/size;
  if(rep==0) rep = 1;
  rep = rep*rep;
  
  LLT<Mat> llt(A);
  LDLT<Mat> ldlt(A);
  PartialPivLU<Mat> lu(A);
  FullPivLU<Mat> fplu(A);
  HouseholderQR<Mat> qr(A);
  ColPivHouseholderQR<Mat> cpqr(A);
  FullPivHouseholderQR<Mat> fpqr(A);
  JacobiSVD<Mat> jsvd(A.rows(),A.cols());
  
  BENCH(t_llt, tries, rep, llt.compute(A));
  BENCH(t_ldlt, tries, rep, ldlt.compute(A));
  BENCH(t_lu, tries, rep, lu.compute(A));
  BENCH(t_fplu, tries, rep, fplu.compute(A));
  BENCH(t_qr, tries, rep, qr.compute(A));
  BENCH(t_cpqr, tries, rep, cpqr.compute(A));
  BENCH(t_fpqr, tries, rep, fpqr.compute(A));
  if(size<500) // JacobiSVD is really too slow for too large matrices
    BENCH(t_jsvd, tries, rep, jsvd.compute(A,ComputeFullU|ComputeFullV));
  
  results["LLT"][id] = t_llt.best();
  results["LDLT"][id] = t_ldlt.best();
  results["PartialPivLU"][id] = t_lu.best();
  results["FullPivLU"][id] = t_fplu.best();
  results["HouseholderQR"][id] = t_qr.best();
  results["ColPivHouseholderQR"][id] = t_cpqr.best();
  results["FullPivHouseholderQR"][id] = t_fpqr.best();
  results["JacobiSVD"][id] = size<500 ? t_jsvd.best() : 0;
}

int main()
{
  const int small = 8;
  const int medium = 100;
  const int large = 1000;
  const int xl = 4000;
  
  bench<float,small>(0);
  bench<float,Dynamic>(1,medium);
  bench<float,Dynamic>(2,large);
  bench<float,Dynamic>(3,xl);
  
  IOFormat fmt(3, 0, " \t", "\n", "", "");
  
  std::cout << "solver/size               " << small << "\t" << medium << "\t" << large << "\t" << xl << "\n";
  std::cout << "LLT                 (ms)  " << (results["LLT"]/1000.).format(fmt) << "\n";
  std::cout << "LDLT                 (%)  " << (results["LDLT"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "PartialPivLU         (%)  " << (results["PartialPivLU"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "FullPivLU            (%)  " << (results["FullPivLU"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "HouseholderQR        (%)  " << (results["HouseholderQR"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "ColPivHouseholderQR  (%)  " << (results["ColPivHouseholderQR"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "FullPivHouseholderQR (%)  " << (results["FullPivHouseholderQR"]/results["LLT"]).format(fmt) << "\n";
  std::cout << "JacobiSVD            (%)  " << (results["JacobiSVD"]/results["LLT"]).format(fmt) << "\n";
}
