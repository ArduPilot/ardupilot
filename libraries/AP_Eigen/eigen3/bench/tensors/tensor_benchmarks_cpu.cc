#define EIGEN_USE_THREADS

#include "base/sysinfo.h"
#include "strings/strcat.h"
#include "third_party/eigen3/tensor_benchmarks.h"
#include "thread/threadpool.h"

#ifdef __ANDROID__
#define CREATE_THREAD_POOL(threads)             \
Eigen::ThreadPoolDevice device(threads);
#else
#define CREATE_THREAD_POOL(threads)             \
ThreadPool tp(threads);                         \
tp.StartWorkers();                              \
Eigen::ThreadPoolDevice device(&tp, threads);
#endif

// Simple functions
#define BM_FuncCPU(FUNC, THREADS)                                \
  static void BM_##FUNC##_##THREADS##T(int iters, int N) {       \
    StopBenchmarkTiming();                                       \
    CREATE_THREAD_POOL(THREADS);                                 \
    BenchmarkSuite<Eigen::ThreadPoolDevice> suite(device, N);    \
    suite.FUNC(iters);                                           \
    SetBenchmarkLabel(StrCat("using ", THREADS, " threads"));    \
  }                                                              \
  BENCHMARK_RANGE(BM_##FUNC##_##THREADS##T, 10, 5000);

BM_FuncCPU(memcpy, 4);
BM_FuncCPU(memcpy, 8);
BM_FuncCPU(memcpy, 12);

BM_FuncCPU(random, 4);
BM_FuncCPU(random, 8);
BM_FuncCPU(random, 12);

BM_FuncCPU(slicing, 4);
BM_FuncCPU(slicing, 8);
BM_FuncCPU(slicing, 12);

BM_FuncCPU(shuffling, 4);
BM_FuncCPU(shuffling, 8);
BM_FuncCPU(shuffling, 12);

BM_FuncCPU(padding, 4);
BM_FuncCPU(padding, 8);
BM_FuncCPU(padding, 12);

BM_FuncCPU(striding, 4);
BM_FuncCPU(striding, 8);
BM_FuncCPU(striding, 12);

BM_FuncCPU(broadcasting, 4);
BM_FuncCPU(broadcasting, 8);
BM_FuncCPU(broadcasting, 12);

BM_FuncCPU(coeffWiseOp, 4);
BM_FuncCPU(coeffWiseOp, 8);
BM_FuncCPU(coeffWiseOp, 12);

BM_FuncCPU(algebraicFunc, 4);
BM_FuncCPU(algebraicFunc, 8);
BM_FuncCPU(algebraicFunc, 12);

BM_FuncCPU(transcendentalFunc, 4);
BM_FuncCPU(transcendentalFunc, 8);
BM_FuncCPU(transcendentalFunc, 12);

BM_FuncCPU(reduction, 4);
BM_FuncCPU(reduction, 8);
BM_FuncCPU(reduction, 12);


// Contractions
#define BM_FuncWithInputDimsCPU(FUNC, D1, D2, D3, THREADS)                     \
  static void BM_##FUNC##_##D1##x##D2##x##D3##_##THREADS##T(int iters, int N) {\
    StopBenchmarkTiming();                                                     \
    if (THREADS == 1) {                                                        \
      Eigen::DefaultDevice device;                                             \
      BenchmarkSuite<Eigen::DefaultDevice> suite(device, D1, D2, D3);          \
      suite.FUNC(iters);                                                       \
    } else {                                                                   \
      CREATE_THREAD_POOL(THREADS);                                             \
      BenchmarkSuite<Eigen::ThreadPoolDevice> suite(device, D1, D2, D3);       \
      suite.FUNC(iters);                                                       \
    }                                                                          \
    SetBenchmarkLabel(StrCat("using ", THREADS, " threads"));                  \
  }                                                                            \
  BENCHMARK_RANGE(BM_##FUNC##_##D1##x##D2##x##D3##_##THREADS##T, 10, 5000);


BM_FuncWithInputDimsCPU(contraction, N, N, N, 1);
BM_FuncWithInputDimsCPU(contraction, N, N, N, 4);
BM_FuncWithInputDimsCPU(contraction, N, N, N, 8);
BM_FuncWithInputDimsCPU(contraction, N, N, N, 12);
BM_FuncWithInputDimsCPU(contraction, N, N, N, 16);

BM_FuncWithInputDimsCPU(contraction, 64, N, N, 1);
BM_FuncWithInputDimsCPU(contraction, 64, N, N, 4);
BM_FuncWithInputDimsCPU(contraction, 64, N, N, 8);
BM_FuncWithInputDimsCPU(contraction, 64, N, N, 12);
BM_FuncWithInputDimsCPU(contraction, 64, N, N, 16);

BM_FuncWithInputDimsCPU(contraction, N, 64, N, 1);
BM_FuncWithInputDimsCPU(contraction, N, 64, N, 4);
BM_FuncWithInputDimsCPU(contraction, N, 64, N, 8);
BM_FuncWithInputDimsCPU(contraction, N, 64, N, 12);
BM_FuncWithInputDimsCPU(contraction, N, 64, N, 16);

BM_FuncWithInputDimsCPU(contraction, 1, N, N, 1);
BM_FuncWithInputDimsCPU(contraction, 1, N, N, 4);
BM_FuncWithInputDimsCPU(contraction, 1, N, N, 8);
BM_FuncWithInputDimsCPU(contraction, 1, N, N, 12);
BM_FuncWithInputDimsCPU(contraction, 1, N, N, 16);

BM_FuncWithInputDimsCPU(contraction, N, N, 1, 1);
BM_FuncWithInputDimsCPU(contraction, N, N, 1, 4);
BM_FuncWithInputDimsCPU(contraction, N, N, 1, 8);
BM_FuncWithInputDimsCPU(contraction, N, N, 1, 12);
BM_FuncWithInputDimsCPU(contraction, N, N, 1, 16);


// Convolutions
#define BM_FuncWithKernelDimsCPU(FUNC, DIM1, DIM2, THREADS)                    \
  static void BM_##FUNC##_##DIM1##x##DIM2##_##THREADS##T(int iters, int N) {   \
    StopBenchmarkTiming();                                                     \
    CREATE_THREAD_POOL(THREADS);                                               \
    BenchmarkSuite<Eigen::ThreadPoolDevice> suite(device, N);                  \
    suite.FUNC(iters, DIM1, DIM2);                                             \
    SetBenchmarkLabel(StrCat("using ", THREADS, " threads"));                  \
  }                                                                            \
  BENCHMARK_RANGE(BM_##FUNC##_##DIM1##x##DIM2##_##THREADS##T, 128, 5000);

BM_FuncWithKernelDimsCPU(convolution, 7, 1, 4);
BM_FuncWithKernelDimsCPU(convolution, 7, 1, 8);
BM_FuncWithKernelDimsCPU(convolution, 7, 1, 12);

BM_FuncWithKernelDimsCPU(convolution, 1, 7, 4);
BM_FuncWithKernelDimsCPU(convolution, 1, 7, 8);
BM_FuncWithKernelDimsCPU(convolution, 1, 7, 12);

BM_FuncWithKernelDimsCPU(convolution, 7, 4, 4);
BM_FuncWithKernelDimsCPU(convolution, 7, 4, 8);
BM_FuncWithKernelDimsCPU(convolution, 7, 4, 12);

BM_FuncWithKernelDimsCPU(convolution, 4, 7, 4);
BM_FuncWithKernelDimsCPU(convolution, 4, 7, 8);
BM_FuncWithKernelDimsCPU(convolution, 4, 7, 12);

BM_FuncWithKernelDimsCPU(convolution, 7, 64, 4);
BM_FuncWithKernelDimsCPU(convolution, 7, 64, 8);
BM_FuncWithKernelDimsCPU(convolution, 7, 64, 12);

BM_FuncWithKernelDimsCPU(convolution, 64, 7, 4);
BM_FuncWithKernelDimsCPU(convolution, 64, 7, 8);
BM_FuncWithKernelDimsCPU(convolution, 64, 7, 12);
