#define EIGEN_USE_GPU

#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include "strings/strcat.h"
#include "third_party/eigen3/tensor_benchmarks.h"



// Simple functions
#define BM_FuncGPU(FUNC)                                                       \
  static void BM_##FUNC(int iters, int N) {                                    \
    StopBenchmarkTiming();                                                     \
    cudaStream_t stream;                                                       \
    cudaStreamCreate(&stream);                                                 \
    Eigen::GpuDevice device(&stream);                                          \
    BenchmarkSuite<Eigen::GpuDevice> suite(device, N);                         \
    cudaDeviceSynchronize();                                                   \
    suite.FUNC(iters);                                                         \
    cudaStreamDestroy(stream);                                                 \
  }                                                                            \
  BENCHMARK_RANGE(BM_##FUNC, 10, 5000);

BM_FuncGPU(memcpy);
BM_FuncGPU(random);
BM_FuncGPU(slicing);
BM_FuncGPU(shuffling);
BM_FuncGPU(padding);
BM_FuncGPU(striding);
BM_FuncGPU(broadcasting);
BM_FuncGPU(coeffWiseOp);
BM_FuncGPU(reduction);


// Contractions
#define BM_FuncWithInputDimsGPU(FUNC, D1, D2, D3)                              \
  static void BM_##FUNC##_##D1##x##D2##x##D3(int iters, int N) {               \
    StopBenchmarkTiming();                                                     \
    cudaStream_t stream;                                                       \
    cudaStreamCreate(&stream);                                                 \
    Eigen::GpuDevice device(&stream);                                          \
    BenchmarkSuite<Eigen::GpuDevice> suite(device, D1, D2, D3);                \
    cudaDeviceSynchronize();                                                   \
    suite.FUNC(iters);                                                         \
    cudaStreamDestroy(stream);                                                 \
  }                                                                            \
  BENCHMARK_RANGE(BM_##FUNC##_##D1##x##D2##x##D3, 10, 5000);


BM_FuncWithInputDimsGPU(contraction, N, N, N);
BM_FuncWithInputDimsGPU(contraction, 64, N, N);
BM_FuncWithInputDimsGPU(contraction, N, 64, N);


// Convolutions
#define BM_FuncWithKernelDimsGPU(FUNC, DIM1, DIM2)                             \
  static void BM_##FUNC##_##DIM1##x##DIM2(int iters, int N) {                  \
    StopBenchmarkTiming();                                                     \
    cudaStream_t stream;                                                       \
    cudaStreamCreate(&stream);                                                 \
    Eigen::GpuDevice device(&stream);                                          \
    BenchmarkSuite<Eigen::GpuDevice> suite(device, N);                         \
    cudaDeviceSynchronize();                                                   \
    suite.FUNC(iters, DIM1, DIM2);                                             \
    cudaStreamDestroy(stream);                                                 \
  }                                                                            \
  BENCHMARK_RANGE(BM_##FUNC##_##DIM1##x##DIM2, 128, 5000);

BM_FuncWithKernelDimsGPU(convolution, 7, 1);
BM_FuncWithKernelDimsGPU(convolution, 1, 7);
BM_FuncWithKernelDimsGPU(convolution, 7, 4);
BM_FuncWithKernelDimsGPU(convolution, 4, 7);
BM_FuncWithKernelDimsGPU(convolution, 7, 64);
BM_FuncWithKernelDimsGPU(convolution, 64, 7);
