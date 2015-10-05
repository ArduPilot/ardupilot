#ifndef THIRD_PARTY_EIGEN3_TENSOR_BENCHMARKS_H_
#define THIRD_PARTY_EIGEN3_TENSOR_BENCHMARKS_H_

typedef int TensorIndex;
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int

#include "third_party/eigen3/unsupported/Eigen/CXX11/Tensor"
#include "testing/base/public/benchmark.h"

using Eigen::Tensor;
using Eigen::TensorMap;


// TODO(bsteiner): also templatize on the input type since we have users
// for int8 as well as floats.
template <typename Device> class BenchmarkSuite {
 public:
  BenchmarkSuite(const Device& device, size_t m, size_t k, size_t n)
      : m_(m), k_(k), n_(n), device_(device) {
    initialize();
  }

  BenchmarkSuite(const Device& device, size_t m)
      : m_(m), k_(m), n_(m), device_(device) {
    initialize();
  }

  ~BenchmarkSuite() {
    device_.deallocate(a_);
    device_.deallocate(b_);
    device_.deallocate(c_);
  }

  void memcpy(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      device_.memcpy(c_, a_, m_ * m_ * sizeof(float));
    }
    // Record the number of values copied per second
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  void random(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    const Eigen::array<TensorIndex, 2> sizes(m_, m_);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizes);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = C.random();
    }
    // Record the number of random numbers generated per second
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  void slicing(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    const Eigen::array<TensorIndex, 2> sizes(m_, m_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, sizes);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, sizes);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizes);

    const Eigen::DSizes<TensorIndex, 2> quarter_sizes(Eigen::array<TensorIndex, 2>(m_/2, m_/2));
    const Eigen::DSizes<TensorIndex, 2> first_quadrant(Eigen::array<TensorIndex, 2>(0, 0));
    const Eigen::DSizes<TensorIndex, 2> second_quadrant(Eigen::array<TensorIndex, 2>(0, m_/2));
    const Eigen::DSizes<TensorIndex, 2> third_quadrant(Eigen::array<TensorIndex, 2>(m_/2, 0));
    const Eigen::DSizes<TensorIndex, 2> fourth_quadrant(Eigen::array<TensorIndex, 2>(m_/2, m_/2));

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.slice(first_quadrant, quarter_sizes).device(device_) =
          A.slice(first_quadrant, quarter_sizes);
      C.slice(second_quadrant, quarter_sizes).device(device_) =
          B.slice(second_quadrant, quarter_sizes);
      C.slice(third_quadrant, quarter_sizes).device(device_) =
          A.slice(third_quadrant, quarter_sizes);
      C.slice(fourth_quadrant, quarter_sizes).device(device_) =
          B.slice(fourth_quadrant, quarter_sizes);
    }
    // Record the number of values copied from the rhs slice to the lhs slice
    // each second
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  void shuffling(int num_iters) {
    eigen_assert(m_ == n_);
    const Eigen::array<TensorIndex, 2> size_a(m_, k_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, size_a);
    const Eigen::array<TensorIndex, 2> size_b(k_, m_);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, size_b);

    const Eigen::array<int, 2> shuffle(1, 0);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      B.device(device_) = A.shuffle(shuffle);
    }
    // Record the number of values shuffled from A and copied to B each second
    finalizeBenchmark(m_ * k_ * num_iters);
  }

 void padding(int num_iters) {
    eigen_assert(m_ == k_);
    const Eigen::array<TensorIndex, 2> size_a(m_, k_-3);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, size_a);
    const Eigen::array<TensorIndex, 2> size_b(k_, m_);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, size_b);

    Eigen::array<Eigen::IndexPair<TensorIndex>, 2> paddings;
    paddings[0] = Eigen::IndexPair<TensorIndex>(0, 0);
    paddings[1] = Eigen::IndexPair<TensorIndex>(2, 1);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      B.device(device_) = A.pad(paddings);
    }
    // Record the number of values copied from the padded tensor A each second
    finalizeBenchmark(m_ * k_ * num_iters);
  }

 void striding(int num_iters) {
    eigen_assert(m_ == k_);
    const Eigen::array<TensorIndex, 2> size_a(m_, k_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, size_a);
    const Eigen::array<TensorIndex, 2> size_b(m_, k_ / 2);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, size_b);

    const Eigen::array<TensorIndex, 2> strides(1, 2);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      B.device(device_) = A.stride(strides);
    }
    // Record the number of values copied from the padded tensor A each second
    finalizeBenchmark(m_ * k_ * num_iters);
  }

  void broadcasting(int num_iters) {
    const Eigen::array<TensorIndex, 2> size_a(m_, 1);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, size_a);
    const Eigen::array<TensorIndex, 2> size_c(m_, n_);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, size_c);

#if defined(__CUDACC__)
    // nvcc doesn't support cxx11
    const Eigen::array<int, 2> broadcast(1, n_);
#else
    // Take advantage of cxx11 to give the compiler information it can use to
    // optimize the code.
    Eigen::IndexList<Eigen::type2index<1>, int> broadcast;
    broadcast.set(1, n_);
#endif

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A.broadcast(broadcast);
    }
    // Record the number of values broadcasted from A and copied to C each second
    finalizeBenchmark(m_ * n_ * num_iters);
  }

  void coeffWiseOp(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    const Eigen::array<TensorIndex, 2> sizes(m_, m_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, sizes);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, sizes);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizes);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A * A.constant(3.14) + B * B.constant(2.7);
    }
    // Record the number of FLOP executed per second (2 multiplications and
    // 1 addition per value)
    finalizeBenchmark(3 * m_ * m_ * num_iters);
  }

  void algebraicFunc(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    const Eigen::array<TensorIndex, 2> sizes(m_, m_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, sizes);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, sizes);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizes);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A.rsqrt() + B.sqrt() * B.square();
    }
    // Record the number of FLOP executed per second (assuming one operation
    // per value)
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  void transcendentalFunc(int num_iters) {
    eigen_assert(m_ == k_ && k_ == n_);
    const Eigen::array<TensorIndex, 2> sizes(m_, m_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, sizes);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, sizes);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizes);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A.exp() + B.log();
    }
    // Record the number of FLOP executed per second (assuming one operation
    // per value)
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  // Simple reduction
  void reduction(int num_iters) {
    const Eigen::array<TensorIndex, 2> input_size(k_, n_);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, input_size);
    const Eigen::array<TensorIndex, 1> output_size(n_);
    TensorMap<Tensor<float, 1>, Eigen::Aligned> C(c_, output_size);

    const Eigen::array<TensorIndex, 1> sum_along_dim(0);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = B.sum(sum_along_dim);
    }
    // Record the number of FLOP executed per second (assuming one operation
    // per value)
    finalizeBenchmark(m_ * m_ * num_iters);
  }

  // do a contraction which is equivalent to a matrix multiplication
  void contraction(int num_iters) {
    const Eigen::array<TensorIndex, 2> sizeA(m_, k_);
    const Eigen::array<TensorIndex, 2> sizeB(k_, n_);
    const Eigen::array<TensorIndex, 2> sizeC(m_, n_);

    const TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, sizeA);
    const TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, sizeB);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, sizeC);

    typedef typename Tensor<float, 2>::DimensionPair DimPair;
    const Eigen::array<DimPair, 1> dims(DimPair(1, 0));

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A.contract(B, dims);
    }
    // Record the number of FLOP executed per second (size_ multiplications and
    // additions for each value in the resulting tensor)
    finalizeBenchmark(static_cast<int64>(2) * m_ * n_ * k_ * num_iters);
  }

  void convolution(int num_iters, int kernel_x, int kernel_y) {
    const Eigen::array<TensorIndex, 2> input_sizes(m_, n_);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> A(a_, input_sizes);
    const Eigen::array<TensorIndex, 2> kernel_sizes(kernel_x, kernel_y);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> B(b_, kernel_sizes);
    const Eigen::array<TensorIndex, 2> result_sizes(
        m_ - kernel_x + 1, n_ - kernel_y + 1);
    TensorMap<Tensor<float, 2>, Eigen::Aligned> C(c_, result_sizes);
    Eigen::array<Tensor<float, 2>::Index, 2> dims(0, 1);

    StartBenchmarkTiming();
    for (int iter = 0; iter < num_iters; ++iter) {
      C.device(device_) = A.convolve(B, dims);
    }
    // Record the number of FLOP executed per second (kernel_size
    // multiplications and additions for each value in the resulting tensor)
    finalizeBenchmark(
        (m_ - kernel_x + 1) * (n_ - kernel_y + 1) * kernel_x * kernel_y * 2 * num_iters);
  }

 private:
  void initialize() {
    a_ = (float *) device_.allocate(m_ * k_ * sizeof(float));
    b_ = (float *) device_.allocate(k_ * n_ * sizeof(float));
    c_ = (float *) device_.allocate(m_ * n_ * sizeof(float));

    // Initialize the content of the memory pools to prevent asan from
    // complaining.
    device_.memset(a_, 12, m_ * k_ * sizeof(float));
    device_.memset(b_, 23, k_ * n_ * sizeof(float));
    device_.memset(c_, 31, m_ * n_ * sizeof(float));

    BenchmarkUseRealTime();
  }

  inline void finalizeBenchmark(int64 num_items) {
#if defined(EIGEN_USE_GPU) && defined(__CUDACC__)
    if (Eigen::internal::is_same<Device, Eigen::GpuDevice>::value) {
      device_.synchronize();
    }
#endif
    StopBenchmarkTiming();
    SetBenchmarkItemsProcessed(num_items);
  }


  size_t m_;
  size_t k_;
  size_t n_;
  float* a_;
  float* b_;
  float* c_;
  Device device_;
};
#endif  // THIRD_PARTY_EIGEN3_TENSOR_BENCHMARKS_H_
