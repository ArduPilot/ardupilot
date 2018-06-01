/*
 * Utility header for benchmarks with Google Benchmark.
 */
#include <benchmark/benchmark.h>

/* The two functions below are an approach proposed by Chandler Carruth in
 * CPPCON 2015: CppCon 2015: "Tuning C++: Benchmarks, and CPUs, and Compilers!
 * Oh My!" in order keep the compiler from optimizing the use of a variable
 * (gbenchmark_escape) or whole memory (gbenchmark_clobber).
 *
 * The compiler optimizer may sometimes remove code when it sees it isn't
 * necessary. For example, when a variable isn't used, the optimizer removes
 * the code that computes the value for that variable - that's not good for
 * benchmarks. The function gbenchmark_escape(void *p) makes the compiler think
 * that that p is being used in a code that might have "unknowable side
 * effects", which keeps it from removing the variable. The "side effects" in
 * the case here would be the benchmark numbers.
 *
 * Here is an example that would give wrong benchmark values:
 *
 * static void BM_Test(benchmark::State& state)
 * {
 *     while (state.KeepRunning()) {
 *         float a = expensive_operation();
 *     }
 * }
 *
 * Since variable a isn't used, the call to expensive_operation() is removed
 * from the compiled program. The benchmark would show that
 * expensive_operation() is extremely fast. The following code would fix that:
 *
 * static void BM_Test(benchmark::State& state)
 * {
 *     while (state.KeepRunning()) {
 *         float a = expensive_operation();
 *         gbenchmark_escape(&a);
 *     }
 * }
 */

inline void gbenchmark_escape(void* p)
{
      asm volatile("" : : "g"(p) : "memory");
}

inline void gbenchmark_clobber()
{
    asm volatile("" : : : "memory");
}
